/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ASTUtils.factory
import org.lflang.generator.KtGeneratorBase
import org.lflang.lf.*


fun ReactorDecl.toDefinition(): Reactor = when (this) {
    is Reactor         -> this
    is ImportedReactor -> this.reactorClass
    else               -> throw AssertionError("unreachable")
}

/**
 * Given a reactor class, return a list of all its actions,
 * which includes actions of base classes that it extends.
 */
val Reactor.allActions: List<Action> get() = superClassRecursor { actions }

/**
 * Given a reactor class, return a list of all its connections,
 * which includes connections of base classes that it extends.
 */
val Reactor.allConnections: List<Connection> get() = superClassRecursor { connections }

/**
 * Given a reactor class, return a list of all its inputs,
 * which includes inputs of base classes that it extends.
 */
val Reactor.allInputs: List<Input> get() = superClassRecursor { inputs }

/**
 * Given a reactor class, return a list of all its outputs,
 * which includes outputs of base classes that it extends.
 */
val Reactor.allOutputs: List<Output> get() = superClassRecursor { outputs }

/**
 * Given a reactor class, return a list of all its instantiations,
 * which includes instantiations of base classes that it extends.
 */
val Reactor.allInstantiations: List<Instantiation> get() = superClassRecursor { instantiations }

/**
 * Given a reactor class, return a list of all its parameters,
 * which includes parameters of base classes that it extends.
 */
val Reactor.allParameters: List<Parameter> get() = superClassRecursor { parameters }

/**
 * Given a reactor class, return a list of all its reactions,
 * which includes reactions of base classes that it extends.
 */
val Reactor.allReactions: List<Reaction> get() = superClassRecursor { reactions }

/**
 * Given a reactor class, return a list of all its state variables,
 * which includes state variables of base classes that it extends.
 */
val Reactor.allStateVars: List<StateVar> get() = superClassRecursor { stateVars }

/**
 * Given a reactor class, return a list of all its  timers,
 * which includes timers of base classes that it extends.
 */
val Reactor.allTimers: List<Timer> get() = superClassRecursor { timers }

private fun <T> Reactor.superClassRecursor(collector: Reactor.() -> List<T>): List<T> =
    superClasses.orEmpty().mapNotNull { it.toDefinition().collector() }.flatten() + this.collector()

val Parameter.isOfTimeType: Boolean
    get() {
        // Either the type has to be declared as a time.
        if (type?.isTime == true) {
            return true
        }
        // Or it has to be initialized as a proper time with units.
        init?.singleOrNull()
            ?.time
            ?.takeIf { it.unit != TimeUnit.NONE }
            ?.let {
                return true
            }
        // In other words, one can write:
        // - `x:time(0)` -OR- 
        // - `x:(0 msec)`, `x:(0 sec)`, etc.     
        return false
    }

fun <T> List<T>.tail() = subList(1, size)
fun <T> List<T>.headAndTail() = Pair(first(), tail())


fun Code?.toText(): String {
    this ?: return ""
    val node = NodeModelUtils.getNode(this)
    return if (node != null) {
        val str = node.leafNodes
            .joinToString { it.text }.trim()
            .removeSurrounding("{=", "=}")

        if ('\n' in str) str.trimIndent() else str.trim()
    } else if (body != null) {
        // Code must have been added as a simple string.
        body.toString()
    } else {
        ""
    }
}

fun TypeParm.toText(): String =
    if (!literal.isNullOrEmpty()) literal
    else code.toText()


/**
 * Return a textual representation of this element,
 * without quotes if there are any. Leading or trailing
 * whitespace is removed.
 *
 * @receiver The element to be rendered as a string.
 */
fun Element.toText(): String {
    if (literal != null) {
        return literal.withoutQuotes().trim()
    }
    if (id != null) {
        return id
    }
    return ""
}


fun Delay.toText(): String {
    if (parameter !== null) {
        return parameter.name
    }
    return interval.toString() + " " + unit
}

/**
 * Remove quotation marks surrounding the specified string.
 */
fun String.withoutQuotes(): String {
    val r = removeSurrounding("\"")
    return if (r !== this) this else removeSurrounding("'")
}


/**
 * Return a string of the form either "name" or "container.name" depending
 * on in which form the variable reference was given.
 * @param this@toText The variable reference.
 */
fun VarRef.toText(): String =
    if (container !== null) {
        "${container.name}.${variable.name}"
    } else {
        variable.name
    }

/**
 * Convert an array specification to its textual representation as it would
 * appear in LF code.
 *
 * @param this@toText The array spec to be converted
 * @return A textual representation
 */
fun ArraySpec.toText(): String =
    if (isOfVariableLength) "[]"
    else "[$length]"


/**
 * Translate the given type into its textual representation, including
 * any array specifications.
 * @param type AST node to render as string.
 * @return Textual representation of the given argument.
 */
fun toText(type: Type): String {
    return type.baseType + type.arraySpec?.toText().orEmpty()
}

/**
 * Find connections in the given resource that have a delay associated with them,
 * and reroute them via a generated delay reactor.
 * @param resource The AST.
 * @param generator A code generator.
 */
fun insertGeneratedDelays(resource: Resource, generator: KtGeneratorBase) {
    // The resulting changes to the AST are performed _after_ iterating
    // in order to avoid concurrent modification problems.
    val oldConnections = mutableListOf<Connection>()
    val newConnections = mutableMapOf<Reactor, MutableList<Connection>>()
    val delayInstances = mutableMapOf<Reactor, MutableList<Instantiation>>()

    // Iterate over the connections in the tree.
    for (container in resource.allContents.asSequence().filterIsInstance<Reactor>()) {
        for (connection in container.connections) {
            if (connection.delay !== null) {
                val parent = connection.eContainer() as Reactor
                // Assume all the types are the same, so just use the first on the right.
                val type = (connection.rightPorts[0].variable as Port).type
                val delayClass = getDelayClass(type, generator)
                val generic =
                    if (generator.supportsGenerics)
                        generator.getTargetType(InferredType.fromAST(type))
                    else ""

                // If the left or right has a multiport or bank, then create a bank
                // of delays with an inferred width.
                // FIXME: If the connection already uses an inferred width on
                // the left or right, then this will fail because you cannot
                // have an inferred width on both sides.
                val isWide = connection.isWide
                val delayInstance = getDelayInstance(delayClass, connection.delay, generic, isWide)

                // Stage the new connections for insertion into the tree.
                newConnections.computeIfAbsent(parent) { mutableListOf() }
                    .addAll(connection.rerouteViaDelay(delayInstance))

                // Stage the original connection for deletion from the tree.
                oldConnections.add(connection)

                // Stage the newly created delay reactor instance for insertion
                delayInstances.computeIfAbsent(parent) { mutableListOf() }
                    .add(delayInstance)
            }
        }
    }
    // Remove old connections; insert new ones.
    for (connection in oldConnections) {
        (connection.eContainer() as Reactor).connections.remove(connection)
    }
    for ((reactor, connections) in newConnections) {
        reactor.connections.addAll(connections)
    }
    // Finally, insert the instances and, before doing so, assign them a unique name.
    for ((reactor, instantiations) in delayInstances) {
        for (instantiation in instantiations) {
            instantiation.name = reactor.getUniqueIdentifier("delay")
            reactor.instantiations.add(instantiation)
        }
    }
}


/**
 * Return true if any port on the left or right of this connection involves
 * a bank of reactors or a multiport.
 */
private val Connection.isWide: Boolean
    get() {
        val allPorts = leftPorts + rightPorts
        return allPorts.any { port ->
            (port.variable as? Port)?.widthSpec != null
                    || port.container?.widthSpec != null
        }
    }


/**
 * Take a connection and reroute it via an instance of a generated delay
 * reactor. This method returns a list to new connections to substitute
 * the original one.
 * @receiver The connection to reroute.
 * @param delayInstance The delay instance to route the connection through.
 */
private fun Connection.rerouteViaDelay(delayInstance: Instantiation): List<Connection> {
    val connections = mutableListOf<Connection>()

    val upstream = factory.createConnection()
    val downstream = factory.createConnection()
    val input = factory.createVarRef()
    val output = factory.createVarRef()

    val delayClass = delayInstance.reactorClass.toDefinition()

    // Establish references to the involved ports.
    input.container = delayInstance
    input.variable = delayClass.inputs[0]
    output.container = delayInstance
    output.variable = delayClass.outputs[0]
    upstream.leftPorts.addAll(leftPorts)
    upstream.rightPorts.add(input)
    downstream.leftPorts.add(output)
    downstream.rightPorts.addAll(rightPorts)

    connections.add(upstream)
    connections.add(downstream)
    return connections
}

/**
 * Produce a unique identifier within a reactor based on a
 * given based name. If the name does not exists, it is returned;
 * if does exist, an index is appended that makes the name unique.
 * @receiver The reactor to find a unique identifier within.
 * @param name The name to base the returned identifier on.
 */
fun Reactor.getUniqueIdentifier(name: String): String {
    val vars = mutableSetOf<String>().apply {
        addAll(allActions.map { it.name })
        addAll(allTimers.map { it.name })
        addAll(allParameters.map { it.name })
        addAll(allInputs.map { it.name })
        addAll(allOutputs.map { it.name })
        addAll(allStateVars.map { it.name })
        addAll(allInstantiations.map { it.name })
    }

    var index = 0
    var suffix = ""
    while (true) {
        val id = name + suffix
        if (id in vars) {
            // already exists
            suffix = "_$index"
            index++
        } else {
            break
        }
    }
    return name + suffix
}

/**
 * Create a new instance delay instances using the given reactor class.
 * The supplied time value is used to override the default interval (which
 * is zero).
 * If the target supports parametric polymorphism, then a single class may
 * be used for each instantiation, in which case a non-empty string must
 * be supplied to parameterize the instance.
 * A default name ("delay") is assigned to the instantiation, but this
 * name must be overridden at the call site, where checks can be done to
 * avoid name collisions in the container in which the instantiation is
 * to be placed. Such checks (or modifications of the AST) are not
 * performed in this method in order to avoid causing concurrent
 * modification exceptions.
 * @param delayClass The class to create an instantiation for
 * @param value A time interval corresponding to the desired delay
 * @param generic A string that denotes the appropriate type parameter,
 *  which should be null or empty if the target does not support generics.
 * @param isWide True to create a variable-width width specification.
 */
private fun getDelayInstance(delayClass: Reactor, delay: Delay, generic: String, isWide: Boolean): Instantiation {
    val delayInstance = factory.createInstantiation().apply {
        reactorClass = delayClass
    }
    if (generic.isNotEmpty()) {
        val typeParm = factory.createTypeParm().apply { literal = generic }
        delayInstance.typeParms.add(typeParm)
    }
    if (isWide) {
        val widthSpec = factory.createWidthSpec()
        delayInstance.widthSpec = widthSpec
        widthSpec.isOfVariableLength = true
    }
    val assignment = factory.createAssignment()
    assignment.lhs = delayClass.parameters[0]
    val value = factory.createValue().apply {
        if (delay.parameter !== null) {
            parameter = delay.parameter
        } else {
            time = factory.createTime()
            time.interval = delay.interval
            time.unit = delay.unit
        }
    }
    assignment.rhs.add(value)
    delayInstance.parameters.add(assignment)
    delayInstance.name = "delay" // This has to be overridden.

    return delayInstance

}

/**
 * Return a synthesized AST node that represents the definition of a delay
 * reactor. Depending on whether the target supports generics, either this
 * method will synthesize a generic definition and keep returning it upon
 * subsequent calls, or otherwise, it will synthesize a new definition for
 * each new type it hasn't yet created a compatible delay reactor for.
 * @param aType The type the delay class must be compatible with.
 * @param generator A code generator.
 */
private fun getDelayClass(aType: Type, generator: KtGeneratorBase): Reactor {
    val className =
        if (generator.supportsGenerics) KtGeneratorBase.GEN_DELAY_CLASS_NAME
        else run {
            val id = Integer.toHexString(InferredType.fromAST(aType).toText().hashCode())
            "${KtGeneratorBase.GEN_DELAY_CLASS_NAME}_$id"
        }

    // If it is there, return it
    generator.findDelayClass(className)?.let { return it }

    val defaultValue = factory.createValue().apply {
        literal = generator.timeInTargetLanguage(TimeValue(0, TimeUnit.NONE))
    }

    val delayParameter = factory.createParameter().apply {
        name = "delay"
        type = factory.createType()
        aType.id = generator.targetTimeType
        init.add(defaultValue)
    }
    val action = factory.createAction().apply {
        name = "act"
        minDelay = factory.createValue()
        minDelay.parameter = delayParameter
        origin = ActionOrigin.LOGICAL
    }

    // Establish references to the action.
    val triggerRef = factory.createVarRef().apply { variable = action }
    val effectRef = factory.createVarRef().apply { variable = action }
    val input = factory.createInput().apply {
        name = "inp"
        type = action.type.getCopy()
    }
    val output = factory.createOutput().apply {
        name = "out"
        type = action.type.getCopy()
    }

    // Establish references to the involved ports.
    val inRef = factory.createVarRef().apply {
        variable = input
    }
    val outRef = factory.createVarRef().apply {
        variable = output
    }


    // Name the newly created action; set its delay and type.

    if (generator.supportsGenerics) {
        action.type = factory.createType()
        action.type.id = "T"
    } else {
        action.type = aType.getCopy()
    }

    val r1 = factory.createReaction().apply {
        // Configure the second reaction, which reads the input.
        triggers.add(inRef)
        effects.add(effectRef)
        code = factory.createCode()
        code.body = generator.generateDelayBody(action, inRef)
    }

    val r2 = factory.createReaction().apply {
        // Configure the first reaction, which produces the output.
        triggers.add(triggerRef)
        effects.add(outRef)
        code = factory.createCode()
        code.body = generator.generateForwardBody(action, outRef)
    }

    val delayClass = factory.createReactor().apply {
        name = className
        // Add the action to the reactor.
        actions += action

        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        reactions += r2
        reactions += r1

        inputs += input
        outputs += output
        parameters += delayParameter

        // Add a type parameter if the target supports it.
        if (generator.supportsGenerics) {
            typeParms += factory.createTypeParm().apply {
                literal = generator.generateDelayGeneric()
            }
        }
    }

    generator.addDelayClass(delayClass)

    return delayClass
}

private fun TypeParm.getCopy(): TypeParm {
    val original = this
    return factory.createTypeParm().apply {
        literal = original.literal
        code = factory.createCode().apply { body = original.code.body }
    }
}

private fun Type.getCopy(): Type {
    val original = this
    return factory.createType().apply {
        id = original.id
        isTime = original.isTime
        stars.addAll(original.stars.orEmpty())

        if (original.code !== null) {
            code = factory.createCode().apply {
                body = original.code.body
            }
        }
        if (original.arraySpec !== null) {
            arraySpec = factory.createArraySpec().apply {
                this.isOfVariableLength = original.arraySpec.isOfVariableLength
                length = original.arraySpec.length
            }
        }

        typeParms.addAll(original.typeParms.orEmpty().map { it.getCopy() })
    }

}

/**
 * Translate the given type into its textual representation, but
 * do not append any array specifications.
 * @receiver AST node to render as string.
 * @return Textual representation of the given argument.
 */
val Type.baseType: String
    get() = when {
        code != null -> code.toText()
        isTime       -> "time"
        else         -> id + stars.orEmpty().joinToString()
    }

/**
 * Report whether the given literal is zero or not.
 * @receiver AST node to inspect.
 * @return True if the given literal denotes the constant `0`, false
 * otherwise.
 */
val String.isZero: Boolean get() = this.toIntOrNull() == 0

val Code.isZero: Boolean get() = this.toText().isZero

/**
 * Report whether the given value is zero or not.
 * @param value AST node to inspect.
 * @return True if the given value denotes the constant `0`, false otherwise.
 */
fun isZero(value: Value): Boolean =
    value.literal?.isZero
        ?: value.code?.isZero
        ?: false

