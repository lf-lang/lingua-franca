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
import java.lang.NumberFormatException


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
 * @receiver The variable reference.
 */
fun VarRef.toText(): String =
    if (container !== null) "${container.name}.${variable.name}"
    else variable.name


/**
 * Convert a value to its textual representation as it would
 * appear in LF code.
 *
 * @receiver The value to be converted
 * @return A textual representation
 */
fun Value.toText(): String =
    parameter?.name
        ?: time?.toText()
        ?: literal
        ?: code?.toText()
        ?: ""


/**
 * Convert a time to its textual representation as it would
 * appear in LF code.
 * @receiver The time to be converted
 */
fun Time.toText(): String = "$interval $unit"


/**
 * Convert an array specification to its textual representation as it would
 * appear in LF code.
 *
 * @receiver The array spec to be converted
 * @return A textual representation
 */
fun ArraySpec.toText(): String =
    if (isOfVariableLength) "[]"
    else "[$length]"


/**
 * Translate the given type into its textual representation, including
 * any array specifications.
 * @receiver AST node to render as string.
 * @return Textual representation of the given argument.
 */
fun Type.toText(): String {
    return baseType + arraySpec?.toText().orEmpty()
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


/**
 * Given the width specification of port or instantiation
 * and an (optional) list of nested intantiations, return
 * the width if it can be determined and -1 if not.
 * It will not be able to be determined if either the
 * width is variable (in which case you should use
 * {@link inferPortWidth(VarRef, Connection, List<Instantiation>})
 * or the list of instantiations is incomplete or missing.
 * If there are parameter references in the width, they are
 * evaluated to the extent possible given the instantiations list.
 *
 * The instantiations list is as in
 * {@link initialValue(Parameter, List<Instantiation>}.
 * If the spec belongs to an instantiation (for a bank of reactors),
 * then the first element on this list should be the instantiation
 * that contains this instantiation. If the spec belongs to a port,
 * then the first element on the list should be the instantiation
 * of the reactor that contains the port.
 *
 * @param spec The width specification or null (to return 1).
 * @param instantiations The (optional) list of instantiations.
 *
 * @return The width, or null if the width could not be determined.
 *
 * @throws IllegalArgumentException If an instantiation provided is not as
 *  given above or if the chain of instantiations is not nested.
 */
fun width(spec: WidthSpec, instantiations: List<Instantiation> = emptyList()): Int? {
    if (spec.isOfVariableLength && spec.eContainer() is Instantiation) {
        // We may be able to infer the width by examining the connections of
        // the enclosing reactor definition. This works, for example, with
        // delays between multiports or banks of reactors.
        // Attempt to infer the width.
        for (c in (spec.eContainer().eContainer() as Reactor).connections) {
            var leftWidth = 0
            var rightWidth = 0
            var isOnLeft: Boolean? = null
            for (leftPort in c.leftPorts) {
                if (leftPort.container === spec.eContainer()) {
                    if (isOnLeft != null) {
                        throw Exception("Multiple ports with variable width on a connection.")
                    }
                    isOnLeft = true
                } else {
                    leftWidth += inferPortWidth(leftPort, c) ?: return null
                }
            }
            for (rightPort in c.rightPorts) {
                if (rightPort.container === spec.eContainer()) {
                    if (isOnLeft != null) {
                        throw  Exception("Multiple ports with variable width on a connection.")
                    }
                    isOnLeft = false
                } else {
                    rightWidth += inferPortWidth(rightPort, c) ?: return null
                }
            }

            if (isOnLeft != null) {
                return if (isOnLeft) rightWidth - leftWidth else leftWidth - rightWidth
            }
        }
        // A connection was not found with the instantiation.
        return null
    }
    var result = 0
    for (term in spec.terms) {
        if (term.parameter !== null) {
            val termWidth = initialValueInt(term.parameter, instantiations)
            if (termWidth !== null) {
                result += termWidth
            } else {
                return -1
            }
        } else {
            result += term.width
        }
    }
    return result
}

/**
 * Infer the width of a port reference in a connection.
 * The port reference one or two parts, a port and an (optional) container
 * which is an Instantiation that may refer to a bank of reactors.
 * The width will be the product of the bank width and the port width.
 * The returned value will be 1 if the port is not in a bank and is not a multiport.
 *
 * If the width cannot be determined, this will return -1.
 * The width cannot be determined if the list of instantiations is
 * missing or incomplete.
 *
 * The instantiations list is as in
 * {@link initialValue(Parameter, List<Instantiation>}.
 * The first element on this list should be the instantiation
 * that contains the specified connection.
 *
 * @param reference A port reference.
 * @param connection A connection, or null if not in the context of a connection.
 * @param instantiations The (optional) list of instantiations.
 *
 * @return The width or null if it could not be determined.
 *
 * @throws IllegalArgumentException If an instantiation provided is not as
 *  given above or if the chain of instantiations is not nested.
 */
fun inferPortWidth(reference: VarRef, connection: Connection?, instantiations: List<Instantiation> = emptyList()): Int? {
    if (reference.variable is Port) {
        // If the port is given as a.b, then we want to prepend a to
        // the list of instantiations to determine the width of this port.
        var extended = instantiations
        if (reference.container !== null) {
            extended = mutableListOf()
            extended.add(reference.container)
            extended.addAll(instantiations)
        }

        val portWidth = width((reference.variable as Port).widthSpec, extended) ?: return null

        // Next determine the bank width. This may be unspecified, in which
        // case it has to be inferred using the connection.
        val bankWidth: Int =
            if (reference.container == null || instantiations == null) {
                1
            } else {
                // Could not determine the bank width.// This is an error.// This port is on the right.
                // Check that portWidth divides the discrepancy.
                // This port is on the left.// Indicate that this port is on the right.// The left port is not the same as this reference.// Indicate that this port is on the left.// This occurs for a bank of delays.// width returned null
                // Try to infer the bank width from the connection.
                width(reference.container.widthSpec, instantiations)
                    ?: if (connection == null) return null
                    else { // width returned null, no connection

                        // Try to infer the bank width from the connection.
                        if (reference.container.widthSpec.isOfVariableLength) {
                            // This occurs for a bank of delays.
                            var leftWidth = 0
                            var rightWidth = 0
                            var leftOrRight = 0
                            for (leftPort in connection.leftPorts) {
                                if (leftPort === reference) {
                                    if (leftOrRight != 0) {
                                        throw Exception("Multiple ports with variable width on a connection.")
                                    }
                                    // Indicate that this port is on the left.
                                    leftOrRight = -1
                                } else {
                                    // The left port is not the same as this reference.
                                    val otherWidth = inferPortWidth(leftPort, connection, instantiations) ?: return null
                                    leftWidth += otherWidth
                                }
                            }
                            for (rightPort in connection.rightPorts) {
                                if (rightPort === reference) {
                                    if (leftOrRight != 0) {
                                        throw Exception("Multiple ports with variable width on a connection.")
                                    }
                                    // Indicate that this port is on the right.
                                    leftOrRight = 1
                                } else {
                                    val otherWidth = inferPortWidth(rightPort, connection, instantiations) ?: return null
                                    rightWidth += otherWidth
                                }
                            }
                            var discrepancy = 0
                            if (leftOrRight < 0) {
                                // This port is on the left.
                                discrepancy = rightWidth - leftWidth
                            } else if (leftOrRight > 0) {
                                // This port is on the right.
                                discrepancy = leftWidth - rightWidth
                            }
                            // Check that portWidth divides the discrepancy.
                            if (discrepancy % portWidth != 0) {
                                return null // This is an error.
                            }
                            discrepancy / portWidth
                        } else {
                            return null // Could not determine the bank width.
                        }
                    }
            }
        return portWidth * bankWidth
    }
    // Argument is not a port.
    return null
}

/**
 * Return the width of the port reference if it can be determined
 * and otherwise return -1.  The width can be determined if the
 * port is not a multiport in a bank of reactors (the width will 1)
 * or if the width of the multiport and/or the bank is given by a
 * literal constant.
 *
 * IMPORTANT: This method should not be used you really need to
 * determine the width! It will not evaluate parameter values.
 * @see width(WidthSpec, List<Instantiation> instantiations)
 * @see inferPortWidth(VarRef, Connection, List<Instantiation>)
 *
 * @param reference A reference to a port.
 * @return The width of a port or null if it cannot be determined.
 * @deprecated
 */
fun multiportWidthIfLiteral(reference: VarRef): Int? = inferPortWidth(reference, null, instantiations = emptyList())

/**
 * Given the specification of the width of either a bank of reactors
 * or a multiport, return the width if it can be determined and otherwise
 * return -1. The width can be determined if it is given by one or more
 * literal constants or if the widthSpec is null (it is not a multiport
 * or reactor bank).
 *
 * IMPORTANT: This method should not be used you really need to
 * determine the width! It will not evaluate parameter values.
 *
 * @receiver The width specification.
 *
 * @return The width or null if it cannot be determined.
 * @deprecated
 */
val WidthSpec.width: Int?
    get() = width(this)

inline fun <reified T : Int?> WidthSpec.width(default: T): T = width(this) as? T ?: default


/**
 * Given an instantiation of a reactor or bank of reactors, return
 * the width. This will be 1 if this is not a reactor bank. Otherwise,
 * this will attempt to determine the width. If the width is declared
 * as a literal constant, it will return that constant. If the width
 * is specified as a reference to a parameter, this will throw an
 * exception. If the width is variable, this will find
 * connections in the enclosing reactor and attempt to infer the
 * width. If the width cannot be determined, it will throw an exception.
 *
 * IMPORTANT: This method should not be used you really need to
 * determine the width! It will not evaluate parameter values.
 * @see width(WidthSpec, List<Instantiation> instantiations)
 *
 * @receiver A reactor instantiation.
 *
 * @return The width, if it can be determined.
 * @throws RuntimeException If the width cannot be determined
 */
fun Instantiation.widthSpecification(): Int =
    width(widthSpec) ?: throw Exception("Cannot determine width for the instance $name")


/**
 * Given a parameter, return its initial value.
 * The initial value is a list of instances of Value, where each
 * Value is either an instance of Time, Literal, or Code.
 *
 * If the instantiations argument is null or an empty list, then the
 * value returned is simply the default value given when the parameter
 * is defined.
 *
 * If a list of instantiations is given, then the first instantiation
 * is required to be an instantiation of the reactor class that is
 * parameterized by the parameter. I.e.,
 * ```
 *     parameter.eContainer == instantiations.get(0).reactorClass
 * ```
 * If a second instantiation is given, then it is required to be an instantiation of a
 * reactor class that contains the first instantiation.  That is,
 * ```
 *     instantiations.get(0).eContainer == instantiations.get(1).reactorClass
 * ```
 * More generally, for all 0 <= i < instantiations.size - 1,
 * ```
 *     instantiations.get(i).eContainer == instantiations.get(i + 1).reactorClass
 * ```
 * If any of these conditions is not satisfied, then an IllegalArgumentException
 * will be thrown.
 *
 * Note that this chain of reactions cannot be inferred from the parameter because
 * in each of the predicates above, there may be more than one instantiation that
 * can appear on the right hand side of the predicate.
 *
 * For example, consider the following program:
 * ```
 *     reactor A(x:int(1)) {}
 *     reactor B(y:int(2)) {
 *         a1 = new A(x = y);
 *         a2 = new A(x = -1);
 *     }
 *     reactor C(z:int(3)) {
 *         b1 = new B(y = z);
 *         b2 = new B(y = -2);
 *     }
 * ```
 * Notice that there are a total of four instances of reactor class A.
 * Then
 * ```
 *     initialValue(x, null) returns 1
 *     initialValue(x, [a1]) returns 2
 *     initialValue(x, [a2]) returns -1
 *     initialValue(x, [a1, b1]) returns 3
 *     initialValue(x, [a2, b1]) returns -1
 *     initialValue(x, [a1, b2]) returns -2
 *     initialValue(x, [a2, b2]) returns -1
 * ```
 * (Actually, in each of the above cases, the returned value is a list with
 * one entry, a Literal, e.g. ["1"]).
 *
 * There are two instances of reactor class B.
 * ```
 *     initialValue(y, null) returns 2
 *     initialValue(y, [a1]) throws an IllegalArgumentException
 *     initialValue(y, [b1]) returns 3
 *     initialValue(y, [b2]) returns -2
 * ```
 *
 * @param parameter The parameter.
 * @param instantiations The (optional) instantiations.
 *
 * @return The value of the parameter.
 *
 * @throws IllegalArgumentException If an instantiation provided is not an
 *  instantiation of the reactor class that is parameterized by the
 *  respective parameter or if the chain of instantiations is not nested.
 */
fun initialValue(parameter: Parameter, instantiations: List<Instantiation> = emptyList()): List<Value> {
    // If instantiations are given, then check to see whether this parameter gets overridden in
    // the first of those instantiations.
    if (instantiations.isNotEmpty()) {
        // Check to be sure that the instantiation is in fact an instantiation
        // of the reactor class for which this is a parameter.
        val instantiation = instantiations[0]
        if (parameter.eContainer() !== instantiation.reactorClass) {
            throw IllegalArgumentException("Parameter ${parameter.name} is not a parameter of reactor instance ${instantiation.name}.")
        }
        // In case there is more than one assignment to this parameter, we need to
        // find the last one.
        val lastAssignment = instantiation.parameters.lastOrNull { it.lhs === parameter }
        if (lastAssignment != null) {
            // Right hand side can be a list. Collect the entries.
            val result = mutableListOf<Value>()
            for (value in lastAssignment.rhs) {
                if (value.parameter !== null) {
                    if (instantiations.size > 1
                        && instantiation.eContainer() !== instantiations[1].reactorClass
                    ) {
                        throw IllegalArgumentException("Reactor instance ${instantiation.name} is not contained by instance ${instantiations[1].name}.")
                    }
                    val elements = initialValue(value.parameter, instantiations.subList(1, instantiations.size))
                    result.addAll(elements)
                } else {
                    result.add(value)
                }
            }
            return result
        }
    }
    // If we reach here, then either no instantiation was supplied or
    // there was no assignment in the instantiation. So just use the
    // parameter's initial value.
    return parameter.init
}

/**
 * Given a parameter return its integer value or null
 * if it does not have an integer value.
 * If the value of the parameter is a list of integers,
 * return the sum of value in the list.
 * The instantiations parameter is as in
 * {@link initialValue(Parameter, List<Instantiation>}.
 *
 * @param parameter The parameter.
 * @param instantiations The (optional) list of instantiations.
 *
 * @return The integer value of the parameter, or null if does not have an integer value.
 *
 * @throws IllegalArgumentException If an instantiation provided is not an
 *  instantiation of the reactor class that is parameterized by the
 *  respective parameter or if the chain of instantiations is not nested.
 */
fun initialValueInt(parameter: Parameter, instantiations: List<Instantiation> = emptyList()): Int? =
    initialValue(parameter, instantiations)
        .sumBy { it.literal.toIntOrNullAnyRadix() ?: return null }


fun String.toIntOrNullAnyRadix(): Int? =
    try {
        Integer.decode(this)
    } catch (e: NumberFormatException) {
        null
    }
