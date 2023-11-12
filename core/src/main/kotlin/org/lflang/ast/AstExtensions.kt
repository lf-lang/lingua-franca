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

import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ast.ASTUtils
import org.lflang.lf.*

/**
 * If this reactor declaration is an import, then
 * return the imported reactor class definition.
 * Otherwise, just return the argument.
 */
fun ReactorDecl.toDefinition(): Reactor = when (this) {
    is Reactor         -> this
    is ImportedReactor -> this.reactorClass
    else               -> throw AssertionError("Unknown reactor type: $this")
}

/**
 * Given a reactor class, return a list of all its actions,
 * which includes actions of base classes that it extends.
 */
val Reactor.allActions: List<Action> get() = collectInSupertypes { actions }

/**
 * Given a reactor class, return a list of all its connections,
 * which includes connections of base classes that it extends.
 */
val Reactor.allConnections: List<Connection> get() = collectInSupertypes { connections }

/**
 * Given a reactor class, return a list of all its inputs,
 * which includes inputs of base classes that it extends.
 */
val Reactor.allInputs: List<Input> get() = collectInSupertypes { inputs }

/**
 * Given a reactor class, return a list of all its outputs,
 * which includes outputs of base classes that it extends.
 */
val Reactor.allOutputs: List<Output> get() = collectInSupertypes { outputs }

/**
 * Given a reactor class, return a list of all its instantiations,
 * which includes instantiations of base classes that it extends.
 */
val Reactor.allInstantiations: List<Instantiation> get() = collectInSupertypes { instantiations }

/**
 * Given a reactor class, return a list of all its parameters,
 * which includes parameters of base classes that it extends.
 */
val Reactor.allParameters: List<Parameter> get() = collectInSupertypes { parameters }

/**
 * Given a reactor class, return a list of all its reactions,
 * which includes reactions of base classes that it extends.
 */
val Reactor.allReactions: List<Reaction> get() = collectInSupertypes { reactions }

/**
 * Given a reactor class, return a list of all its state variables,
 * which includes state variables of base classes that it extends.
 */
val Reactor.allStateVars: List<StateVar> get() = collectInSupertypes { stateVars }

/**
 * Given a reactor class, return a list of all its  timers,
 * which includes timers of base classes that it extends.
 */
val Reactor.allTimers: List<Timer> get() = collectInSupertypes { timers }

/**
 * Apply the [collector] method recursively to the receiving reactor and all its superclasses.
 *
 * This collects the return values for indivudal reactors in a flat list, creating a collected list
 * over all visisted reactors.
 */
private fun <T> Reactor.collectInSupertypes(collector: Reactor.() -> List<T>): List<T> =
    superClasses.orEmpty().mapNotNull { it.toDefinition().collectInSupertypes(collector) }.flatten() + this.collector()

/**
 * Return all components declared within this reactor in an
 * unspecified order.
 */
fun Reactor.allComponents(): List<Variable> =
    collectInSupertypes {
        outputs + inputs + actions + timers
    }

/**
 * Check if the reactor class uses generics
 * @receiver the reactor to check
 * @true true if the reactor uses generics
 */
val Reactor.isGeneric get() = ASTUtils.isGeneric(toDefinition())

/**
 * Report whether the given parameter has been declared a type or has been
 * inferred to be a type. Note that if the parameter was declared to be a
 * time, its initialization may still be faulty (assigning a value that is
 * not actually a valid time).
 * @see ASTUtils.isOfTimeType
 * @return True if the receiver denotes a time, false otherwise.
 */
val Parameter.isOfTimeType: Boolean get() = ASTUtils.isOfTimeType(this)

/**
 * Report whether the given state variable denotes a time or not.
 * @see ASTUtils.isOfTimeType
 * @return True if the receiver denotes a time, false otherwise.
 */
val StateVar.isOfTimeType: Boolean get() = ASTUtils.isOfTimeType(this)

/**
 * Translate this code element into its textual representation
 * with {@code CodeMap.Correspondence} tags inserted.
 * @see ASTUtils.toText
 */
fun EObject.toText(): String = ASTUtils.toText(this)

fun Time.toTimeValue(): TimeValue = ASTUtils.toTimeValue(this)

fun TimeValue.toTimeNode(): Time =
    LfFactory.eINSTANCE.createTime().also {
        it.unit = this.unit.canonicalName
        it.interval = this.magnitude.toInt()
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
 * @receiver AST node to inspect.
 * @return True if the given value denotes the constant `0`, false otherwise.
 */
val Expression.isZero: Boolean get() = ASTUtils.isZero(this)

/**
 * Given a parameter, return an inferred type. Only two types can be
 * inferred: "time" and "timeList". Return the "undefined" type if
 * neither can be inferred.
 *
 * @see ASTUtils.getInferredType
 * @return The inferred type, or "undefined" if none could be inferred.
 */
val Parameter.inferredType: InferredType get() = ASTUtils.getInferredType(this)

/**
 * Given a state variable, return an inferred type. Only two types can be
 * inferred: "time" and "timeList". Return the "undefined" type if
 * neither can be inferred.
 *
 * @see ASTUtils.getInferredType
 * @return The inferred type, or "undefined" if none could be inferred.
 */
val StateVar.inferredType: InferredType get() = ASTUtils.getInferredType(this)

/**
 * Construct an inferred type from an "action" AST node based
 * on its declared type. If no type is declared, return the "undefined"
 * type.
 *
 * @see ASTUtils.getInferredType
 * @return The inferred type, or "undefined" if none was declared.
 */
val Action.inferredType: InferredType get() = ASTUtils.getInferredType(this)

/**
 * Construct an inferred type from a "port" AST node based on its declared
 * type. If no type is declared, return the "undefined" type.
 *
 * @see ASTUtils.getInferredType
 * @return The inferred type, or "undefined" if none was declared.
 */
val Port.inferredType: InferredType get() = ASTUtils.getInferredType(this)

/**
 * Report whether a state variable has been initialized or not.
 * @receiver The state variable to be checked.
 * @return True if the variable was initialized, false otherwise.
 */
val StateVar.isInitialized: Boolean get() = init != null

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
 * The [instantiations] list is as in
 * [ASTUtils.initialValue]
 * If the spec belongs to an instantiation (for a bank of reactors),
 * then the first element on this list should be the instantiation
 * that contains this instantiation. If the spec belongs to a port,
 * then the first element on the list should be the instantiation
 * of the reactor that contains the port.
 *
 * @param instantiations The (optional) list of instantiations.
 *
 * @receiver The width, or -1 if the width could not be determined.
 *
 * @throws IllegalArgumentException If an instantiation provided is not as
 *  given above or if the chain of instantiations is not nested.
 */
fun WidthSpec.getWidth(instantiations: List<Instantiation>? = null) = ASTUtils.width(this, instantiations)

/** Get the LF Model of a resource */
val Resource.model: Model get() = this.allContents.asSequence().filterIsInstance<Model>().first()

/** Get a label representing the receiving reaction.
 *
 * If the reaction is named, then the name is returned.
 * If it is not named but annotated with a label, then the label is returned.
 * Otherwise, a reaction name is generated based on its priority.
 */
val Reaction.label get(): String = name ?: AttributeUtils.getLabel(this) ?: "reaction_$priority"

/** Get the priority of a receiving reaction */
val Reaction.priority
    get(): Int {
        val r = this.eContainer() as Reactor
        return r.reactions.lastIndexOf(this) + 1
    }

/** Return true if the receiving action is logical */
val Action.isLogical get() = this.origin == ActionOrigin.LOGICAL

/** Return true if the receiving action is physical */
val Action.isPhysical get() = this.origin == ActionOrigin.PHYSICAL

/**
 * Return true if the receiving is a multiport.
 */
val Port.isMultiport get() = ASTUtils.isMultiport(this)

/**
 * Return true if the receiving Variable is a port and a multiport.
 */
val Variable.isMultiport get() = (this is Port) && this.isMultiport

/** Get the reactor that is instantiated in the receiving instantiation. */
val Instantiation.reactor get() = this.reactorClass.toDefinition()

/** Check if the receiver is a bank instantiation. */
val Instantiation.isBank: Boolean get() = this.widthSpec != null


/** The index of a reaction in its containing reactor. */
val Reaction.indexInContainer
    get(): Int = containingReactor.reactions.lastIndexOf(this)

/** The reactor containing a given reaction. */
val Reaction.containingReactor get() = this.eContainer() as Reactor

/** Returns true if this is an input port (not an output port). */
val Port.isInput get() = this is Input


/**
 * Produce the text of the given node in the source LF file.
 * May be null if the node model is unavailable (please blame eclipse).
 */
fun EObject.toTextTokenBased(): String? {
    val node = NodeModelUtils.getNode(this) ?: return null
    val builder = StringBuilder(node.totalLength.coerceAtLeast(1))
    var hidden = true // remove hidden tokens until the first non-hidden token
    for (leaf in node.leafNodes) {
        hidden = hidden && leaf.isHidden
        if (!hidden) {
            builder.append(leaf.text)
        }
    }
    return builder.trim().toString()
}

val Connection.hasMultipleConnections: Boolean get() = ASTUtils.hasMultipleConnections(this)
