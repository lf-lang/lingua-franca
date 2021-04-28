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

import org.lflang.lf.*

/**
 * If this reactor declaration is an import, then
 * return the imported reactor class definition.
 * Otherwise, just return the argument.
 */
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

val Parameter.isOfTimeType: Boolean get() = ASTUtils.isOfTimeType(this)

/**
 * Translate this code element into its textual representation.
 * @see ASTUtils.toText
 */
fun Code.toText(): String = ASTUtils.toText(this)

/**
 * Translate this code element into its textual representation.
 * @see ASTUtils.toText
 */
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
fun Element.toText(): String =
    literal?.withoutQuotes()?.trim() ?: id ?: ""


fun Delay.toText(): String {
    if (parameter !== null) {
        return parameter.name
    }
    return "$interval $unit"
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
fun Type.toText(): String = baseType + arraySpec?.toText().orEmpty()

/**
 * Produce a unique identifier within a reactor based on a
 * given based name. If the name does not exists, it is returned;
 * if does exist, an index is appended that makes the name unique.
 * @receiver The reactor to find a unique identifier within.
 * @param name The name to base the returned identifier on.
 */
fun Reactor.getUniqueIdentifier(name: String): String =
    ASTUtils.getUniqueIdentifier(this, name)

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
 */
val WidthSpec.width: Int?
    get() = ASTUtils.width(this, null).takeIf { it >= 0 }


// more general extensions

fun String.toIntOrNullAnyRadix(): Int? =
    try {
        Integer.decode(this)
    } catch (e: NumberFormatException) {
        null
    }

fun <T> List<T>.tail() = subList(1, size)
fun <T> List<T>.headAndTail() = Pair(first(), tail())
