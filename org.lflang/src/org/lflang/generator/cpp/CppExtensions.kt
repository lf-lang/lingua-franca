package org.lflang.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.*
import org.lflang.lf.BuiltinTriggerRef
import org.lflang.lf.Expression
import org.lflang.lf.Port
import org.lflang.lf.Preamble
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef
import org.lflang.lf.Visibility
import org.lflang.lf.WidthSpec

/*************
 * Copyright (c) 2019-2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

/* *******************************************************************************************
 *
 * The following definition provide extension that are likely useful across targets
 *
 * TODO Move these definitions to a common place and check if they are already implemented elsewhere
 */

/** Get the "name" a reaction is represented with in target code.*/
val Reaction.name
    get(): String = "r$indexInContainer"

/* **********************************************************************************************
 * C++ specific extensions shared across classes
 */
// TODO: Most of the extensions defined here should be moved to companion objects of their
//  corresponding generator classes. See for instance the CppParameterGenerator

/** Convert a LF time value to a representation in C++ code */
fun TimeValue.toCppCode() = CppTypes.getTargetTimeExpr(this)

/**
 * Convert a LF time value to a representation in C++ code.
 * @param inferredType Type that the expr has (or null), may guide code generation if ambiguous
 */
fun Expression.toCppCode(inferredType: InferredType? = null): String =
    CppTypes.getTargetExpr(this, inferredType)


/**
 * Convert a value to a time representation in C++ code*
 *
 * If the value evaluates to 0, it is interpreted as a time.
 *
 * @param outerContext A flag indicating whether to generate code for the scope of the outer reactor class.
 *                    This should be set to false if called from code generators for the inner class.
 */
fun Expression?.toCppTime(): String =
    this?.toCppCode(inferredType = InferredType.time()) ?: "reactor::Duration::zero()"

/** Get the textual representation of a width in C++ code */
fun WidthSpec.toCppCode(): String = terms.joinToString(" + ") {
    when {
        it.parameter != null -> "__lf_parameters." + it.parameter.name
        it.port != null      -> with(it.port) {
            if (container?.isBank == true) {
                if ((variable as Port).isMultiport) "(${container.name}.size() * ${container.name}[0]->${variable.name}.size())"
                else "${container.name}.size()"
            } else {
                if ((variable as Port).isMultiport) "$name.size()"
                else "1"
            }
        }
        it.code != null      -> it.code.toText()
        else                 -> it.width.toString()
    }
}

/** True if the preamble is public */
val Preamble.isPublic: Boolean get() = this.visibility == Visibility.PUBLIC

/** True if the preamble is private */
val Preamble.isPrivate: Boolean get() = this.visibility == Visibility.PRIVATE

/** The template line preceding the class declaration and any member definitions for a `reactor */
val Reactor.templateLine
    get() =
        if (isGeneric) """template<${typeParms.joinToString(", ") { "class ${it.toText()}" }}>"""
        else ""

/** Get templated name of a reactor class */
val Reactor.templateName: String get() = if (isGeneric) "$name<${typeParms.joinToString(", ") { it.toText() }}>" else name

/** Get a C++ code representation of the given variable */
val VarRef.name: String
    get() = if (this.container == null) this.variable.name
    else "${this.container.name}->${this.variable.name}"

/** Get a C++ code representation of the given trigger */
val TriggerRef.name: String
    get() = when (this) {
        is VarRef            -> this.name
        is BuiltinTriggerRef -> type.literal
        else                 -> unreachable()
    }

/** Return a comment to be inserted at the top of generated files. */
fun fileComment(r: Resource) = """
    /*
     * This file was autogenerated by the Lingua Franca Compiler.
     *
     * Source: ${r.uri}
     */
    """.trimIndent()

val InferredType.cppType: String
    get() = CppTypes.getTargetType(this)


/** Convert a log level to a severity number understood by the reactor-cpp runtime. */
val TargetProperty.LogLevel.severity
    get() = when (this) {
        TargetProperty.LogLevel.ERROR -> 1
        TargetProperty.LogLevel.WARN  -> 2
        TargetProperty.LogLevel.INFO  -> 3
        TargetProperty.LogLevel.LOG   -> 4
        TargetProperty.LogLevel.DEBUG -> 4
    }

fun Reactor.hasBankIndexParameter() = parameters.firstOrNull { it.name == "bank_index" } != null
