package org.lflang.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.*
import org.lflang.lf.*
import java.nio.file.Path
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

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
 * The following definitions are shortcuts to access static members of FileConfig and ASTUtils
 *
 * TODO these should likely be moved to a common place in the future
 */

val Resource.name: String get() = FileConfig.getName(this)

fun Path.toUnixString(): String = FileConfig.toUnixString(this)
fun Path.createDirectories() = FileConfig.createDirectories(this)

val Reactor.isGeneric get() = ASTUtils.isGeneric(this.toDefinition())

/* *******************************************************************************************
 *
 * The following definition provide extension that are likely useful across targets
 *
 * TODO Move these definitions to a common place and check if they are already implemented elsewhere
 */

/** Get the LF Model of a resource */
val Resource.model: Model get() = this.allContents.asSequence().filterIsInstance<Model>().first()

/** Get the "name" a reaction is represented with in target code.*/
val Reaction.name
    get(): String {
        val r = this.eContainer() as Reactor
        return "r" + r.reactions.lastIndexOf(this)
    }

/** Get a label representing the reaction.
 *
 * If the reactions is annotated with a label, then the label is returned. Otherwise, the reactions name is returned.
 */
val Reaction.label get(): String = ASTUtils.label(this) ?: this.name

/** Get the reactions priority */
val Reaction.priority
    get(): Int {
        val r = this.eContainer() as Reactor
        return r.reactions.lastIndexOf(this) + 1
    }

/** Return true if the given action is logical */
val Action.isLogical get() = this.origin == ActionOrigin.LOGICAL

/** Return true if the given action is physical */
val Action.isPhysical get() = this.origin == ActionOrigin.PHYSICAL

/**
 * Return true if the specified port is a multiport.
 * @receiver The port.
 * @return True if the port is a multiport.
 * TODO This is a duplicate of GeneratorBase.isMultiport
 */
val Port.isMultiport get() = this.widthSpec != null

/** Get the width of a port */
val Port.width: Int get() = this.widthSpec?.getWidth() ?: -1

/** Get the reactor that is instantiated in the receiving instantiation */
val Instantiation.reactor get() = this.reactorClass.toDefinition()

/** Check if the receiver is a bank instantiation */
val Instantiation.isBank: Boolean get() = this.widthSpec != null

/** Get the width of a bank instantiation */
val Instantiation.width: Int get() = this.widthSpec?.getWidth() ?: -1

/* ********************************************************************************************/

/** An object that defines the ,, (rangeTo) operator to prepend each line of the rhs multiline string with the lhs prefix.
 *
 * This is a neat little trick that allows for convenient insertion of multiline strings in string templates
 * while correctly managing the indentation. Consider this multiline string:
 * ```
 * val foo = """
 *    void foo() {
 *        // do something useful
 *    }""".trimIndent()
 * ```
 * Inserting this multiline string into another one like this:
 * ```
 * val bar = """
 *     class Bar {
 *         $foo
 *     };""".trimIndent()
 * ```
 * will unfortunately not produce  correctly indented result. Instead, the result will look like this:
 * * ```
 *     class Bar {
 *         void foo()
 *     // do something useful
 * }
 *     };""".trimIndent()
 * ```
 * This is because kotlin will insert the plain multiline string `foo` into the higher level string
 * without preserving the indentation of the first line. Using the .. operator as defined below,
 * we can properly indent `foo` while keeping a nice looking syntax that visualizes the expected
 * indentation in the resulting string.
 *
 *
 *  With the ,, operator, `foo` could be inserted into a higher level multiline string like this:
 *
 * ```
 * with (prependOperator) {
 *      val bar = """
 *          |class Bar {
 *      ${" |    "..foo}
 *          |};
 *      """.trimMargin()
 *      }
 * ```
 *
 * This will produce the expected output:
 * ```
 * class Bar {
 *     void foo() {
 *         // do something useful
 * };
 * ```
 *
 * Note that we define the .. operator inside of an object in order to restrict its visiblity. Since
 * the meaning of .. might not be obvious in other contexts, it needs to be explicilty enabled with
 * `with(prependOperator)`.
 *
 * TODO We likely want to move this to a central place
 */
object prependOperator {
    operator fun String.rangeTo(str: String) = str.replaceIndent(this)
}

/* **********************************************************************************************
 * C++ specific extensions shared across classes
 */
// TODO: Most of the extensions defined here should be moved to companion objects of their
//  corresponding generator classes. See for instance the CppParameterGenerator

/** Convert a log level to a severity number understood by the reactor-cpp runtime. */
val TargetProperty.LogLevel.severity
    get() = when (this) {
        TargetProperty.LogLevel.ERROR -> 1
        TargetProperty.LogLevel.WARN  -> 2
        TargetProperty.LogLevel.INFO  -> 3
        TargetProperty.LogLevel.LOG   -> 4
        TargetProperty.LogLevel.DEBUG -> 4
    }

/** Get a C++ representation of a LF unit. */
val TimeValue.cppUnit
    get() = when (this.unit) {
        TimeUnit.NSEC    -> "ns"
        TimeUnit.NSECS   -> "ns"
        TimeUnit.USEC    -> "us"
        TimeUnit.USECS   -> "us"
        TimeUnit.MSEC    -> "ms"
        TimeUnit.MSECS   -> "ms"
        TimeUnit.SEC     -> "s"
        TimeUnit.SECS    -> "s"
        TimeUnit.SECOND  -> "s"
        TimeUnit.SECONDS -> "s"
        TimeUnit.MIN     -> "min"
        TimeUnit.MINS    -> "min"
        TimeUnit.MINUTE  -> "min"
        TimeUnit.MINUTES -> "min"
        TimeUnit.HOUR    -> "h"
        TimeUnit.HOURS   -> "h"
        TimeUnit.DAY     -> "d"
        TimeUnit.DAYS    -> "d"
        TimeUnit.WEEK    -> "d*7"
        TimeUnit.WEEKS   -> "d*7"
        TimeUnit.NONE    -> ""
        null             -> ""
    }

/** Convert a LF time value to a representation in C++ code */
fun TimeValue.toCode() = if (this.time == 0L) "reactor::Duration::zero()" else "${this.time}${this.cppUnit}"

/** Convert a Time to a representation in C++ code
 *
 * FIXME this is redundant to GeneratorBase.getTargetTime
 */
fun Time.toCode() = TimeValue(this.interval.toLong(), this.unit).toCode()

/** Convert a value to a time representation in C++ code*
 *
 * If the value evaluates to 0, it is interpreted as a time.
 * FIXME this is redundant to GeneratorBase.getTargetTime
 */
fun Value.toTime(): String = when {
    this.time != null -> this.time.toCode()
    this.isZero       -> TimeValue(0, TimeUnit.NONE).toCode()
    else              -> this.toText()
}

/**
 * Get textual representation of a value in C++ code
 *
 * If the value evaluates to 0, it is interpreted as a normal value.
 * FIXME this is redundant to GeneratorBase.getTargetValue
 */
fun Value.toCode(): String = this.time?.toCode() ?: this.toText()

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
    get() = when {
        this is VarRef  -> this.name
        this.isShutdown -> LfPackage.Literals.TRIGGER_REF__SHUTDOWN.name
        this.isStartup  -> LfPackage.Literals.TRIGGER_REF__STARTUP.name
        else            -> throw AssertionError()
    }

/** Return a comment to be inserted at the top of generated files. */
fun fileComment(r: Resource) = """
    /*
     * This file was autogenerated by the Lingua Franca Compiler
     *
     * Source: ${r.uri}
     * Date: ${LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"))}
     */
    """.trimIndent()

/* *************************************************************************************************
 * FIXME The following extensions for handling types are actually defined in `GeneratorBase` and should be overridden by the
 *  derived CppGenerator Class. However, this causes a weird design because we would need to pass around a reference
 *  to CppGenerator to derive target types from AST nodes... Moreover, it would not be possible to use the convenient extension
 *  mechanism. The code below is a workaround, but a more general solution should be found.
 */

val StateVar.targetType get():String = this.inferredType.targetType
val Port.targetType get():String = this.inferredType.targetType
val Action.targetType: String
    get() {
        val inferred = this.inferredType
        return if (inferred.isUndefined) "void" else inferred.targetType
    }

private fun fixedSizeListType(baseType: String, size: Int) = "std::array<$baseType, $size>"
private fun variableSizeListType(baseType: String) = "std::vector<$baseType>"

val InferredType.targetType: String
    get() = when {
        this.isUndefined        -> "/* undefined type */"
        this.isTime             -> when {
            this.isFixedSizeList    -> fixedSizeListType("reactor::Duration", this.listSize)
            this.isVariableSizeList -> variableSizeListType("reactor::Duration")
            else                    -> "reactor::Duration"
        }
        this.isFixedSizeList    -> fixedSizeListType(this.baseType(), this.listSize)
        this.isVariableSizeList -> variableSizeListType(this.baseType())
        else                    -> this.toText()
    }
