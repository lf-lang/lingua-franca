package org.lflang.generator.uc

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.*
import org.lflang.generator.cpp.CppInstanceGenerator.Companion.isEnclave
import org.lflang.generator.cpp.name
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
import org.lflang.target.property.type.LoggingType.LogLevel

/* *******************************************************************************************
 *
 * The following definition provide extension that are likely useful across targets
 *
 * TODO Move these definitions to a common place and check if they are already implemented elsewhere
 */

/** Get the "name" a reaction is represented with in target code.*/
val Reaction.codeName
    get(): String = name ?: "reaction_$priority"

/* **********************************************************************************************
 * C++ specific extensions shared across classes
 */
// TODO: Most of the extensions defined here should be moved to companion objects of their
//  corresponding generator classes. See for instance the CppParameterGenerator

///** Convert a LF time value to a representation in C++ code */
fun TimeValue.toCCode() = UcTypes.getTargetTimeExpr(this)
//
///**
// * Convert a LF time value to a representation in C++ code.
// * @param inferredType Type that the expr has (or null), may guide code generation if ambiguous
// */
fun Expression.toCCode(inferredType: InferredType? = null): String =
    UcTypes.getTargetExpr(this, inferredType)
//
//
///**
// * Convert a value to a time representation in C++ code
// *
// * If the value evaluates to 0, it is interpreted as a time.
// *
// */
fun Expression?.toCTime(): String =
    this?.toCCode(inferredType = InferredType.time()) ?: "reactor::Duration::zero()"
//

val TriggerRef.name: String
    get() = when (this) {
        is VarRef            -> this.name
        is BuiltinTriggerRef -> type.literal
        else                 -> unreachable()
    }
