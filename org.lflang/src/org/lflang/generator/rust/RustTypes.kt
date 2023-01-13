/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

package org.lflang.generator.rust

import org.lflang.ASTUtils.toText
import org.lflang.InferredType
import org.lflang.TimeValue
import org.lflang.generator.TargetCode
import org.lflang.generator.TargetTypes
import org.lflang.inBlock
import org.lflang.lf.Code
import org.lflang.lf.Expression

object RustTypes : TargetTypes {

    override fun supportsGenerics(): Boolean = true

    override fun getTargetTimeType(): String = "Duration"

    override fun getTargetTagType(): String = "EventTag"

    override fun getTargetUndefinedType(): String = "()"

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String =
        "[ $baseType ; $size ]"

    override fun getTargetVariableSizeListType(baseType: String): String =
        "Vec<$baseType>"

    override fun escapeIdentifier(ident: String): String =
        if (ident in RustKeywords) "r#$ident"
        else ident

    override fun getTargetExpr(expr: Expression, type: InferredType?): String = when (expr) {
        is Code  -> toText(expr).inBlock()
        else -> super.getTargetExpr(expr, type)
    }

    override fun getTargetTimeExpr(timeValue: TimeValue): TargetCode = with(timeValue) {
        val unit = unit?.canonicalName.orEmpty()
        "delay!($magnitude $unit)"
    }

    override fun getFixedSizeListInitExpression(
        contents: List<String>,
        listSize: Int,
        withBraces: Boolean
    ): String =
        contents.joinToString(", ", "[", "]")

    override fun getVariableSizeListInitExpression(contents: List<String>, withBraces: Boolean): String =
        contents.joinToString(", ", "vec![", "]")

    override fun getMissingExpr(type: InferredType): String =
        "Default::default()"
}

val RustKeywords = setOf(
    // https://doc.rust-lang.org/reference/keywords.html
    "as", "break", "const", "continue", "crate", "else",
    "enum", "extern", /*"false",*/ "fn", "for", "if", "impl",
    "in", "let", "loop", "match", "mod", "move", "mut",
    "pub", "ref", "return", /*"self",*/ "Self", "static",
    "struct", "super", "trait", /*"true",*/ "type", "unsafe",
    "use", "where", "while",
    // reserved kws
    "abstract", "async", "await", "dyn", "become", "box",
    "do", "final", "macro", "override", "priv", "typeof",
    "unsized", "virtual", "yield", "try",
    // "weak" keywords, disallow them anyway
    "union", "dyn"
)
