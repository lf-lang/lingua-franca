package org.lflang.generator.lean

import org.lflang.ASTUtils
import org.lflang.InferredType
import org.lflang.TimeUnit
import org.lflang.TimeValue
import org.lflang.generator.TargetCode
import org.lflang.generator.TargetTypes
import org.lflang.inBlock
import org.lflang.lf.Code
import org.lflang.lf.Expression

object LeanTypes : TargetTypes {

    override fun supportsGenerics(): Boolean = true

    override fun getTargetTimeType(): String = "Time"

    override fun getTargetTagType(): String = "Tag"

    override fun getTargetUndefinedType(): String = TODO()

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String =
        "Array ($baseType)"

    override fun getTargetVariableSizeListType(baseType: String): String =
        "Array ($baseType)"

    override fun escapeIdentifier(ident: String): String =
        if (ident in LeanKeywords) "«$ident»"
        else ident

    override fun getTargetTimeExpr(timeValue: TimeValue): TargetCode = with(timeValue) {
        if (unit == null) "0" else {
            val unitName = when (unit) {
                TimeUnit.NANO   -> "ns)"
                TimeUnit.MICRO  -> "μs"
                TimeUnit.MILLI  -> "ms"
                TimeUnit.SECOND -> "s"
                TimeUnit.MINUTE -> "min"
                TimeUnit.HOUR   -> "hour"
                TimeUnit.DAY    -> "day"
                TimeUnit.WEEK   -> "week"
            }
            return "Time.of $magnitude .$unitName"
        }
    }

    override fun getFixedSizeListInitExpression(
        contents: List<String>,
        listSize: Int,
        withBraces: Boolean
    ): String =
        contents.joinToString(", ", "#[", "]")

    override fun getVariableSizeListInitExpression(contents: List<String>, withBraces: Boolean): String =
        contents.joinToString(", ", "#[", "]")
}

val LeanKeywords = setOf(
    // Curated from https://raw.githubusercontent.com/leanprover/lean4/master/doc/latex/lstlean.tex
    "protected", "private", "noncomputable", "renaming",
    "hiding", "variable", "example", "open", "export", "axiom",
    "inductive", "with", "structure", "universe", "match",
    "infix", "infixl", "infixr", "notation", "postfix", "prefix",
    "instance", "end", "this", "using", "namespace", "section",
    "attribute", "local", "set_option", "extends", "class", "calc",
    "have", "show", "suffices", "by", "in", "at", "let", "forall",
    "fun", "exists", "if", "then", "else", "from", "unless",
    "break", "continue", "mutual", "do", "def", "partial", "mut",
    "where", "macro", "syntax", "deriving", "return", "try",
    "catch", "for", "macro_rules", "declare_syntax_cat", "abbrev",
    "Sort", "Type", "Prop", "repeat"
)
