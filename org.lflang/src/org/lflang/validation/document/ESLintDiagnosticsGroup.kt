package org.lflang.validation.document

import kotlinx.serialization.*

@Serializable
data class ESLintDiagnosticsGroup(
    val filePath: String,
    val messages: List<ESLintDiagnostic>,
    val errorCount: Int,
    val fatalErrorCount: Int,
    val warningCount: Int,
    val fixableErrorCount: Int,
    val fixableWarningCount: Int,
    val source: String
) {
    companion object {
        val serializer: KSerializer<ESLintDiagnosticsGroup> = kotlinx.serialization.serializer()
    }
}

@Serializable
data class ESLintDiagnostic(
    val ruleId: String,
    val severity: Int,
    val message: String,
    val line: Int,
    val column: Int,
    val nodeType: String,
    val messageId: String,
    val endLine: Int,
    val endColumn: Int,
    val fix: ESLintFix? = null
)

@Serializable
data class ESLintFix(val range: List<Int>, val text: String)

