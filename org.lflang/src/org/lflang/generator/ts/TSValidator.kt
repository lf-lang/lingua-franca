package org.lflang.generator.ts

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.lsp4j.DiagnosticSeverity
import org.lflang.ErrorReporter
import org.lflang.generator.CodeMap
import org.lflang.generator.DiagnosticReporting
import org.lflang.generator.Position
import org.lflang.generator.ValidationStrategy
import org.lflang.generator.Validator
import org.lflang.util.LFCommand
import java.nio.file.Path

@Suppress("ArrayInDataClass")  // Data classes here must not be used in data structures such as hashmaps.
class TSValidator(
    private val fileConfig: TSFileConfig,
    errorReporter: ErrorReporter,
    codeMaps: Map<Path, CodeMap>
): Validator(errorReporter, codeMaps) {

    companion object {
        private val mapper = ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
    }

    private data class ESLintOutput(
        @JsonProperty("filePath") val filePath: String,
        @JsonProperty("messages") val messages: Array<ESLintMessage>,
        @JsonProperty("errorCount") val errorCount: Int,
        @JsonProperty("fatalErrorCount") val fatalErrorCount: Int,
        @JsonProperty("warningCount") val warningCount: Int,
        @JsonProperty("fixableErrorCount") val fixableErrorCount: Int,
        @JsonProperty("fixableWarningCount") val fixableWarningCount: Int,
        @JsonProperty("source") val source: String
    )

    private data class ESLintMessage(
        @JsonProperty("ruleId") val ruleId: String?,
        @JsonProperty("severity") val _severity: Int,
        @JsonProperty("message") val message: String,
        @JsonProperty("line") val line: Int,
        @JsonProperty("column") val column: Int,
        @JsonProperty("nodeType") val nodeType: String?,
        @JsonProperty("messageId") val messageId: String?,
        @JsonProperty("endLine") val endLine: Int,
        @JsonProperty("endColumn") val endColumn: Int,
        @JsonProperty("fix") val fix: ESLintFix?
    ) {
        val start: Position = Position.fromOneBased(line, column)
        val end: Position = if (endLine >= line) Position.fromOneBased(endLine, endColumn) else start.plus(" ")
        val severity: DiagnosticSeverity = when (_severity) {
            0 -> DiagnosticSeverity.Information
            1 -> DiagnosticSeverity.Warning
            2 -> DiagnosticSeverity.Error
            else -> DiagnosticSeverity.Warning // This should never happen
        }
    }

    private data class ESLintFix(
        @JsonProperty("range") val range: Array<Int>,
        @JsonProperty("text") val text: String
    )

    override val possibleStrategies: Iterable<ValidationStrategy> = listOf(object: ValidationStrategy {
        override fun getCommand(generatedFile: Path?): LFCommand? {
            return generatedFile?.let {
                LFCommand.get(
                    "npx",
                    listOf("eslint", "--format", "json", fileConfig.srcGenPath.relativize(it).toString()),
                    fileConfig.srcGenPath
                )
            }
        }

        override fun getErrorReportingStrategy() = DiagnosticReporting.Strategy { _, _, _ -> }

        override fun getOutputReportingStrategy() = DiagnosticReporting.Strategy {
            validationOutput, errorReporter, map -> validationOutput.lines().filter { it.isNotBlank() }.forEach {
                line: String -> mapper.readValue(line, Array<ESLintOutput>::class.java).forEach {
                    output: ESLintOutput -> output.messages.forEach {
                        message: ESLintMessage ->
                        val genPath: Path = fileConfig.srcGenPath.resolve(output.filePath)
                        map[genPath]?.let {
                            codeMap ->
                            codeMap.lfSourcePaths().forEach {
                                val lfStart = codeMap.adjusted(it, message.start)
                                val lfEnd = codeMap.adjusted(it, message.end)
                                if (!lfStart.equals(Position.ORIGIN)) {
                                    errorReporter.report(
                                        message.severity,
                                        DiagnosticReporting.messageOf(message.message, genPath, message.start),
                                        lfStart,
                                        if (lfEnd > lfStart) lfEnd else lfStart.plus(" "),
                                    )
                                }
                            }
                        }
                    }
                }
            }
        }

        override fun isFullBatch(): Boolean = false // ESLint permits glob patterns. We could make this full-batch if desired.

        override fun getPriority(): Int = 0

    })
    override val buildReportingStrategies: Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy>
        get() = Pair(DiagnosticReporting.Strategy {_, _, _ -> }, DiagnosticReporting.Strategy {_, _, _ -> })  // TODO

}
