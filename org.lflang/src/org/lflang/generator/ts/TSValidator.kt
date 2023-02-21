package org.lflang.generator.ts

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.lsp4j.DiagnosticSeverity
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.generator.CodeMap
import org.lflang.generator.DiagnosticReporting
import org.lflang.generator.HumanReadableReportingStrategy
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.Position
import org.lflang.generator.ValidationStrategy
import org.lflang.generator.Validator
import org.lflang.util.LFCommand
import java.nio.file.Path
import java.util.regex.Pattern

private val TSC_OUTPUT_LINE: Pattern = Pattern.compile(
    "(?<path>[^:]*):(?<line>\\d+):(?<column>\\d+) - (?<severity>\\w+).*: (?<message>.*)"
)
private val TSC_LABEL: Pattern = Pattern.compile("((?<=\\s))(~+)")

/**
 * A validator for generated TypeScript.
 *
 * @author Peter Donovan
 */
@Suppress("ArrayInDataClass")  // Data classes here must not be used in data structures such as hashmaps.
class TSValidator(
    private val fileConfig: FileConfig,
    errorReporter: ErrorReporter,
    codeMaps: Map<Path, CodeMap>
): Validator(errorReporter, codeMaps) {

    private class TSLinter(
        private val fileConfig: FileConfig,
        errorReporter: ErrorReporter,
        codeMaps: Map<Path, CodeMap>
    ): Validator(errorReporter, codeMaps) {
        companion object {
            private val mapper = ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        }

        // See https://eslint.org/docs/user-guide/formatters/#json for the best documentation available on
        //  the following data classes.
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

        override fun getPossibleStrategies(): Collection<ValidationStrategy> = listOf(object: ValidationStrategy {
            override fun getCommand(generatedFile: Path?): LFCommand? {
                return generatedFile?.let {
                    LFCommand.get(
                        "npx",
                        listOf("eslint", "--format", "json", fileConfig.srcGenPkgPath.relativize(it).toString()),
                        true,
                        fileConfig.srcGenPkgPath
                    )
                }
            }

            override fun getErrorReportingStrategy() = DiagnosticReporting.Strategy { _, _, _ -> }

            override fun getOutputReportingStrategy() = DiagnosticReporting.Strategy {
                validationOutput, errorReporter, map -> validationOutput.lines().filter { it.isNotBlank() }.forEach {
                    line: String -> mapper.readValue(line, Array<ESLintOutput>::class.java).forEach {
                        output: ESLintOutput -> output.messages.forEach {
                            message: ESLintMessage ->
                            val genPath: Path = fileConfig.srcGenPkgPath.resolve(output.filePath)
                            map[genPath]?.let {
                                codeMap ->
                                codeMap.lfSourcePaths().forEach {
                                    val lfStart = codeMap.adjusted(it, message.start)
                                    val lfEnd = codeMap.adjusted(it, message.end)
                                    if (!lfStart.equals(Position.ORIGIN)) {  // Ignore linting errors in non-user-supplied code.
                                        errorReporter.report(
                                            it,
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
        override fun getBuildReportingStrategies(): Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy>
            = Pair(DiagnosticReporting.Strategy { _, _, _ -> }, DiagnosticReporting.Strategy { _, _, _ -> })  // Not applicable
    }

    override fun getPossibleStrategies(): Collection<ValidationStrategy>
        = listOf(object: ValidationStrategy {
            override fun getCommand(generatedFile: Path?): LFCommand? {  // FIXME: Add "--incremental" argument if we update to TypeScript 4
                return LFCommand.get("npx", listOf("tsc", "--pretty", "--noEmit"), true, fileConfig.srcGenPkgPath)
            }

            override fun getErrorReportingStrategy() = DiagnosticReporting.Strategy { _, _, _ -> }

            override fun getOutputReportingStrategy() = HumanReadableReportingStrategy(
                TSC_OUTPUT_LINE, TSC_LABEL, fileConfig.srcGenPkgPath
            )

            override fun isFullBatch(): Boolean = true

            override fun getPriority(): Int = 0

        })

    override fun getBuildReportingStrategies(): Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy>
        = Pair(possibleStrategies.first().errorReportingStrategy, possibleStrategies.first().outputReportingStrategy)

    /**
     * Run a relatively fast linter on the generated code.
     * @param context The context of the current build.
     */
    fun doLint(context: LFGeneratorContext) {
        TSLinter(fileConfig, errorReporter, codeMaps).doValidate(context)
    }

    // If this is not true, then the user might as well be writing JavaScript.
    override fun validationEnabledByDefault(context: LFGeneratorContext?): Boolean = true
}
