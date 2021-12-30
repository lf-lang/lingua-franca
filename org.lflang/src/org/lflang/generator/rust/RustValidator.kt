package org.lflang.generator.rust

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.generator.*
import org.lflang.util.LFCommand
import java.nio.file.Path

@Suppress("ArrayInDataClass")  // Data classes here must not be used in data structures such as hashmaps.
class RustValidator(
    private val fileConfig: FileConfig,
    errorReporter: ErrorReporter,
    codeMaps: Map<Path, CodeMap>
): Validator(errorReporter, codeMaps) {
    companion object {
        private val mapper = ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        private const val COMPILER_MESSAGE_REASON = "compiler-message"
    }
    // See the following references for details on these data classes:
    //  * https://doc.rust-lang.org/cargo/reference/external-tools.html#json-messages
    //  * https://doc.rust-lang.org/rustc/json.html
    private data class RustOutput(
        @JsonProperty("reason") val reason: String,
    )
    private data class RustCompilerMessage(
        @JsonProperty("reason") val reason: String,
        @JsonProperty("package_id") val packageId: String,
        @JsonProperty("manifest_path") val manifestPath: String,
        @JsonProperty("target") val target: RustCompilerTarget,
        @JsonProperty("message") val message: RustDiagnostic
    )
    private data class RustCompilerTarget(
        @Suppress("ArrayInDataClass") @JsonProperty("kind") val kind: Array<String>,
        @Suppress("ArrayInDataClass") @JsonProperty("crate_types") val crateTypes: Array<String>,
        @JsonProperty("name") val name: String,
        @JsonProperty("src_path") val srcPath: String,
        @JsonProperty("edition") val edition: String,
        @Suppress("ArrayInDataClass") @JsonProperty("required_features") val requiredFeatures: Array<String>?,
        @JsonProperty("doctest") val doctest: Boolean
    )
    private data class RustDiagnostic(
        @JsonProperty("message") val message: String,
        @JsonProperty("code") val code: RustDiagnosticCode?,
        @JsonProperty("level") val level: String,
        @Suppress("ArrayInDataClass") @JsonProperty("spans") val spans: Array<RustSpan>,  // Ignore warning. This will not be used in hashmaps etc.
        @Suppress("ArrayInDataClass") @JsonProperty("children") val children: Array<RustDiagnostic?>,
        @JsonProperty("rendered") val rendered: String?
    )
    private data class RustDiagnosticCode(
        @JsonProperty("code") val code: String,
        @JsonProperty("explanation") val explanation: String?
    )
    private data class RustSpan(
        @JsonProperty("file_name") val fileName: String,
        @JsonProperty("byte_start") val byteStart: Int,
        @JsonProperty("byte_end") val byteEnd: Int,
        @JsonProperty("line_start") val lineStart: Int,
        @JsonProperty("line_end") val lineEnd: Int,
        @JsonProperty("column_start") val columnStart: Int,
        @JsonProperty("column_end") val columnEnd: Int,
        @JsonProperty("is_primary") val isPrimary: Boolean,
        @JsonProperty("text") val text: Array<RustSpanText>,  // Ignore warning. This will not be used in hashmaps etc.
        @JsonProperty("label") val label: String?,
        @JsonProperty("suggested_replacement") val suggestedReplacement: String?,
        @JsonProperty("suggestion_applicability") val suggestionApplicability: String?,
        @JsonProperty("expansion") val expansion: RustSpanExpansion?
    )
    private data class RustSpanExpansion(
        @JsonProperty("span") val span: String,
        @JsonProperty("macro_decl_name") val macroDeclName: String,
        @JsonProperty("def_site_span") val defSiteSpan: RustSpan?
    )
    private data class RustSpanText(
        @JsonProperty("text") val text: String,
        @JsonProperty("highlight_start") val highlightStart: Int,
        @JsonProperty("highlight_end") val highlightEnd: Int
    )

    override val possibleStrategies: Iterable<ValidationStrategy> = listOf(object: ValidationStrategy {
        override fun getCommand(generatedFile: Path?): LFCommand {
            return LFCommand.get("cargo", listOf("clippy", "--message-format", "json"), fileConfig.srcGenPath)
        }

        override fun getErrorReportingStrategy() = CommandErrorReportingStrategy { _, _, _ -> }

        override fun getOutputReportingStrategy() = CommandErrorReportingStrategy {
            validationOutput, errorReporter, map -> validationOutput.lines().forEach { messageLine ->
                if (messageLine.isNotBlank() && mapper.readValue(messageLine, RustOutput::class.java).reason == COMPILER_MESSAGE_REASON) {
                    val message = mapper.readValue(messageLine, RustCompilerMessage::class.java).message
                    System.err.println("DEBUG: $message")
                    if (message.spans.isEmpty()) errorReporter.reportError(message.message)  // FIXME: This might not actually be an error!
                    for (s: RustSpan in message.spans) {
                        val p: Path = fileConfig.srcGenPath.parent.resolve(s.fileName)
                        map[p]?.let {
                            for (lfSourcePath: Path in it.lfSourcePaths()) getReport(message.level).invoke(
                                p,
                                it.adjusted(lfSourcePath, Position.fromOneBased(s.lineStart, s.columnStart)).oneBasedLine,
                                message.message
                            )
                        }
                    }
                }
            }
        }

        private fun getReport(level: String): (Path, Int, String) -> Unit = when (level) {
            "error" -> errorReporter::reportError
            "warning" -> errorReporter::reportWarning
            "note" -> errorReporter::reportWarning  // FIXME: errorReporter needs a bigger interface!
            "help" -> errorReporter::reportWarning
            "failure-note" -> errorReporter::reportWarning
            "error: internal compiler error" -> errorReporter::reportError
            else -> errorReporter::reportWarning
        }

        override fun getPriority(): Int = 0

        override fun isFullBatch(): Boolean = true
    })

    override val buildReportingStrategies: Pair<CommandErrorReportingStrategy, CommandErrorReportingStrategy> = Pair(
        possibleStrategies.first().errorReportingStrategy, possibleStrategies.first().outputReportingStrategy
    )
}
