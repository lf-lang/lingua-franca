package org.lflang.generator.rust

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.lsp4j.DiagnosticSeverity
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.generator.CodeMap
import org.lflang.generator.DiagnosticReporting
import org.lflang.generator.Position
import org.lflang.generator.ValidationStrategy
import org.lflang.generator.Validator
import org.lflang.util.LFCommand
import java.nio.file.Path
import java.nio.file.Paths

/**
 * A validator for generated Rust.
 *
 * @author Peter Donovan
 */
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
    // See the following reference for details on cargo metadata: https://doc.rust-lang.org/cargo/commands/cargo-metadata.html
    public data class RustMetadata(
        // Other fields exist, but we don't need them. The mapper is configured not to fail on unknown properties.
        @JsonProperty("workspace_root") private val _workspaceRoot: String,
        @JsonProperty("target_directory") private val _targetDirectory: String
    ) {
        val workspaceRoot: Path
            get() = Paths.get(_workspaceRoot)
        val targetDirectory: Path
            get() = Paths.get(_targetDirectory)
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
    ) {
        val severity: DiagnosticSeverity
            get() = when (level) {
                "error" -> DiagnosticSeverity.Error
                "warning" -> DiagnosticSeverity.Warning
                "note" -> DiagnosticSeverity.Information
                "help" -> DiagnosticSeverity.Hint
                "failure-note" -> DiagnosticSeverity.Information
                "error: internal compiler error" -> DiagnosticSeverity.Error
                else -> DiagnosticSeverity.Warning // Should be impossible
            }
    }
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
    ) {
        val start: Position
            get() = Position.fromOneBased(lineStart, columnStart)
        val end: Position
            get() = Position.fromOneBased(lineEnd, columnEnd)
    }
    private data class RustSpanExpansion(
        @JsonProperty("span") val span: RustSpan,
        @JsonProperty("macro_decl_name") val macroDeclName: String,
        @JsonProperty("def_site_span") val defSiteSpan: RustSpan?
    )
    private data class RustSpanText(
        @JsonProperty("text") val text: String,
        @JsonProperty("highlight_start") val highlightStart: Int,
        @JsonProperty("highlight_end") val highlightEnd: Int
    )

    private var _metadata: RustMetadata? = null

    public fun getMetadata(): RustMetadata? {
        val nullableCommand = LFCommand.get("cargo", listOf("metadata", "--format-version", "1"), true, fileConfig.srcGenPkgPath)
        _metadata = _metadata ?: nullableCommand?.let { command ->
            command.run { false }
            command.output.toString().lines().filter { it.startsWith("{") }.mapNotNull {
                try {
                    mapper.readValue(it, RustMetadata::class.java)
                } catch (e: JsonProcessingException) {
                    null
                }
            }.firstOrNull()
        }
        return _metadata
    }

    override fun getPossibleStrategies(): Collection<ValidationStrategy> = listOf(object: ValidationStrategy {
        override fun getCommand(generatedFile: Path?): LFCommand {
            return LFCommand.get(
                "cargo",
                listOf("clippy", "--message-format", "json-diagnostic-rendered-ansi"),
                true,
                fileConfig.srcGenPkgPath
            )
        }

        override fun getErrorReportingStrategy() = DiagnosticReporting.Strategy { _, _, _ -> }

        override fun getOutputReportingStrategy() = DiagnosticReporting.Strategy {
            validationOutput, errorReporter, map -> validationOutput.lines().forEach { messageLine ->
                if (messageLine.isNotBlank() && mapper.readValue(messageLine, RustOutput::class.java).reason == COMPILER_MESSAGE_REASON) {
                    val message = mapper.readValue(messageLine, RustCompilerMessage::class.java).message
                    if (message.spans.isEmpty()) errorReporter.report(null, message.severity, message.message)
                    for (s: RustSpan in message.spans) {
                        val p: Path? = getMetadata()?.workspaceRoot?.resolve(s.fileName)
                        map[p]?.let {
                            for (lfSourcePath: Path in it.lfSourcePaths()) {
                                errorReporter.report(
                                    lfSourcePath,
                                    message.severity,
                                    DiagnosticReporting.messageOf(message.message, p, s.start),
                                    it.adjusted(lfSourcePath, s.start),
                                    it.adjusted(lfSourcePath, s.end),
                                )
                            }
                        }
                    }
                    message.rendered?.let { println(it) }
                }
            }
        }

        override fun getPriority(): Int = 0

        override fun isFullBatch(): Boolean = true
    })

    override fun getBuildReportingStrategies(): Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy> = Pair(
        possibleStrategies.first().errorReportingStrategy, possibleStrategies.first().outputReportingStrategy
    )
}
