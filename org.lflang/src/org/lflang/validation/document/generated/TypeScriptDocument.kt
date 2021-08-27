package org.lflang.validation.document.generated

import kotlinx.serialization.KSerializer
import kotlinx.serialization.Serializable
import org.lflang.generator.ts.sourcemap.getSourceMapOf
import org.lflang.generator.ts.sourcemap.SourceMap
import org.lflang.generator.ts.sourcemap.SourceMapSegment
import org.lflang.validation.document.DiagnosticAcceptor.Severity

import java.io.File
import java.io.IOException
import java.util.NavigableMap
import java.util.TreeMap

import kotlin.collections.List as ListKt
import kotlinx.serialization.json.Json
import kotlinx.serialization.decodeFromString
import org.lflang.validation.document.DiagnosticAcceptor
import org.lflang.validation.document.Position

class TypeScriptDocument(lines: MutableList<String>, directory: File) :
    GeneratedDocument(lines, getMap(lines, directory), directory) {

    @Serializable
    data class ESLintDiagnosticsGroup(
        val filePath: String,
        val messages: kotlin.collections.List<ESLintDiagnostic>,
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
    data class ESLintFix(val range: kotlin.collections.List<Int>, val text: String)

    /* ---------------------  PROTECTED METHODS  ------------------------ */

    override fun getVerificationProcess(): ProcessBuilder {
        val eslint = ProcessBuilder(
            "npx", "eslint", "--stdin",
            "--format", "json"
        )
        var srcGenRoot: File = directory
        while (!srcGenRoot.parentFile.name.equals("src-gen")) {
            srcGenRoot = srcGenRoot.parentFile
        }
        eslint.directory(srcGenRoot)
        return eslint
    }

    override fun addDiagnostic(line: String, acceptor: DiagnosticAcceptor) {
        if (line.isBlank()) return
        val diagnosticsGroups = Json.decodeFromString<ListKt<ESLintDiagnosticsGroup>>(line);
        for (group: ESLintDiagnosticsGroup in diagnosticsGroups) {
            // FIXME: diagnosticsGroups should always have length 1 (or maybe zero) -- unless there
            //  actually is a many-to-many relationship between source files and generated documents.
            //  But in that would be a larger problem than just this method.
            for (diagnostic: ESLintDiagnostic in group.messages) {
                acceptor.acceptDiagnostic(
                    intToSeverity(diagnostic.severity),
                    diagnostic.message,
                    Position.fromOneBased(diagnostic.line, diagnostic.column),
                    Position.fromOneBased(diagnostic.endLine, diagnostic.endColumn)
                )
            }
        }
    }

    override fun getVerificationUsesStdout(): Boolean {
        return true;
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    companion object {
        /**
         * Returns a <code>NavigableMap</code> with mappings
         * from the content of a generated TypeScript document
         * to the content of the source code from which it
         * was generated.
         * @param lines the contents of the TypeScript document
         *              of interest
         * @param directory the directory in which the
         *                  TypeScript document lives
         * @return a <code>NavigableMap</code> with mappings
         * from the content of a generated TypeScript document
         * to the content of the source code from which it
         * was generated
         */
        private fun getMap(lines: MutableList<String>, directory: File): NavigableMap<Position, Position> {
            val ret: NavigableMap<Position, Position> = TreeMap();
            val sourceMap: SourceMap?;
            try {
                sourceMap = getSourceMapOf(lines, directory)
            } catch (e: IOException) {
                return ret;
            }
            sourceMap?.mappings?.let { mappings ->
                for (segment: SourceMapSegment? in mappings) {
                    segment?.let {
                        ret.put(
                            Position.fromZeroBased(it.getTargetLine(), it.getTargetColumn()),
                            Position.fromZeroBased(it.getSourceLine(), it.getSourceColumn())
                        );
                    }
                }
            }
            return ret;
        }

        /**
         * Returns the severity level corresponding to
         * <code>severity</code>.
         */
        private fun intToSeverity(severity: Int): Severity {
            // FIXME: Factor out?
            return when (severity) {
                2 -> Severity.ERROR;
                1 -> Severity.WARNING;
                else -> Severity.INFO; // FIXME: This code is probably unreachable in practice.
            }
        }
    }
}
