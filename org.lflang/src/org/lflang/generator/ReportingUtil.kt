package org.lflang.generator

import org.eclipse.xtext.diagnostics.Severity
import org.eclipse.xtext.validation.Issue
import java.io.IOException
import java.io.PrintStream
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.util.*
import kotlin.math.max
import kotlin.math.min
import kotlin.system.exitProcess


/**
 * Abstraction over output streams. This is provided in case
 * we want to mock an environment for tests.
 */
class Io @JvmOverloads constructor(
    val err: PrintStream = System.err,
    val out: PrintStream = System.out,
    val wd: Path = Paths.get("").toAbsolutePath()
)

/**
 * Helper class to print messages from [Main].
 * This contains a nice issue formatter that looks like what
 * the rust compiler produces.
 *
 * @param io             Environment of the process, contains IO streams
 * @param colors         An instance of the ANSI formatter to use
 * @param numLinesAround Number of lines of context to include
 *                       around error messages when printing a
 *                       code snippet from the file in which
 *                       the error originated
 *
 */
class ReportingHelper @JvmOverloads constructor(
    private val io: Io,
    private val colors: AnsiColors = AnsiColors(true),
    private val numLinesAround: Int = 2,
) {
    // absolute path to lines
    private val fileCache = mutableMapOf<Path, List<String>?>()
    private val HEADER = colors.bold("lfc: ")

    private fun getLines(path: Path?): List<String>? =
        if (path == null) null
        else fileCache.computeIfAbsent(path.toAbsolutePath()) {
            try {
                Files.readAllLines(it, StandardCharsets.UTF_8)
            } catch (e: IOException) {
                null
            }
        }


    /** Print a fatal error message to [Io.err] and exit with code 1. */
    @JvmOverloads
    fun printFatalErrorAndExit(message: String, cause: Throwable? = null): Nothing {
        printFatalError(message, cause)
        exitProcess(1)
    }

    /** Print a fatal error message to [Io.err] and exit with code 1. */
    @JvmOverloads
    fun printFatalError(message: String, cause: Throwable? = null) {
        io.err.println(HEADER + colors.redAndBold("fatal error: ") + colors.bold(message))
        cause?.printStackTrace(io.err)
    }

    /** Print an error message to [Io.err]. */
    fun printError(message: String) {
        io.err.println(HEADER + colors.redAndBold("error: ") + message)
    }

    /** Print a warning message to [Io.err]. */
    fun printWarning(message: String) {
        io.err.println(HEADER + colors.yellowAndBold("warning: ") + message)
    }

    /** Print an informational message to [Io.out]. */
    fun printInfo(message: String) {
        io.out.println(HEADER + colors.bold("info: ") + message)
    }

    /**
     * Print a nicely formatted view of the region of code
     * surrounding the given [issue]. The issue is assumed to
     * have been found in a file located at the given [path].
     */
    fun printIssue(issue: Issue, path: Path?) {
        val severity = issue.severity
        val filePath = Paths.get(issue.uriToProblem.toFileString()).normalize()

        val header = severity.name.toLowerCase(Locale.ROOT)

        var fullMessage: String = HEADER + colors.severityColors(header, severity) + colors.bold(": " + issue.message) + "\n"
        val snippet: String? = formatIssue(issue, path)

        if (snippet == null) {
            val displayPath: Path = io.wd.relativize(filePath)
            fullMessage += " --> " + displayPath + ":" + issue.lineNumber + ":" + issue.column
            fullMessage += " - " + issue.message
        } else {
            fullMessage += snippet
        }
        io.err.println(fullMessage)
        io.err.println()
    }

    private fun formatIssue(issue: Issue, path: Path?): String? {
        val lines = getLines(path) ?: return null

        fun Int?.isInvalid() = this == null || this <= 0

        // those are nullable and need to be checked
        if (issue.lineNumber.isInvalid()
            || issue.column.isInvalid()
            || issue.length == null
        ) return null

        val fileDisplayName = path?.let { io.wd.relativize(path) }?.toString() ?: "(unknown file)"

        return getBuilder(issue, lines, fileDisplayName).build()
    }

    private fun getBuilder(issue: Issue, lines: List<String>, displayPath: String): MessageTextBuilder {
        val zeroL = issue.lineNumber - 1
        val firstL = max(0, zeroL - numLinesAround + 1)
        val lastL = min(lines.size, zeroL + numLinesAround)
        val strings: List<String> = lines.subList(firstL, lastL)
        return MessageTextBuilder(strings, firstL, zeroL - firstL, displayPath, issue)
    }

    /** Renders a single issue. */
    inner class MessageTextBuilder(
        private val lines: List<String>,
        /** Line number of the first line of the list in the real document, one-based. */
        private val first: Int,
        /** Index in the list of the line that has the error, zero-based.  */
        private val errorIdx: Int,
        private val fileDisplayName: String,
        private val issue: Issue
    ) {

        init {
            assert(0 <= errorIdx && errorIdx < lines.size) { "Weird indices --- first=$first, errorIdx=$errorIdx, lines=$lines" }
        }

        fun build(): String {
            // the padding to apply to line numbers
            val pad = 2 + widthOfLargestLineNum()
            val withLineNums: MutableList<String> =
                lines.indices.mapTo(ArrayList()) { numberedLine(it, pad) }

            withLineNums.add(errorIdx + 1, makeErrorLine(pad))
            withLineNums.add(errorIdx + 2, emptyGutter(pad)) // skip a line

            // skip a line at the beginning
            // add it at the end to not move other indices
            withLineNums.add(0, emptyGutter(pad))
            withLineNums.add(0, makeHeaderLine(pad))

            return withLineNums.joinToString("\n")
        }

        private fun widthOfLargestLineNum() = (lines.size + first).toString().length

        /**
         * This formats the first line as
         *     --> src/Foo.lf:1:3
         * where the arrow is aligned on the gutter of the line numbers
         */
        private fun makeHeaderLine(pad: Int): String {
            val prefix = formatLineNum("-->".padStart(pad))

            return "$prefix $fileDisplayName:${issue.lineNumber}:${issue.column}"
        }


        private fun makeErrorLine(pad: Int): String {
            val caretLine = with(issue) { buildCaretLine(message.trim(), column, length) }
            // gutter has its own ANSI stuff so only caretLine gets severityColors
            return emptyGutter(pad) + colors.severityColors(caretLine, issue.severity)
        }

        private fun numberedLine(idx: Int, pad: Int): String {
            val lineNum = 1 + idx + first
            val line = lines[idx]
            return formatLineNum("$lineNum |".padStart(pad)) + " $line"
        }

        private fun emptyGutter(pad: Int): String = formatLineNum("|".padStart(pad))

        private fun formatLineNum(str: String) = colors.cyanAndBold(str)

        private fun buildCaretLine(message: String, column: Int, rangeLen: Int): String {
            fun StringBuilder.repeatChar(c: Char, n: Int) {
                repeat(n) { append(c) }
            }

            return buildString {
                repeatChar(' ', column)
                repeatChar('^', max(rangeLen, 1))
                append(' ').append(message)
            }
        }
    }
}


/**
 * A strategy to add colors to messages. This uses ANSI escape
 * sequences, and can be disabled.
 *
 * @param useAnsi If true, colors will be used, otherwise all
 *                functions of this class return their argument
 *                without change
 */
class AnsiColors(private val useAnsi: Boolean) {

    private fun apply(s: String, f: () -> String) =
        if (useAnsi) f() else s

    /** Return the given string in bold face. */
    fun bold(s: String): String = apply(s) { "$BOLD$s$END_BOLD" }

    /** Return the given string in red color and bold face. */
    fun redAndBold(s: String): String = apply(s) { "$RED_BOLD$s$ANSI_RESET" }

    /** Return the given string in yellow color and bold face.  */
    fun yellowAndBold(s: String): String = apply(s) { "\u001b[1;33m$s$ANSI_RESET" }

    /** Return the given string in cyan color and bold face.  */
    fun cyanAndBold(s: String): String = apply(s) { "\u001b[1;36m$s$ANSI_RESET" }


    /** Add a color determined by message severity. */
    fun severityColors(message: String, severity: Severity): String = apply(message) {
        when (severity) {
            Severity.ERROR   -> redAndBold(message)
            Severity.WARNING -> yellowAndBold(message)
            else             -> bold(message)
        }
    }


    companion object {
        /** ANSI sequence color escape sequence for red bold font. */
        private const val RED_BOLD = "\u001b[1;31m"

        /** ANSI sequence color escape sequence for resetting all attributes. */
        private const val ANSI_RESET = "\u001b[0m"

        /** ANSI sequence color escape sequence for bold font. */
        private const val BOLD = "\u001b[1m"

        /** ANSI sequence color escape sequence for ending bold font. */
        private const val END_BOLD = "\u001b[0m"
    }
}
