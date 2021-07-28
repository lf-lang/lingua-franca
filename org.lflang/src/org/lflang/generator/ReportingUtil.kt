package org.lflang.generator

import org.eclipse.xtext.validation.Issue
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.util.ArrayList
import kotlin.math.max
import kotlin.math.min

/**
 * Return a nicely formatted view of the region of code
 * surrounding the given [issue]. The issue is assumed to
 * have been found in a file located at the given [absolutePath].
 * The [numLinesAround] parameter determines how many lines
 * of context are fetched.
 *
 * This generates for instance:
 *
 *     22|     timer toe(z);       // Implicit type time
 *     23|     state baz({=p=});   // Implicit type i32[]
 *             ^^^^^^^^^^^^^^^^^ State must have a type.
 *
 *     24|     state period(z);    // Implicit type time
 */
@JvmOverloads
fun getCodeSnippet(
    absolutePath: Path,
    displayPath: Path,
    issue: Issue,
    numLinesAround: Int = 3,
    useColors: Boolean = true,
    colorMessage: (String) -> String = { it }
): String? {
    val lines = try {
        Files.readAllLines(absolutePath, StandardCharsets.UTF_8)
    } catch (e: IOException) {
        return null
    }
    with(issue) {
        // those are nullable and need to be checked
        if (lineNumber == null
            || column == null
            || length == null) return@getCodeSnippet null
    }
    return getBuilder(issue, lines, numLinesAround).build(issue, colorMessage, useColors, displayPath)
}

private fun getBuilder(issue: Issue, lines: List<String>, numLinesAround: Int = 3): MessageTextBuilder {
    val zeroL = issue.lineNumber - 1
    val firstL = max(0, zeroL - numLinesAround + 1)
    val lastL = min(lines.size, zeroL + numLinesAround)
    val strings: List<String> = lines.subList(firstL, lastL)
    return MessageTextBuilder(strings, firstL, zeroL - firstL)
}

/** Format lines around the [errorIdx] to display the message. */
internal class MessageTextBuilder(
    private val lines: List<String>,
    /** Line number of the first line of the list in the real document, one-based. */
    private val first: Int,
    /** Index in the list of the line that has the error, zero-based.  */
    private val errorIdx: Int
) {

    init {
        assert(0 <= errorIdx && errorIdx < lines.size) { "Weird indices --- first=$first, errorIdx=$errorIdx, lines=$lines" }
    }

    fun build(
        issue: Issue,
        colorMessage: (String) -> String = { it },
        useColors: Boolean,
        path: Path
    ): String {
        val pad = stringLengthOf(lines.size + first)
        val withLineNums: MutableList<String> =
            lines.indices.mapTo(ArrayList()) { addLineNum(it, pad, useColors) }

        withLineNums.add(errorIdx + 1, makeErrorLine(pad, useColors, issue, colorMessage))
        withLineNums.add(errorIdx + 2, emptyLine(pad, useColors)) // skip a line

        // skip a line at the beginning
        // add it at the end to not move other indices
        withLineNums.add(0, emptyLine(pad, useColors))
        withLineNums.add(0, makeHeaderLine(pad, useColors, path, issue))

        return withLineNums.joinToString("\n")
    }

    /**
     * This formats the first line as
     *     --> src/Foo.lf:1:3
     * where the arrow is aligned on the gutter of the line numbers
     */
    private fun makeHeaderLine(pad: Int, useColors: Boolean, path: Path, issue: Issue): String {
        val prefix = String.format(" %${pad}s ", "-->").let { formatLineNum(it, useColors) }

        return "$prefix $path:${issue.lineNumber}:${issue.column}"
    }


    private fun makeErrorLine(
        pad: Int,
        useColors: Boolean,
        issue: Issue,
        colorMessage: (String) -> String
    ): String {
        val prefix = emptyLine(pad, useColors)
        val caretLine = with(issue) { buildCaretLine(message.trim(), column, length) }
        return if (useColors) prefix + colorMessage(caretLine)  // prefix contains an ANSI_RESET
        else prefix + caretLine
    }

    private fun stringLengthOf(i: Int): Int = i.toString().length

    private fun addLineNum(idx: Int, pad: Int, colors: Boolean): String =
        formatLineNum(" %${pad}d |", colors).let { prefix ->
            String.format("$prefix %s", 1 + idx + first, lines[idx])
        }

    private fun emptyLine(pad: Int, colors: Boolean): String =
        String.format(" %${pad}s |", "").let {
            formatLineNum(it, colors)
        }

    private fun formatLineNum(str: String, useColors: Boolean) =
        if (useColors) Main.cyanAndBold(str)
        else str


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
