/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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

package org.lflang

import org.eclipse.xtext.validation.Issue
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.util.*
import kotlin.math.max
import kotlin.math.min


/**
 * Parse and return an integer from this string, much
 * like [String.toIntOrNull], but allows any radix.
 *
 * @see Integer.decode
 */
internal fun String.toIntOrNullAnyRadix(): Int? =
    try {
        Integer.decode(this)
    } catch (e: NumberFormatException) {
        null
    }

/**
 * Return the sublist consisting of the tail elements of this list,
 * ie, everything except the first elements. This is a list view,
 * and does not copy the backing buffer (if any).
 *
 * @throws NoSuchElementException if the list is empty
 */
internal fun <T> List<T>.tail() = subList(1, size)

/**
 * Return a pair consisting of the [List.first] element and the [tail] sublist.
 * This may be used to deconstruct a list recursively, as is usual in
 * functional languages.
 *
 * @throws NoSuchElementException if the list is empty
 */
internal fun <T> List<T>.headAndTail() = Pair(first(), tail())

/**
 * Return [this] string surrounded with double quotes.
 */
internal fun String.withDQuotes() = "\"$this\""


/**
 * Remove quotation marks (double XOR single quotes)
 * surrounding the specified string.
 */
internal fun String.withoutQuotes(): String {
    val r = removeSurrounding("\"")
    return if (r !== this) this else removeSurrounding("'")
}

/**
 * Join this list into a comma-separated string. The toString
 * of members is used. Space must be irrelevant.
 */
internal fun List<CharSequence>.joinWithCommas() = joinToString(", ") { it }


/**
 * Convert a string in Camel case to snake case. E.g.
 * `MinimalReactor` will be converted to `minimal_reactor`.
 * The string is assumed to be a single camel case identifier
 * (no whitespace).
 */
fun String.camelToSnakeCase(): String {
    val words = this.split(Regex("(?<![A-Z])(?=[A-Z])"))
        .map { it.toLowerCase(Locale.ROOT) }
        .filter { it.isNotEmpty() }

    return words.joinToString("_")
}

private val nlPattern = Regex("\\R\\s+")

/**
 * Replace newlines with a single space.
 */
fun String.joinLines(): String = replace(nlPattern, " ")

/**
 * Something to throw to convince the compiler a branch is
 * unreachable.
 */
fun unreachable(message: String? = null): Nothing =
    throw AssertionError("Unreachable branch" + message?.let { ": $it" }.orEmpty())

/**
 * Return a nicely formatted view of the region of code
 * surrounding the given [issue]. The issue is assumed to
 * have been found in a file located at the given [path].
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
fun getCodeSnippet(path: Path, issue: Issue, numLinesAround: Int = 3): String? {
    val lines = try {
        Files.readAllLines(path, StandardCharsets.UTF_8)
    } catch (e: IOException) {
        return null
    }

    return getBuilder(issue, lines, numLinesAround).build(issue, path)
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

    fun build(issue: Issue, path: Path): String {
        val pad = stringLengthOf(lines.size + first)
        val withLineNums: MutableList<String> =
            lines.indices.mapTo(ArrayList()) { addLineNum(it, pad) }

        val errorLine = addLineNum(errorIdx, pad)
        // diff added by line numbers
        val offset = errorLine.length - lines[errorIdx].length
        val messageLine: String = buildCaretLine(
            issue.message.trim(),
            issue.column + offset - 1,
            issue.length
        )
        withLineNums.add(errorIdx + 1, messageLine)
        withLineNums.add(errorIdx + 2, "") // skip a line

        return  withLineNums.joinToString("\n")
    }

    private fun stringLengthOf(i: Int): Int = i.toString().length

    private fun addLineNum(idx: Int, pad: Int): String =
        String.format(" %" + pad + "d| %s", 1 + idx + first, lines[idx])

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
