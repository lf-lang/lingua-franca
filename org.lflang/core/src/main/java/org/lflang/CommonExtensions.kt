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

import org.lflang.generator.TargetCode
import org.lflang.util.StringUtil
import java.util.*

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
 * This escapes the string's content (see [escapeStringLiteral]).
 */
internal fun String.withDQuotes() = "\"${this.escapeStringLiteral()}\""

/**
 * Return [this] string with some common escapes to place it
 * into a string literal.
 */
fun String.escapeStringLiteral() =
    replace(Regex("[\\\\ \t\"]")) {
        when (it.value) {
            "\\" -> "\\\\"
            "\t" -> "\\t"
            "\"" -> "\\\""
            else -> it.value
        }
    }

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
fun String.camelToSnakeCase(): String = StringUtil.camelToSnakeCase(this)

private val nlPattern = Regex("\\R\\s*")

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

/** Turn the first char into uppercase. The stdlib capitalize is deprecated. */
fun String.capitalize(): String = replaceFirstChar { it.uppercaseChar() }

/** Returns true if this string is an alphanumeric identifier. */
val String.isIdentifier get() = matches(IDENT_REGEX)

/** Matches alphanumeric identifiers. */
val IDENT_REGEX = Regex("[a-zA-Z][a-zA-Z0-9_]*")


/** Join with new lines. */
fun Iterable<CharSequence>.joinLn(): String =
    joinToString("\n")

/**
 * Join [this] iterable with commas. Supports an optional
 * [trailing] comma. The [transform] is used to render each
 * item. If [skipLines] is true, a newline will additionally
 * be inserted after each item except the last. The [prefix]
 * and [postfix] are appended even if this iterable is empty.
 */
fun <T> Iterable<T>.joinWithCommas(
    prefix: CharSequence = "",
    postfix: CharSequence = "",
    skipLines: Boolean = false,
    trailing: Boolean = true,
    transform: (T) -> CharSequence = { it.toString() }
): String {
    val delim =
        (if (skipLines) "\n" else " ")
            .let { if (trailing) it else ",$it" }

    return joinToString(delim, prefix, postfix) { t ->
        transform(t).let { if (trailing) "$it," else it }
    }
}

/** Like [joinWithCommas], setting the skipLines parameter to true. */
fun <T> Iterable<T>.joinWithCommasLn(
    prefix: CharSequence = "",
    postfix: CharSequence = "",
    trailing: Boolean = true,
    transform: (T) -> CharSequence = { it.toString() }
): String = joinWithCommas(prefix, postfix, skipLines = true, trailing, transform)

/**
 * Join the elements of [this] sequence with newlines. The
 * [prefix] and [postfix] are added even if this iterable is empty.
 */
fun <T> Iterable<T>.joinWithLn(
    prefix: CharSequence = "",
    postfix: CharSequence = "",
    transform: (T) -> CharSequence
): String = joinToString(separator = "\n", prefix = prefix, postfix = postfix, transform = transform)

/**
 * Join this list with commas, surrounding it with angled brackets (`<...>`).
 * If this list is empty, returns an empty string.
 */
fun List<TargetCode>.angle() = if (this.isEmpty()) "" else joinWithCommas("<", ">")

/**
 * Adds braces around this string.
 */
fun String.inBlock(): String = "{$this}"
