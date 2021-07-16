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

import java.util.Locale

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
