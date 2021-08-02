package org.lflang.generator.ts.sourcemap

import kotlin.math.abs

/**
 * Provides utility methods for handling string representations of relative
 * positions, as specified in the
 * <a href="https://sourcemaps.info/spec.html">Source Map Revision 3
 * Proposal</a>
 */
object VLQ {
    private val BASE64_MAP: CharArray = ("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghi" +
            "jklmnopqrstuvwxyz0123456789+/").toCharArray()

    /**
     * Returns the string representation of an integer using base-64 VLQ
     * encoding.
     * @param x the number to be represented as a string
     * @return the string representation of x using base-64 VLQ
     *     encoding
     */
    fun toVLQ(x: Int): String {
        val out = StringBuilder()
        var y = abs(x)
        var n = (y % 16) * 2
        // This code should probably be written in terms of basic operations
        // such as bit shifts.
        if (x < 0) {
            n += 1
        }
        if (y > 15) {
            n += 32
        }
        out.append(BASE64_MAP[n])
        y /= 16
        while (y > 0) {
            n = y % 32
            if (y > 31) {
                n += 32
            }
            out.append(BASE64_MAP[n])
            y /= 32
        }
        return out.toString()
    }

    /**
     * Converts a string that uses base-64 VLQ encoding to an array of
     * integers.
     * @param s the string representation of some number using base-64
     *     VLQ encoding
     * @return the number represented by s
     */
    fun fromVLQ(s: String): IntArray {
        // TODO: Determine if return type should be non-primitive (e.g.,
        //  MutableList)
        val input: List<Int> = s.toCharArray().map { toIntBase64(it) }
        val out: MutableList<Int> = ArrayList()
        out.add(0)
        var pos = 0
        while (pos < s.length) {
            val negative: Boolean = input[pos] % 2 == 0
            var n: Int = (input[pos] % 32) / 2
            while (input[pos] > 32) {
                pos++
                n += input[pos] % 32
            }
            if (negative) {
                n = -n
            }
            out.add(n)
        }
        return out.toIntArray()
    }

    /**
     * Converts a character to the corresponding integer in the interval
     * [0, 63]
     * @param c a character corresponding to an integer in the interval [0, 63]
     * @return the integer represented by c
     */
    private fun toIntBase64(c: Char): Int {
        for (i in BASE64_MAP.indices) {
            if (BASE64_MAP[i] == c) {
                return i
            }
        }
        throw IllegalArgumentException("")
    }
}