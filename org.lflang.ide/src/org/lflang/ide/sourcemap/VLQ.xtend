package org.lflang.ide.sourcemap

/**
 * Provides utility methods for handling string representations of relative
 * positions, as specified in the
 * <a href="https://sourcemaps.info/spec.html">Source Map Revision 3
 * Proposal</a>
 */
class VLQ {
	
	val static BASE64_MAP = ('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvw'
	    + 'xyz0123456789+/').toCharArray
	
	/**
     * Returns the string representation of an integer using base-64 VLQ
     * encoding.
     * 
     * @param x the number to be represented as a string
     * @return the string representation of x using base-64 VLQ
     *     encoding
     */
	def static String toVLQ(int x) {
		val StringBuilder out = new StringBuilder
		var int y = Math.abs(x)
		var n = (y % 16) * 2
		// This code is painfully verbose and should probably be written in terms of
		// basic operations such as bit shifts. It was originally written
		// using ternary operators, but when that operator was used in this file,
		// I observed that the editor froze for a few seconds after every few
		// keystrokes. I suspect that this is a bug in the Xtend language support.
		if (x < 0) {
			n += 1
		}
		if (y > 15) {
			n += 32
		}
		out.append(BASE64_MAP.get(n))
		y = y / 16
		while (y > 0) {
			n = y % 32
			if (y > 31) {
				n += 32
			}
			out.append(BASE64_MAP.get(n))
			y = y / 32
		}
		return out.toString()
	}
	
	/**
     * Converts a string that uses base-64 VLQ encoding to an array of
     * integers.
     * 
     * @param s the string representation of some number using base-64
     *     VLQ encoding
     * @return the number represented by s
     */
	def static int[] fromVLQ(String s) {
		val int[] in = s.toCharArray.map [ c | toIntBase64(c) ]
		val int[] out = newIntArrayOfSize(s.length)
		var i = 0
		var current = 0
		for (var pos = 0; pos < s.length; pos = current) {
			for (; in.get(current) < 32; current++) {
				out.set(i,
					(in.get(current) % 32) / (current == 0 ? 2 : 1)
				)
			}
			if (in.get(pos) % 2 > 0) {
				out.set(i, -out.get(i))
			}
		}
		val int[] ret = newIntArrayOfSize(i)
		for (var j = 0; j < i; j++) ret.set(j, out.get(j))
		return ret
	}
	
	/**
     * Converts a character to the corresponding integer in the interval
     * [0, 63]
     * 
     * @param c a character corresponding to an integer in the interval
     *     [0, 63]
     * @return the integer represented by c
     */
	def private static int toIntBase64(char c) throws IllegalArgumentException {
		for (var i = 0; i < BASE64_MAP.length; i++) {
			if (BASE64_MAP.get(i) == c) {
				return i
			}
		}
		throw new IllegalArgumentException
	}
}