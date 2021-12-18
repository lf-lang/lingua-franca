/* A representation for a mixed radix number. */

/*
Copyright (c) 2019-2021, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.lflang.generator;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Representation of a mixed radix integer.
 * A mixed radix number is a number representation where each digit can have
 * a distinct radix. The radixes are given by a list of numbers, r0, r1, ... , rn,
 * where r0 is the radix of the lowest-order digit and rn is the radix of the
 * highest order digit that has a specified radix. There is an additional implict
 * radix after rn of infinity, which means that the mixed radix number may have a
 * total n + 2 digits, where the last digit is unbounded.
 * 
 * The {@link #toString()} method gives a string representation of the number
 * where each digit is represented by the string "d%r", where d is the digit
 * and r is the radix. For example, the number "1%2, 2%3, 1%4" has value 11,
 * 1 + (2*2) + (1*2*3).
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
public class MixedRadixInt {
    
    /**
     * Create a mixed radix number with the specified digits and radixes,
     * which are given low-order digits first.
     * If there is one more digit than radixes, then the last digit is
     * assumed to have infinite radix. If there are more digits than that,
     * throw an exception.
     * @param digits The digits.
     * @param radixes The radixes.
     */
    public MixedRadixInt(List<Integer> digits, List<Integer> radixes) {
        if (radixes == null || (digits != null && digits.size() > radixes.size() + 1)) {
            throw new IllegalArgumentException("Invalid constructor arguments.");
        }
        this.radixes = radixes;
        if (digits != null) {
            this.digits = digits;
        } else {
            this.digits = new ArrayList<Integer>(1);
            this.digits.add(0);
        }
    }

    //////////////////////////////////////////////////////////
    //// Public methods

    /**
     * Get the value as an integer.
     */
    public int get() {
        int result = 0;
        int scale = 1;
        Iterator<Integer> radixIterator = radixes.iterator();
        for (int digit : digits) {
            result += digit * scale;
            if (radixIterator.hasNext()) {
                scale *= radixIterator.next();
            }
        }
        return result;
    }
    
    /**
     * Increment the number by one.
     */
    public void increment() {
        increment(0);
    }
    
    /**
     * Permute the digits of the number and return a new number.
     * The argument is required to be a list of indices from 0 to L-1,
     * where L is less than or equal to the length of the radixes list
     * specified in the constructor. Each index in this list specifies
     * the index from which the corresponding digit should be taken.
     * 
     * For example, if this number is "1%2, 2%5", which has value 5 = 1 + 2*2,
     * then permute([1, 0]) will return the number "2%5, 1%2", which
     * has value 7 = 2 + 1*5.
     * 
     * If this number has a final radix-infinity term, then that term
     * will be included in the result as a radix-infinity term.
     * 
     * @param permutation The permutation array.
     * @throws IllegalArgumentException If the argument size does not equal
     *  the radixes size.
     */
    public MixedRadixInt permute(List<Integer> permutation) {
        if (permutation.size() > radixes.size()) {
            throw new IllegalArgumentException(
                    "Permutation list cannot be larger than the radixes, " 
                    + radixes.size());
        }
        List<Integer> newRadixes = new ArrayList<Integer>(radixes.size());
        List<Integer> newDigits = new ArrayList<Integer>(digits.size());
        for (int p : permutation) {
            newRadixes.add(radixes.get(p));
            newDigits.add(digits.get(p));
        }
        // Handle possible radix-infinity digit.
        if (digits.size() > radixes.size()) {
            newDigits.add(digits.get(digits.size() - 1));
        }
        return new MixedRadixInt(newDigits, newRadixes);
    }
    
    /**
     * Set the value of this number to equal that of the specified integer.
     * @param v The ordinary integer value of this number.
     */
    public void set(int v) {
        int temp = v;
        int count = 0;
        for (int radix : radixes) {
            if (count >= digits.size()) {
                digits.add(temp % radix);
            } else {
                digits.set(count, temp % radix);
            }
            count++;
            temp = temp / radix;
        }
    }
    
    /**
     * Give a string representation of the number, where each digit is
     * represented as n%r, where r is the radix.
     */
    @Override
    public String toString() {
        List<String> pieces = new LinkedList<String>();
        Iterator<Integer> radixIterator = radixes.iterator();
        for (int digit : digits) {
            if (! radixIterator.hasNext()) {
                pieces.add(digit + "%infinity");
            } else {
                pieces.add(digit + "%" + radixIterator.next());
            }
        }
        return String.join(", ", pieces);
    }

    //////////////////////////////////////////////////////////
    //// Private methods

    /**
     * Increment the specified digit by one.  If an overflow occurs,
     * then a radix-infinity digit will be added to the digits array
     * if there isn't one there already.
     * @param d The digit to increment, which assumed to be less than
     *  the length of digits.
     */
    public void increment(int d) {
        int value = digits.get(d) + 1;
        if (d >= radixes.size()) {
            // We are at a radix infinity digit.
            digits.set(d, value);
            return;
        }
        if (value < radixes.get(d)) {
            digits.set(d, value);
            return;
        } else {
            digits.set(d, 0);
            if (d < digits.size() - 1) {
                increment(d + 1);
            } else {
                digits.add(1);
            }
            return;
        }
    }

    //////////////////////////////////////////////////////////
    //// Private variables

    private List<Integer> radixes;
    private List<Integer> digits;
}
