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

import com.google.common.collect.ImmutableList;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/**
 * Representation of a permuted mixed radix (PMR) integer. A mixed radix number is a number
 * representation where each digit can have a distinct radix. The radixes are given by a list of
 * numbers, r0, r1, ... , rn, where r0 is the radix of the lowest-order digit and rn is the radix of
 * the highest order digit that has a specified radix.
 *
 * <p>A PMR is a mixed radix number that, when incremented, increments the digits in the order given
 * by the permutation matrix. For an ordinary mixed radix number, the permutation matrix is [0, 1,
 * ..., n-1]. The permutation matrix may be any permutation of these digits, [d0, d1, ..., dn-1], in
 * which case, when incremented, the d0 digit will be incremented first. If it overflows, it will be
 * set to 0 and the d1 digit will be incremented. If it overflows, the next digit is incremented. If
 * the last digit overflows, then the number wraps around so that all digits become zero.
 *
 * <p>This implementation realizes a finite set of numbers, where incrementing past the end of the
 * range wraps around to the beginning. As a consequence, the increment() function from any starting
 * point is guaranteed to eventually cover all possible values.
 *
 * <p>The {@link #toString()} method gives a string representation of the number where each digit is
 * represented by the string "d%r", where d is the digit and r is the radix. For example, the number
 * "1%2, 2%3, 1%4" has value 11, 1 + (2*2) + (1*2*3).
 *
 * @author Edward A. Lee
 */
public class MixedRadixInt {

  /**
   * Create a mixed radix number with the specified digits and radixes, which are given low-order
   * digits first. If there is one more digit than radixes, throw an exception.
   *
   * @param digits The digits, or null to get a zero-valued number.
   * @param radixes The radixes.
   * @param permutation The permutation matrix, or null for the default permutation.
   */
  public MixedRadixInt(List<Integer> digits, List<Integer> radixes, List<Integer> permutation) {
    if (radixes == null
        || (digits != null && digits.size() > radixes.size())
        || (permutation != null && permutation.size() != radixes.size())) {
      throw new IllegalArgumentException("Invalid constructor arguments.");
    }
    this.radixes = ImmutableList.copyOf(radixes);
    if (digits != null) {
      this.digits = digits;
    } else {
      this.digits = new ArrayList<>(1);
      this.digits.add(0);
    }
    if (permutation != null) {
      // Check the permutation matrix.
      Set<Integer> indices = new HashSet<>();
      for (int p : permutation) {
        if (p < 0 || p >= radixes.size() || indices.contains(p)) {
          throw new IllegalArgumentException(
              "Permutation list is required to be a permutation of [0, 1, ... , n-1].");
        }
        indices.add(p);
      }
      this.permutation = permutation;
    }
  }

  /** A zero-valued mixed radix number with just one digit will radix 1. */
  public static final MixedRadixInt ZERO = new MixedRadixInt(null, List.of(1), null);

  //////////////////////////////////////////////////////////
  //// Public methods

  /** Get the value as an integer. */
  public int get() {
    return get(0);
  }

  /**
   * Get the value as an integer after dropping the first n digits.
   *
   * @param n The number of digits to drop.
   */
  public int get(int n) {
    int result = 0;
    int scale = 1;
    if (n < 0) n = 0;
    for (int i = n; i < radixes.size(); i++) {
      if (i >= digits.size()) return result;
      result += digits.get(i) * scale;
      scale *= radixes.get(i);
    }
    return result;
  }

  /** Return the digits. This is assured of returning as many digits as there are radixes. */
  public List<Integer> getDigits() {
    while (digits.size() < radixes.size()) {
      digits.add(0);
    }
    return digits;
  }

  /** Return the permutation list. */
  public List<Integer> getPermutation() {
    if (permutation == null) {
      // Construct a default permutation.
      permutation = new ArrayList<>(radixes.size());
      for (int i = 0; i < radixes.size(); i++) {
        permutation.add(i);
      }
    }
    return permutation;
  }

  /** Return the radixes. */
  public List<Integer> getRadixes() {
    return radixes;
  }

  /**
   * Increment the number by one, using the permutation vector to determine the order in which the
   * digits are incremented. If an overflow occurs, then a radix-infinity digit will be added to the
   * digits array if there isn't one there already.
   */
  public void increment() {
    int i = 0;
    while (i < radixes.size()) {
      int digit_to_increment = getPermutation().get(i);
      while (digit_to_increment >= digits.size()) {
        digits.add(0);
      }
      digits.set(digit_to_increment, digits.get(digit_to_increment) + 1);
      if (digits.get(digit_to_increment) >= radixes.get(digit_to_increment)) {
        digits.set(digit_to_increment, 0);
        i++;
      } else {
        return; // All done.
      }
    }
  }

  /**
   * Return the magnitude of this PMR, which is defined to be the number of times that increment()
   * would need to invoked starting with zero before the value returned by {@link #get()} would be
   * reached.
   */
  public int magnitude() {
    int factor = 1;
    int result = 0;
    List<Integer> p = getPermutation();
    for (int i = 0; i < radixes.size(); i++) {
      if (digits.size() <= i) return result;
      result += factor * digits.get(p.get(i));
      factor *= radixes.get(p.get(i));
    }
    return result;
  }

  /**
   * Return the number of digits in this mixed radix number. This is the size of the radixes list.
   */
  public int numDigits() {
    return radixes.size();
  }

  /**
   * Set the value of this number to equal that of the specified integer.
   *
   * @param v The ordinary integer value of this number.
   */
  public void set(int v) {
    // it does not make sense to call set
    int temp = v;
    int count = 0;
    for (int radix : radixes) {
      var digit = radix == 0 ? 0 : temp % radix;
      if (count >= digits.size()) {
        digits.add(digit);
      } else {
        digits.set(count, digit);
      }
      count++;
      temp = temp == 0 ? temp : temp / radix;
    }
  }

  /**
   * Set the magnitude of this number to equal that of the specified integer, which is the number of
   * times that increment must be invoked from zero for the value returned by {@link #get()} to
   * equal v.
   *
   * @param v The new magnitude of this number.
   */
  public void setMagnitude(int v) {
    int temp = v;
    for (int i = 0; i < radixes.size(); i++) {
      int p = getPermutation().get(i);
      while (digits.size() < p + 1) digits.add(0);
      var r = radixes.get(p);
      if (r == 0 && v == 0) {
        digits.set(p, 0); // zero does not make sense here, but we have to put something.
      } else {
        digits.set(p, temp % r);
        temp = temp / r;
      }
    }
  }

  /**
   * Give a string representation of the number, where each digit is represented as n%r, where r is
   * the radix.
   */
  @Override
  public String toString() {
    List<String> pieces = new LinkedList<>();
    Iterator<Integer> radixIterator = radixes.iterator();
    for (int digit : digits) {
      if (!radixIterator.hasNext()) {
        pieces.add(digit + "%infinity");
      } else {
        pieces.add(digit + "%" + radixIterator.next());
      }
    }
    return String.join(", ", pieces);
  }

  @Override
  public int hashCode() {
    int sum = 0;
    for (var radix : radixes) sum = sum * 31 + radix;
    for (var digit : digits) sum = sum * 31 + digit;
    for (var p : permutation) sum = sum * 31 + p;
    return sum;
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof MixedRadixInt mri
        && radixes.equals(mri.radixes)
        && digits.equals(mri.digits)
        && permutation.equals(mri.permutation);
  }

  public MixedRadixInt copy() {
    return new MixedRadixInt(List.copyOf(digits), List.copyOf(radixes), List.copyOf(permutation));
  }

  //////////////////////////////////////////////////////////
  //// Private variables

  private final ImmutableList<Integer> radixes;
  private final List<Integer> digits;
  private List<Integer> permutation;
}
