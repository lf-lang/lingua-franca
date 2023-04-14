/**
 * @file
 * @author Muhammad Khubaib Umer (khubaib@magnition.io)
 *
 * @section LICENSE
Copyright (c) 2023, MagnitionIO
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
 * @section DESCRIPTION
 *
 * This Class provides a Pair of Templated Types
 * This provides the same facility as std::pair<T, U> in C++
 */

package org.lflang.util;

public final class Pair<T, U> {
    private final T first;
    private final U second;
    public Pair(final T t, final U u)
    {
        this.first = t;
        this.second = u;
    }

    public T getFirst()
    {
        return this.first;
    }

    public U getSecond()
    {
        return this.second;
    }

    @Override
    public boolean equals(Object other) {
        if (other == null)
            return false;
        if (this == other)
            return true;
        if (this.getClass().equals(other.getClass())) {
            Pair<?, ?> otherPair = (Pair<?, ?>) other;
            boolean isEqual = (first == null) ? otherPair.getFirst() == null : first.equals(otherPair.getFirst());

            if (!isEqual)
                return false;

            return (second == null) ? otherPair.getSecond() == null : second.equals(otherPair.getSecond());
        }
        return false;
    }

    @Override
    public int hashCode() {
        return first == null ? 0 : first.hashCode() + 27 * (second == null ? 0 : second.hashCode());
    }

    @Override
    public String toString() {
        return "Pair(" + first + ", " + second + ")";
    }
}
