/* Abstract class for ranges of NamedInstance. */

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

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.lflang.lf.Connection;

/**
 * Class representing a range of runtime instance objects
 * (port channels, reactions, etc.). This class and its derived classes
 * have the most detailed information about the structure of a Lingua Franca
 * program.  There are three levels of detail:
 * 
 * * The abstract syntax tree (AST).
 * * The instantiation graph (IG).
 * * The runtime instance graph (RIG).
 * 
 * In the AST, each reactor class is represented once.
 * In the IG, each reactor class is represented as many times as it is
 * instantiated, except that a bank has only one representation (as
 * in the graphical rendition). Equivalently, each IG node has a unique
 * full name, even though it may represent many runtime instances.
 * The IG is represented by
 * {@link NamedInstance<T extends EObject>} and its derived classes.
 * In the RIG, each bank is expanded so each bank member and
 * each port channel is represented.
 * 
 * In general, determining dependencies between reactions requires analysis
 * at the level of the RIG.  But a brute-force representation of the RIG
 * can get very large, and for most programs it has a great deal of
 * redundancy. In a fully detailed representation of the RIG, for example,
 * a bank of width N that contains a bank of width M within which there
 * is a reactor R with port P will have N*M runtime instances of the port.
 * If the port is a multiport with width L, then there are N*M*L
 * edges connected to instances of that port, each of which may go
 * to a distinct set of other remote runtime port instances.
 * 
 * This class and its subclasses give a more compact representation of the
 * RIG in most common cases where collections of runtime instances all have
 * the same dependencies.
 * 
 * A Range represents an adjacent set of RIG objects (port channels, reactions
 * reactors). Specifically, it is a list of RIG objects, each of which may have
 * a width greater than one, listed in the order in which they should be iterated
 * over, plus a start offset into the expanded list and a width of the range.
 * 
 * The simplest Ranges are those where the corresponding IG node represents
 * only one runtime instance (its ig_instance is not (deeply) within a bank 
 * and is not a multiport). In this case, the Range and all its RIG objects
 * will have width = 1.
 * 
 * In a more complex instance, consider a bank A of width 2 that contains a
 * bank B of width 2 that contains a port instance P with width 2. .
 * 
 * There are a total of 8 instances of P, which we can name:
 * 
 *      A0.B0.P0
 *      A0.B0.P1
 *      A0.B1.P0
 *      A0.B1.P1
 *      A1.B0.P0
 *      A1.B0.P1
 *      A1.B1.P0
 *      A1.B1.P1
 * 
 * If there is no interleaving, the list of RIG objects will be P, B, A,
 * indicating that they should be iterated by incrementing the index of P
 * first, then the index of B, then the index of A.
 * 
 * If the connection within B to port P is interleaved, then the order
 * of iteration will be B, P, A, resulting in the list:
 * 
 *      A0.B0.P0
 *      A0.B1.P0
 *      A0.B0.P1
 *      A0.B1.P1
 *      A1.B0.P0
 *      A1.B1.P0
 *      A1.B0.P1
 *      A1.B1.P1
 *      
 *  If the connection within A to B is also interleaved, then the order
 *  will be A, B, P, resulting in the list:
 *  
 *      A0.B0.P0
 *      A1.B0.P0
 *      A0.B1.P0
 *      A1.B1.P0
 *      A0.B0.P1
 *      A1.B0.P1
 *      A0.B1.P1
 *      A1.B1.P1
 * 
 * Finally, if the connection within A to B is interleaved, but not the
 * connection within B to P, then the order will be A, P, B, resulting in
 * the list:
 * 
 *      A0.B0.P0
 *      A1.B0.P0
 *      A0.B0.P1
 *      A1.B0.P1
 *      A0.B1.P0
 *      A1.B1.P0
 *      A0.B1.P1
 *      A1.B1.P1
 * 
 * A Range is a contiguous subset of one of the above lists, given by
 * a start offset and a width.
 * 
 * The head and tail functions split such a range.
 * 
 * This class and subclasses are designed to be immutable.
 * Modifications always return a new Range.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
*/
public class Range<T extends NamedInstance<?>> implements Comparable<Range<?>> {
    
    /**
     * Create a new range representing the full width of the specified instance
     * with no interleaving. The instances will be a list with the specified instance
     * first, its parent next, and on up the hierarchy until the depth 1 parent (the
     * top-level reactor is not included because it can never be a bank).
     * @param instance The instance.
     * @param connection The connection establishing this range or null if not unique.
     */
    public Range(
            T instance,
            Connection connection
    ) {
        this(instance, 0, 0, connection);
    }

    /**
     * Create a new range representing a range of the specified instance
     * with no interleaving. The instances will be a list with the specified instance
     * first, its parent next, and on up the hierarchy until the depth 1 parent (the
     * top-level reactor is not included because it can never be a bank).
     * @param instance The instance over which this is a range (port, reaction, etc.)
     * @param start The starting index for the range.
     * @param width The width of the range or 0 to specify the maximum possible width.
     * @param connection The connection establishing this range or null if not unique.
     */
    public Range(
            T instance,
            int start,
            int width,
            Connection connection
    ) {
        this.instance = instance;
        int totalWidth = instance.width;
        NamedInstance<?> parent = instance.parent;
        while (parent.depth > 0) {
            totalWidth *= parent.width;
            parent = parent.parent;
        }
        this.start = start;
        if (width > 0 && width + start < totalWidth) {
            this.width = width;
        } else {
            this.width = totalWidth - start;
        }
        this.totalWidth = totalWidth;
        this.connection = connection;
    }
    
    //////////////////////////////////////////////////////////
    //// Public variables
    
    /** The connection establishing this range or null if not unique. */
    public final Connection connection;

    /** The instance that this is a range of. */
    public final T instance;
    
    /** The start offset of this range. */
    public final int start;
    
    /** The total width of the list of instances. */
    public final int totalWidth;
    
    /** The width of this range. */
    public final int width;
    
    //////////////////////////////////////////////////////////
    //// Public methods

    /**
     * Compare ranges by first comparing their start offset, and then,
     * if these are equal, comparing their widths. This comparison is
     * meaningful only for ranges that have the same instances.
     */
    @Override
    public int compareTo(Range<?> o) {
        if (start < o.start) {
            return -1;
        } else if (start == o.start) {
            if (width < o.width) {
                return -1;
            } else if (width == o.width) {
                return 0;
            } else {
                return 1;
            }
        } else {
            return 1;
        }
    }
    
    /**
     * Return a new Range that is identical to this range but
     * with width reduced to the specified width.
     * If the new width is greater than or equal to the width
     * of this range, then return this range.
     * If the newWidth is 0 or negative, return null.
     * @param newWidth The new width.
     */
    public Range<T> head(int newWidth) {
        if (newWidth >= width) return this;
        if (newWidth <= 0) return null;
        return new Range<T>(instance, start, newWidth, connection);
    }
    
    /**
     * Return a list containing the instance for this range and all of its
     * parents, not including the top level reactor, in the order in which
     * their banks and multiport channels should be iterated.
     * For each depth at which the connection is interleaved, that parent
     * will appear before this instance in depth order (shallower to deeper).
     * For each depth at which the connection is not interleaved, that parent
     * will appear after this instance in reverse depth order (deeper to
     * shallower).
     */
    public List<NamedInstance<?>> iterationOrder() {
        LinkedList<NamedInstance<?>> result = new LinkedList<NamedInstance<?>>();
        result.add(instance);
        ReactorInstance parent = instance.parent;
        while (parent.depth > 0) {
            if (_interleaved.contains(parent)) {
                // Put the parent at the head of the list.
                result.add(0, parent);
            } else {
                result.add(parent);
            }
        }
        return result;
    }

    /**
     * Return a new range that represents the leftover elements
     * starting at the specified offset. If the offset is greater
     * than or equal to the width, then this returns null.
     * If this offset is 0 then this returns this range unmodified.
     * @param offset The number of elements to consume. 
     */
    public Range<T> tail(int offset) {
        if (offset == 0) return this;
        if (offset >= width) return null;
        return new Range<T>(instance, start + offset, width - offset, connection);
    }
    
    /**
     * Toggle the interleaved status of the specified reactor, which is assumed
     * to be a parent of the instance of this range.
     * If it was previously interleaved, make it not interleaved
     * and vice versa.  This returns a new Range.
     * @param reactor The parent reactor at which to toggle interleaving.
     */
    public Range<T> toggleInterleaved(ReactorInstance reactor) {
        Set<ReactorInstance> newInterleaved = new HashSet<ReactorInstance>(_interleaved);
        if (newInterleaved.contains(reactor)) {
            newInterleaved.remove(reactor);
        } else {
            newInterleaved.add(reactor);
        }
        Range<T> result = new Range<T>(instance, start, width, connection);
        result._interleaved = newInterleaved;
        return result;
    }
        
    @Override
    public String toString() {
        return instance.getFullName() + "(" + start + "," + width + ")";
    }

    //////////////////////////////////////////////////////////
    //// Public inner classes
    
    public static class Port extends Range<PortInstance> {
        public Port(PortInstance instance) {
            super(instance, null);
        }
        public Port(PortInstance instance, Connection connection) {
            super(instance, connection);
        }
        public Port(PortInstance instance, int start, int width, Connection connection) {
            super(instance, start, width, connection);
        }
    }

    //////////////////////////////////////////////////////////
    //// Protected variables
    
    /** Record of which levels are interleaved. */
    Set<ReactorInstance> _interleaved = new HashSet<ReactorInstance>();
}
