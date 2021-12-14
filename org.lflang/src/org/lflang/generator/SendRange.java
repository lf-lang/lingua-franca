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

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Class representing a range of a port that sources data
 * together with a list of destination ranges of other ports that should all
 * receive the same data sent in this range.
 * All ranges in the destinations list have the same width as this range,
 * but not necessarily the same start offsets.
 * This class also includes a field representing the number of destination
 * reactors.
 * 
 * This class and subclasses are designed to be immutable.
 * Modifications always return a new Range.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
*/
public class SendRange extends Range.Port {
    
    /**
     * Create a new send range.
     * @param instance The instance over which this is a range of.
     * @param start The starting index.
     * @param width The width.
     */
    public SendRange(
            PortInstance instance,
            int start,
            int width
    ) {
        super(instance, start, width, null);
    }

    //////////////////////////////////////////////////////////
    //// Public variables

    /** The list of destination ranges to which this broadcasts. */
    public final List<Range<PortInstance>> destinations = new ArrayList<Range<PortInstance>>();

    //////////////////////////////////////////////////////////
    //// Public methods

    /**
     * Return the total number of destination reactors. Specifically, this
     * is the number of distinct reactors that have one or more ports
     * with dependent reactions in the reactor.
     */
    public int getNumberOfDestinationReactors() {
        if (_numberOfDestinationReactors < 0) {
            // Has not been calculated before. Calculate now.
            Set<ReactorInstance> result = new HashSet<ReactorInstance>();
            for (Range<PortInstance> destination : this.destinations) {
                for (NamedInstance<?> instance : destination.iterationOrder()) {
                    if (instance instanceof PortInstance 
                            && !((PortInstance)instance).dependentReactions.isEmpty()) {
                        result.add(instance.getParent());
                    }
                }
            }
            _numberOfDestinationReactors = result.size();
        }
        return _numberOfDestinationReactors;
    }

    /**
     * Return a new SendRange that is identical to this range but
     * with width reduced to the specified width.
     * If the new width is greater than or equal to the width
     * of this range, then return this range.
     * If the newWidth is 0 or negative, return null.
     * This overrides the base class to also apply head()
     * to the destination list.
     * @param newWidth The new width.
     */
    @Override
    public SendRange head(int newWidth) {
        // NOTE: Cannot use the superclass because it returns a Range, not a SendRange.
        if (newWidth >= width) return this;
        if (newWidth <= 0) return null;

        SendRange result = new SendRange(instance, start, newWidth);
        
        for (Range<PortInstance> destination : destinations) {
            result.destinations.add(destination.head(newWidth));
        }
        return result;
    }

    /**
     * Return a new SendRange that is like this one, but
     * converted to the specified upstream range. The returned
     * SendRange inherits the destinations of this range.
     * 
     * Normally, the total width of the specified range is
     * the same as that of this range, but if it is not,
     * then the minimum of the two widths is returned.
     * 
     * @param srcRange A new source range.
     */
    protected SendRange newSendRange(Range<PortInstance> srcRange) {
        SendRange reference = this;
        if (srcRange.totalWidth > totalWidth) {
            srcRange = srcRange.head(totalWidth);
        } else if (srcRange.totalWidth < totalWidth) {
            reference = head(srcRange.totalWidth);
        }
        SendRange result = new SendRange(srcRange.instance, srcRange.start, srcRange.width);
        
        result.destinations.addAll(reference.destinations);

        return result;
    }

    /**
     * Return a new SendRange that represents the leftover elements
     * starting at the specified offset. If the offset is greater
     * than or equal to the width, then this returns null.
     * If this offset is 0 then this returns this range unmodified.
     * This overrides the base class to also apply tail()
     * to the destination list.
     * @param offset The number of elements to consume. 
     */
    @Override
    public SendRange tail(int offset) {
        if (offset == 0) return this;
        if (offset >= width) return null;
        SendRange result = new SendRange(instance, start + offset, width - offset);

        for (Range<PortInstance> destination : destinations) {
            result.destinations.add(destination.tail(offset));
        }
        return result;
    }

    //////////////////////////////////////////////////////////
    //// Private variables

    private int _numberOfDestinationReactors = -1; // Never access this directly.
}
