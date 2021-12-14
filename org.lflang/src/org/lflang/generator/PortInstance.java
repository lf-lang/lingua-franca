/** A data structure for a port instance. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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
***************/
package org.lflang.generator;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;

import org.lflang.ErrorReporter;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;

/** 
 * Representation of a runtime instance of a port.
 * This may be a single port or a multiport.
 *  
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
public class PortInstance extends TriggerInstance<Port> {

    /**
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param definition The Instance statement in the AST.
     * @param parent The parent.
     */
    public PortInstance(Port definition, ReactorInstance parent) {
        this(definition, parent, null);
    }

    /**
     * Create a port instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param definition The Instance statement in the AST.
     * @param parent The parent.
     * @param errorReporter An error reporter, or null to throw exceptions.
     */
    public PortInstance(Port definition, ReactorInstance parent, ErrorReporter errorReporter) {
        super(definition, parent);
        
        if (parent == null) {
            throw new NullPointerException("Cannot create a PortInstance with no parent.");
        }
        
        setInitialWidth(errorReporter);
    }

    //////////////////////////////////////////////////////
    //// Public methods

    /**
     * Clear cached information about the connectivity of this port.
     * In particular, {@link #eventualDestinations()} and {@link #eventualSources()}
     * cache the lists they return. To force those methods to recompute
     * their lists, call this method. This method also clears the caches
     * of any ports that are listed as eventual destinations and sources.
     */
    public void clearCaches() {
        if (clearingCaches) return; // Prevent stack overflow.
        clearingCaches = true;
        try {
            if (eventualSourceRanges != null) {
                for (Range<PortInstance> sourceRange : eventualSourceRanges) {
                    sourceRange.instance.clearCaches();
                }
            }
            if (eventualDestinationRanges != null) {
                for (SendRange sendRange : eventualDestinationRanges) {
                    for (Range<PortInstance> destinationRange : sendRange.destinations) {
                        destinationRange.instance.clearCaches();
                    }
                }
            }
            eventualDestinationRanges = null;
            eventualSourceRanges = null;
        } finally {
            clearingCaches = false;
        }
    }

    /**
     * Return a list of ranges of this port, where each range sends
     * to a list of destination ports that receive data from the range of
     * this port. Each destination port is annotated with the channel
     * range on which it receives data.
     * The ports listed are only ports that are sources for reactions,
     * not relay ports that the data may go through on the way.
     * 
     * If this port itself has dependent reactions,
     * then this port will be included as a destination in all items
     * on the returned list.
     * 
     * Each item in the returned list has the following fields:
     * * startRange The starting channel index of this port.
     * * rangeWidth The number of channels sent to the destinations.
     * * destinations A list of port ranges for destination ports, each
     *   of which has the same width as rangeWidth.
     *   
     * Each item also has a method, getNumberOfDestinationReactors(),
     * that returns the total number of unique destination reactors for
     * its range. This is not necessarily the same as the number
     * of ports in its destinations field because some of the ports may 
     * share the same container reactor.
     */
    public List<SendRange> eventualDestinations() {
        if (eventualDestinationRanges != null) {
            return eventualDestinationRanges;
        }
        
        // Construct the full range for this port.
        Range<PortInstance> range = new Range.Port(this);
        eventualDestinationRanges = eventualDestinations(range);
        return eventualDestinationRanges;
    }
    
    /**
     * Given a Range, return a list of SendRange that describes
     * the eventual destinations of the given range.
     * The sum of the total widths of the send ranges on the returned list
     * will equal the total width of the specified range.
     * The returned list will be non-overlapping ranges in
     * the order in which they should be traversed (channels, then
     * banks if the specified range is not interleaved, or banks
     * then channels otherwise).  Each returned SendRange has a list
     * of destinations Range, each of which represents a port that
     * has dependent reactions. Intermediate ports with no dependent
     * reactions are not listed.
     * @param srcRange The source range.
     */
    public static List<SendRange> eventualDestinations(Range<PortInstance> srcRange) {

        // Getting the destinations is more complex than getting the sources
        // because of multicast, where there is more than one connection statement
        // for a source of data. The strategy we follow here is to first get all
        // the ports that this port eventually sends to. Then, if needed, split
        // the resulting ranges so that each source range width matches all the
        // destination range widths.  We make two passes. First, we build
        // a queue of ranges that may overlap, then we split those ranges
        // and consolidate their destinations.

        List<SendRange> result = new ArrayList<SendRange>();
        PriorityQueue<SendRange> queue = new PriorityQueue<SendRange>();
        PortInstance srcPort = srcRange.instance;
        
        // Start with, if this port has dependent reactions, then add it to 
        // every range of the result.
        if (!srcRange.instance.dependentReactions.isEmpty()) {
            // This will be the final result if there are no connections.
            SendRange candidate = new SendRange(
                    srcRange.instance,
                    srcRange.start, 
                    srcRange.width
            );
            candidate.destinations.add(srcRange);
            result.add(candidate);
        }

        // Start with ports that are downstream of the range.
        int srcWidthCovered = 0;
        int depWidthCovered = 0;
        Iterator<Range<PortInstance>> dependentPorts = srcPort.dependentPorts.iterator();
        if (dependentPorts.hasNext()) {
            Range<PortInstance> dep = dependentPorts.next();
            while(srcWidthCovered < srcRange.totalWidth) {
                if (srcRange.start >= depWidthCovered + dep.totalWidth) {
                    // Destination is fully before this range.
                    depWidthCovered += dep.totalWidth;
                    if (!dependentPorts.hasNext()) break; // This should be an error.
                    dep = dependentPorts.next();
                    continue;
                }
                if (depWidthCovered >= srcRange.start + srcRange.totalWidth) {
                    // Source range is covered. We are finished.
                    break;
                }
                // Dependent port overlaps the range of interest.
                // Get a new range that possibly subsets the target range.
                // Argument is guaranteed by above checks to be less than
                // dep.totalWidth, so the result will not be null.
                Range<PortInstance> subDep = dep.tail(depWidthCovered - srcRange.start);
                // The following argument is guaranteed to be greater than
                // depWidthCovered - srcRange.getStartOffset().
                subDep = subDep.head(srcRange.totalWidth);
                            
                // At this point, dep is the subrange of the dependent port of interest.
                // Recursively get the send ranges of that destination port.
                List<SendRange> dstSendRanges = eventualDestinations(subDep);
                
                // For each returned SendRange, convert it to a SendRange
                // for the srcRange port rather than the dep port.
                for (SendRange dstSend : dstSendRanges) {
                    // FIXME: Need to figure out interleaved level!
                    queue.add(dstSend.newSendRange(srcRange));
                }
                depWidthCovered += subDep.totalWidth;
                srcWidthCovered += subDep.totalWidth;
                if (dep.start + dep.totalWidth <= subDep.start + subDep.totalWidth) {
                    // dep range is exhausted. Get another one.
                    if (!dependentPorts.hasNext()) break; // This should be an error.
                    dep = dependentPorts.next();
                    depWidthCovered = 0;
                }
            }
        }

        // Now check for overlapping ranges, constructing a new result.
        SendRange candidate = queue.poll();
        SendRange next = queue.poll();
        while (candidate != null) {
            if (next == null) {
                // No more candidates.  We are done.
                result.add(candidate);
                break;
            }
            if (candidate.start == next.start) {
                // Ranges have the same starting point. Need to merge them.
                if (candidate.totalWidth <= next.totalWidth) {
                    // Can use all of the channels of candidate.
                    // Import the destinations of next and split it.
                    candidate.destinations.addAll(next.destinations);
                    if (candidate.totalWidth < next.totalWidth) {
                        // The next range has more channels connected to this sender.
                        next = (SendRange)next.tail(candidate.totalWidth);
                        // Truncate the destinations just imported.
                        candidate = candidate.head(candidate.totalWidth);
                    } else {
                        // We are done with next and can discard it.
                        next = queue.poll();
                    }
                } else {
                    // candidate is wider than next. Switch them and continue.
                    SendRange temp = candidate;
                    candidate = next;
                    next = temp;
                }
            } else {
                // Because the result list is sorted, next starts at
                // a higher channel than candidate.
                if (candidate.start + candidate.start <= next.start) {
                    // Can use candidate as is and make next the new candidate.
                    result.add(candidate);
                    candidate = next;
                    next = queue.poll();
                } else {
                    // Ranges overlap. Can use a truncated candidate and make its
                    // truncated version the new candidate.
                    result.add(candidate.head(next.start));
                    candidate = (SendRange)candidate.tail(next.start);
                }
            }
        }
        
        return result;
    }

    /**
     * Return a list of ports that send data to this port annotated with the channel
     * and bank ranges of each source port. If this port is directly written to by
     * one more more reactions, then it is its own eventual source and only this port
     * will be represented in the result.
     * 
     * If this is not a multiport and is not within a bank, then the list will have
     * only one item and the range will have a total width of one. Otherwise, it will
     * have enough items so that the range widths add up to the width of this
     * multiport multiplied by the total number of instances within containing banks.
     * 
     * The ports listed are only ports that are written to by reactions,
     * not relay ports that the data may go through on the way.
     */
    public List<Range<PortInstance>> eventualSources() {
        if (eventualSourceRanges == null) {
            // Cached result has not been created.
            eventualSourceRanges = new ArrayList<Range<PortInstance>>();
            
            if (!dependsOnReactions.isEmpty()) {
                eventualSourceRanges.add(new Range.Port(this));
            } else {
                for (Range<PortInstance> sourceRange : dependsOnPorts) {
                    // FIXME FIXME This needs to use the sourceRange!
                    eventualSourceRanges.addAll(sourceRange.instance.eventualSources());
                }
            }
        }
        return eventualSourceRanges;
    }

    /** 
     * Return the list of downstream ports that are connected to this port
     * or an empty list if there are none.
     */
    public List<Range<PortInstance>> getDependentPorts() {
        return dependentPorts;
    }

    /** 
     * Return the list of upstream ports that are connected to this port,
     * or an empty set if there are none.
     * For an ordinary port, this list will have length 0 or 1.
     * For a multiport, it can have a larger size.
     */
    public List<Range<PortInstance>> getDependsOnPorts() {
        return dependsOnPorts;
    }
    
    /** 
     * Return true if the port is an input.
     */
    public boolean isInput() {
        return (definition instanceof Input);
    }
    
    /**
     * Return true if this is a multiport.
     */
    public boolean isMultiport() {
        return isMultiport;
    }

    /** 
     * Return true if the port is an output.
     */
    public boolean isOutput() {
        return (definition instanceof Output);
    }
    
    /** 
     * Return the number of destination reactors for this port instance.
     * This can be used to initialize reference counting, but not for
     * multiport.  For multiports, the number of destinations can vary
     * by channel, and hence must be obtained from the ranges reported
     * by eventualDestinations();
     */
    public int numDestinationReactors() {
        List<SendRange> sourceChannelRanges = eventualDestinations();
        int result = 0;
        for (SendRange ranges : sourceChannelRanges) {
            result += ranges.getNumberOfDestinationReactors();
        }
        return result;
    }
    
    @Override
    public String toString() {
        return "PortInstance " + getFullName();
    }    

    //////////////////////////////////////////////////////
    //// Protected fields.

    /** 
     * Downstream ports that are connected directly to this port.
     * These are listed in the order they appear in connections.
     * If this port is input port, then the connections are those
     * in the parent reactor of this port (inside connections).
     * If the port is an output port, then the connections are those
     * in the parent's parent (outside connections).
     * The sum of the widths of the dependent ports is required to
     * be an integer multiple N of the width of this port (this is checked
     * by the validator). Each channel of this port will be broadcast
     * to N recipients.
     */
    List<Range<PortInstance>> dependentPorts = new ArrayList<Range<PortInstance>>();

    /** 
     * Upstream ports that are connected directly to this port, if there are any.
     * For an ordinary port, this set will have size 0 or 1.
     * For a multiport, it can have a larger size.
     * This initially has capacity 1 because that is by far the most common case.
     */
    List<Range<PortInstance>> dependsOnPorts = new ArrayList<Range<PortInstance>>(1);
    
    /** Indicator of whether this is a multiport. */
    boolean isMultiport = false;
    
    //////////////////////////////////////////////////////
    //// Private methods.
    
    /**
     * Set the initial multiport width, if this is a multiport, from the widthSpec
     * in the definition.
     * @param errorReporter For reporting errors.
     */
    private void setInitialWidth(ErrorReporter errorReporter) {
        // If this is a multiport, determine the width.
        WidthSpec widthSpec = definition.getWidthSpec();

        if (widthSpec != null) {
            if (widthSpec.isOfVariableLength()) {
                errorReporter.reportError(definition,
                        "Variable-width multiports not supported (yet): " + definition.getName());
            } else {
                isMultiport = true;
                
                // Determine the initial width, if possible.
                // The width may be given by a parameter or even sum of parameters.
                width = 0;
                for (WidthTerm term : widthSpec.getTerms()) {
                    Parameter parameter = term.getParameter();
                    if (parameter != null) {
                        Integer parameterValue = parent.initialIntParameterValue(parameter);
                        // Only a Literal is supported.
                        if (parameterValue != null) {
                            width += parameterValue;
                        } else {
                            errorReporter.reportWarning(definition,
                                "Width of a multiport cannot be determined. Assuming 1."
                            );
                            width += 1;
                        }
                    } else {
                        width += term.getWidth();
                    }
                }
            }
        }
    }

    //////////////////////////////////////////////////////
    //// Private fields.
    
    /** Cached list of destination ports with channel ranges. */
    private List<SendRange> eventualDestinationRanges;

    /** Cached list of source ports with channel ranges. */
    private List<Range<PortInstance>> eventualSourceRanges;
    
    /** Indicator that we are clearing the caches. */
    private boolean clearingCaches = false;
}
