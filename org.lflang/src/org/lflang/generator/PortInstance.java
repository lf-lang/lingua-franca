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
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.lflang.ErrorReporter;
import org.lflang.lf.Connection;
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
                for (Range sourceRange : eventualSourceRanges) {
                    sourceRange.getPort().clearCaches();
                }
            }
            if (eventualDestinationRanges != null) {
                for (SendRange sendRange : eventualDestinationRanges) {
                    for (Range destinationRange : sendRange.destinations) {
                        destinationRange.getPort().clearCaches();
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
        
        // Getting the destinations is more complex than getting the sources
        // because of multicast, where there is more than one connection statement
        // for a source of data. The strategy we follow here is to first get all
        // the ports that this port eventually sends to. Then, if needed, split
        // the resulting ranges so that each source range width matches all the
        // destination range widths.  We make two passes. First, we build
        // a queue of ranges that may overlap, then we split those ranges
        // and consolidate their destinations.
        PriorityQueue<SendRange> result = new PriorityQueue<SendRange>();
        
        // If this port has dependent reactions, then add it to the result.
        if (!dependentReactions.isEmpty()) {
            // This will be the final result if there are no connections.
            // The width depends on whether this is an input port or an output port.
            // For an input port (of a contained reactor), the width is just the width
            // of the port. For an output port, that width is multiplied by the the bank width.
            if (isInput()) {
                SendRange candidate = new SendRange(0, 0, width, false, null);
                candidate.destinations.add(newRange(0, 0, width, false, null));
                result.add(candidate);
            } else {
                // For an output port, the entire bank is treated as a send range
                // because every channel can carry different data.
                SendRange candidate = new SendRange(0, 0, width * parent.width(), false, null);
                candidate.destinations.add(newRange(0, 0, width * parent.width(), false, null));
                result.add(candidate);            }
        }
        
        // Next, look at downstream ports.
        int sourceWidthCovered = 0;
        int totalWidth = width * parent.width();
        
        Iterator<Range> destinations = dependentPorts.iterator();
        while(destinations.hasNext()) {
            Range dst = destinations.next();
            
            // Recursively get the send ranges of that destination port.
            Iterator<SendRange> dstSendIterator = dst.getPort().eventualDestinations().iterator();
            
            if (dstSendIterator.hasNext()) {
                SendRange dstSend = dstSendIterator.next();
                while (true) {
                    if (dstSend.getTotalWidth() <= totalWidth - sourceWidthCovered) {
                        // Destination range fits in current multicast iteration.
                        result.add(dstSend);
                        if (!dstSendIterator.hasNext()) break;
                        dstSend = dstSendIterator.next();
                        sourceWidthCovered += dstSend.getTotalWidth();
                    } else {
                        // Destination range spills over into the next multicast iteration.
                        // Need to split the destination range.
                        SendRange residualDst = (SendRange)dstSend.tail(totalWidth - sourceWidthCovered);
                        dstSend = dstSend.truncate(totalWidth - sourceWidthCovered);
                        result.add(dstSend);
                        dstSend = residualDst;
                        sourceWidthCovered = 0;
                    }
                }
            }
        }
                        
        // Now check for overlapping ranges, constructing a new result.
        eventualDestinationRanges = new ArrayList<SendRange>(result.size());
        SendRange candidate = result.poll();
        SendRange next = result.poll();
        while (candidate != null) {
            if (next == null) {
                // No more candidates.  We are done.
                eventualDestinationRanges.add(candidate);
                break;
            }
            if (candidate.startChannel == next.startChannel) {
                // Ranges have the same starting point. Need to merge them.
                if (candidate.getTotalWidth() <= next.getTotalWidth()) {
                    // Can use all of the channels of candidate.
                    // Import the destinations of next and split it.
                    candidate.destinations.addAll(next.destinations);
                    if (candidate.getTotalWidth() < next.getTotalWidth()) {
                        // The next range has more channels connected to this sender.
                        next = next.truncate(candidate.getTotalWidth());
                        // Truncate the destinations just imported.
                        candidate = candidate.truncate(candidate.getTotalWidth());
                    } else {
                        // We are done with next and can discard it.
                        next = result.poll();
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
                if (candidate.startChannel + candidate.getTotalWidth() <= next.startChannel) {
                    // Can use candidate as is and make next the new candidate.
                    eventualDestinationRanges.add(candidate);
                    candidate = next;
                    next = result.poll();
                } else {
                    // Ranges overlap. Have to split candidate.
                    SendRange candidateTail = (SendRange)candidate.tail(next.startChannel);
                    candidate = candidate.truncate(next.startChannel);
                    result.add(candidate);
                    candidate = candidateTail;
                }
            }
        }
        return eventualDestinationRanges;
    }
    
    /**
     * Return a list of ports that send data to this port annotated with the channel
     * and bank ranges of each source port. If this port is directly written to by
     * one more more reactions, then it is its own eventual source an only this port
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
    public List<Range> eventualSources() {
        if (eventualSourceRanges == null) {
            // Cached result has not been created.
            eventualSourceRanges = new ArrayList<Range>();
            
            if (!dependsOnReactions.isEmpty()) {
                eventualSourceRanges.add(newRange(0, 0, width * parent.width(), false, null));
            } else {
                for (Range sourceRange : dependsOnPorts) {
                    eventualSourceRanges.addAll(sourceRange.getPort().eventualSources());
                }
            }
        }
        return eventualSourceRanges;
    }

    /** 
     * Return the list of downstream ports that are connected to this port
     * or an empty list if there are none.
     */
    public List<Range> getDependentPorts() {
        return dependentPorts;
    }

    /** 
     * Return the list of upstream ports that are connected to this port,
     * or an empty set if there are none.
     * For an ordinary port, this list will have length 0 or 1.
     * For a multiport, it can have a larger size.
     */
    public List<Range> getDependsOnPorts() {
        return dependsOnPorts;
    }
    
    /**
     * Return the width of this port, which in this base class is 1.
     */
    public int getWidth() {
        return width;
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
    //// Protected methods.
    
    /**
     * Create a Range representing a subset of the channels of this port.
     * @param startChannel The lower end of the channel range.
     * @param startBank The lower end of the bank range (or 0 for non-banks).
     * @param width The width of the range (total number of connections).
     * @param interleaved Marker whether bank and multiport iteration should be reversed.
     * @param connection The connection associated with this range or null if none.
     * @return A new instance of Range.
     */
    protected Range newRange(
            int startChannel, int startBank, int width, boolean interleaved, Connection connection
    ) {
        return new Range(startChannel, startBank, width, interleaved, connection);
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
    List<Range> dependentPorts = new ArrayList<Range>();

    /** 
     * Upstream ports that are connected directly to this port, if there are any.
     * For an ordinary port, this set will have size 0 or 1.
     * For a multiport, it can have a larger size.
     * This initially has capacity 1 because that is by far the most common case.
     */
    List<Range> dependsOnPorts = new ArrayList<Range>(1);
    
    /** Indicator of whether this is a multiport. */
    boolean isMultiport = false;
    
    /** 
     * The width of this port instance.
     * For an ordinary port, this is 1.
     * For a multiport, it may be larger than 1.
     */
    int width = 1;

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
    private List<Range> eventualSourceRanges;
    
    /** Indicator that we are clearing the caches. */
    private boolean clearingCaches = false;

    //////////////////////////////////////////////////////
    //// Inner classes.

    /**
     * Class representing a range of channels of this port that broadcast to some
     * number of destination ports' channels. All ranges have the same
     * width, but not necessarily the same start index.
     * This class extends its base class with a list destination channel ranges,
     * all of which have the same width as this channel range.
     * It also includes a field representing the number of destination
     * reactors.
     */
    public class SendRange extends Range {
        
        public SendRange(int startChannel, int startBank, int totalWidth, boolean interleaved, Connection connection) {
            super(startChannel, startBank, totalWidth, interleaved, connection);
        }

        public int getNumberOfDestinationReactors() {
            if (_numberOfDestinationReactors < 0) {
                // Has not been calculate before. Calculate now.
                Set<ReactorInstance> destinations = new HashSet<ReactorInstance>();
                for (Range destination : this.destinations) {
                    destinations.add(destination.getPort().getParent());
                }
                _numberOfDestinationReactors = destinations.size();
            }
            return _numberOfDestinationReactors;
        }

        public List<Range> destinations = new ArrayList<Range>();

        /**
         * Override the base class to return a SendRange where
         * each of the destinations is the tail of the original destinations.
         * @param offset The number of channels to consume. 
         */
        @Override
        public Range tail(int offset) {
            SendRange result = (SendRange)super.tail(offset);
            for (Range destination : destinations) {
                result.destinations.add(destination.tail(offset));
            }
            return result;
        }
        
        /**
         * Override the base class to return a SendRange rather than Range.
         */
        @Override
        protected Range newRange(
                int startChannel, int startBank, int totalWidth, boolean interleaved, Connection connection
        ) {
            return new SendRange(
                    startChannel,
                    startBank,
                    totalWidth, 
                    interleaved,
                    connection
            );
        }
        
        /**
         * Override the base class to also truncate the destinations.
         * @param newWidth The new width.
         */
        @Override
        protected SendRange truncate(int newWidth) {
            SendRange result = (SendRange)super.truncate(newWidth);
            for (Range destination : destinations) {
                result.destinations.add(destination.truncate(newWidth));
            }
            return result;
        }

        private int _numberOfDestinationReactors = -1; // Never access this directly.
    }
    
    /**
     * Class representing a range of channels of the enclosing port instance.
     * If the enclosing port instance is not a multiport, this range will
     * be (0,1).
     */
    public class Range implements Comparable<Range> {
        
        public Range(
                int startChannel,
                int startBank,
                int totalWidth,
                boolean interleaved,
                Connection connection
        ) {
            int widthLimit = width * parent.width();
            // Some targets determine widths at runtime, in which case a
            // width of 0 is reported here. Tolerate that.
            if (totalWidth > 0
                    && (startChannel < 0 || startChannel >= widthLimit 
                    || totalWidth < 0 || startChannel + totalWidth > widthLimit)) {
                throw new RuntimeException("Invalid range of port channels.");
            }
            this.startChannel = startChannel;
            this.startBank = startBank;
            this.totalWidth = totalWidth;
            this.interleaved = interleaved;
            this.connection = connection;
        }
        public final int startChannel;
        public final int startBank;
        public final boolean interleaved;
        public final Connection connection;
        public PortInstance getPort() {
            return PortInstance.this;
        }
        
        /**
         * Compare the ranges by just comparing their startChannel index.
         */
        @Override
        public int compareTo(Range o) {
            if (startChannel < o.startChannel) {
                return -1;
            } else if (startChannel == o.startChannel) {
                return 0;
            } else {
                return 1;
            }
        }
        
        /**
         * Return the total width of the range.
         */
        public int getTotalWidth() {
            return totalWidth;
        }

        /**
         * Return a new range that represents the leftover channels
         * starting at the specified offset. Depending on
         * whether this range is interleaved, this will consume from
         * multiport channels first (if not interleaved) or banks first
         * (if interleaved). The offset is required to be less
         * than the total width or an exception will be thrown.
         * @param offset The number of channels to consume. 
         */
        public Range tail(int offset) {
            if (offset >= totalWidth) {
                throw new RuntimeException("Insufficient channels in range.");
            }
            int channelWidth = PortInstance.this.width;
            int bankWidth = PortInstance.this.parent.width();
            
            int banksToConsume, channelsToConsume;
            if (interleaved) {
                banksToConsume = offset / bankWidth;
                channelsToConsume = offset % bankWidth;
            } else {
                banksToConsume = offset / channelWidth;
                channelsToConsume = offset % channelWidth;
            }
            return newRange(
                    startChannel + channelsToConsume,
                    startBank + banksToConsume,
                    totalWidth - offset, 
                    interleaved,
                    connection
            );
        }
        
        @Override
        public String toString() {
            return String.format(
                    "%s(channel %d, bank %d, width %d)",
                    PortInstance.this.getFullName(),
                    startChannel,
                    startBank,
                    totalWidth
            );
        }
        
        /**
         * Return a new range that is identical to this range but
         * with a reduced total width to the specified width.
         * @param newWidth The new width.
         */
        protected Range truncate(int newWidth) {
            return newRange(startChannel, startBank, newWidth, interleaved, connection);
        }
        
        /**
         * Create a new Range with the specified parameters.
         */
        protected Range newRange(
                int startChannel, int startBank, int totalWidth, boolean interleaved, Connection connection
        ) {
            return new Range(
                    startChannel,
                    startBank,
                    totalWidth, 
                    interleaved,
                    connection
            );
        }
        
        private int totalWidth; // Allow modification.
    }
}
