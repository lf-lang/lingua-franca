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
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

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
     * @param instance The Instance statement in the AST.
     * @param parent The parent.
     */
    public PortInstance(Port definition, ReactorInstance parent) {
        this(definition, parent, null);
    }

    /**
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param instance The Instance statement in the AST.
     * @param parent The parent.
     * @param errorReporter An error reporter, or null to throw exceptions.
     */
    public PortInstance(Port definition, ReactorInstance parent, ErrorReporter errorReporter) {
        super(definition, parent);
        
        if (parent == null) {
            throw new NullPointerException("Cannot create a PortInstance with no parent.");
        }
        
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
                            // This could throw NumberFormatException
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
    //// Public methods

    /**
     * Return a list of ranges of this port, where each range sends
     * to a list of destination ports that receive data from the range of
     * this port. Each destination port is annotated with the channel
     * range on which it receives data.
     * The ports listed are only ports that are sources for reactions,
     * not relay ports that the data may go through on the way.
     * 
     * If this port itself has dependent reactions,
     * that this port will be included as a destination in all items
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
    public List<SendingChannelRange> eventualDestinations() {
        if (eventualDestinationRanges != null) {
            return eventualDestinationRanges;
        }

        // Getting the destinations is more complex than getting the sources
        // because of multicast, where there is more than one connection statement
        // for a source of data. The strategy we follow here is to first get all
        // the ports that this port sends to. Then use eventualSource to get
        // the ranges of this port that send to each destination.
        PriorityQueue<SendingChannelRange> result = new PriorityQueue<SendingChannelRange>();

        Set<PortInstance> destinationPorts = null;
        if (isOutput()) {
            // For an output, obtain the destination ports from the parent
            // of the port's parent.
            ReactorInstance container = parent.getParent();
            // If the port's parent has no parent, then there are no destinations.
            if (parent == null) {
                return new LinkedList<SendingChannelRange>();
            }
            
            destinationPorts = container.transitiveClosure(this);
        } else {
            // For an input, obtain the destination ports from the parent of this port.
            // The port will be included in the returned set because it is an input.
            destinationPorts = parent.transitiveClosure(this);
        }
        
        // If this port has dependent reactions, then add an entry for this port.
        if (dependentReactions.size() > 0) {
            SendingChannelRange thisPort = new SendingChannelRange(0, width);
            thisPort.destinations.add(new PortChannelRange(0, width));
            result.add(thisPort);
        }

        for (PortInstance destinationPort: destinationPorts) {
            // If the destination port has no dependent reactions, skip it.
            // Also skip this port.
            if (destinationPort.dependentReactions.isEmpty() || destinationPort == this) {
                continue;
            }
            // Get the destination's source ranges and find the one(s) that match this port.
            int destinationChannel = 0;
            for (PortChannelRange source : destinationPort.eventualSources()) {
                if (source.getPortInstance() == this) {
                    // This destinationPort receives data from the channel range
                    // given by source of port. Add to the result list.
                    SendingChannelRange sendingRange = new SendingChannelRange(
                            source.startChannel, source.channelWidth
                    );
                    PortChannelRange receivingRange = destinationPort.newRange(
                            destinationChannel, source.channelWidth);
                    sendingRange.destinations.add(receivingRange);
                    result.add(sendingRange);
                }
                destinationChannel += source.channelWidth;
            }
        }
                
        // Now check for overlapping ranges, constructing a new result.
        eventualDestinationRanges = new ArrayList<SendingChannelRange>(result.size());
        SendingChannelRange candidate = result.poll();
        while (!result.isEmpty()) {
            SendingChannelRange next = result.poll();
            if (candidate.startChannel == next.startChannel) {
                // Ranges have the same starting point.
                if (candidate.channelWidth <= next.channelWidth) {
                    // Can use all of the channels. Import the destinations.
                    candidate.destinations.addAll(next.destinations);
                    if (candidate.channelWidth < next.channelWidth) {
                        // The next range has more channels connected to this sender.
                        next.startChannel += candidate.channelWidth;
                        next.channelWidth -= candidate.channelWidth;
                        result.add(next);
                    } // else we are done with next and can discard it.
                } else {
                    // candidate is wider than next.
                    // Use next as the new candidate and split candidate.
                    candidate.startChannel += next.channelWidth;
                    candidate.channelWidth -= next.channelWidth;
                    result.add(candidate);
                    candidate = next;
                }
            } else {
                // Because the result list is sorted, next starts at
                // a higher channel than candidate.
                if (candidate.startChannel + candidate.channelWidth <= next.startChannel) {
                    // Can use candidate as is and make next the new candidate.
                    eventualDestinationRanges.add(candidate);
                    candidate = next;
                } else {
                    // Ranges overlap. Have to split candidate.
                    SendingChannelRange candidateTail = new SendingChannelRange(
                            next.startChannel, 
                            candidate.channelWidth - (next.startChannel - candidate.startChannel)
                    );
                    candidateTail.destinations.addAll(candidate.destinations);
                    result.add(candidateTail);
                    candidate.channelWidth -= candidateTail.channelWidth;
                    // Put next back on the list.
                    result.add(next);
                }
            }
        }
        if (candidate != null) eventualDestinationRanges.add(candidate);

        return eventualDestinationRanges;
    }
    
    /**
     * Return a list of ports that send data to this port annotated
     * with the channel ranges of each source port. If this is not
     * a multiport, then the list will have only one item and the
     * channel range will contain only one channel.
     * Otherwise, it will have enough items so that the ranges
     * add up to the width of this multiport.
     * The ports listed are only ports that are written to by reactions,
     * not relay ports that the data may go through on the way.
     */
    public List<PortChannelRange> eventualSources() {
        if (eventualSourceRanges == null) {
            eventualSourceRanges = eventualSources(0, width);
        }
        return eventualSourceRanges;
    }

    /** 
     * Return the list of downstream ports that are connected to this port
     * or an empty list if there are none.
     */
    public List<PortChannelRange> getDependentPorts() {
        return dependentPorts;
    }

    /** 
     * Return the list of upstream ports that are connected to this port,
     * or an empty set if there are none.
     * For an ordinary port, this list will have length 0 or 1.
     * For a multiport, it can have a larger size.
     */
    public List<PortChannelRange> getDependsOnPorts() {
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
        List<SendingChannelRange> sourceChannelRanges = eventualDestinations();
        int result = 0;
        for (SendingChannelRange ranges : sourceChannelRanges) {
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
     * Return a list of ports that send data to the specified channels of
     * this port. The ports returned are annotated  with the channel
     * ranges of each source port.
     * The ports listed are only ports that are written to by reactions,
     * not relay ports that the data may go through on the way.
     * @param startRange The channel index for the start of the range of interest.
     * @param rangeWidth The number of channels to find sources for.
     */
    protected List<PortChannelRange> eventualSources(int startRange, int rangeWidth) {
        List<PortChannelRange> result = null;
        if (!isMultiport) {
            result = new ArrayList<PortChannelRange>(1);
        } else {
            result = new ArrayList<PortChannelRange>();
        }
        int channelsToSkip = startRange;
        int channelsProvided = 0;
        for (PortChannelRange sourceRange : dependsOnPorts) {
            // sourceRange.channelWidth is the number of channels this source has to offer.
            if (sourceRange.channelWidth <= channelsToSkip) {
                // No useful channels in this port. Skip it.
                channelsToSkip -= sourceRange.channelWidth;
                continue;
            }
            // If we get here, the source can provide some channels. How many?
            int srcStart = channelsToSkip;
            int srcWidth = sourceRange.channelWidth - srcStart; // Candidate width if we can use them all.
            if (channelsProvided + srcWidth > rangeWidth) {
                // Can't use all the source channels.
                srcWidth = (channelsProvided + srcWidth) - rangeWidth;
            }
            PortInstance src = sourceRange.getPortInstance();
            // If this source depends on reactions, then include it in the result.
            // Otherwise, keep looking upstream from it.
            if (src.dependsOnReactions.isEmpty()) {
                // Keep looking.
                result.addAll(src.eventualSources(srcStart, srcWidth));
            } else {
                result.add(src.newRange(srcStart, srcWidth));
            }
            channelsProvided += srcWidth;
            // No need to skip any more channels.
            channelsToSkip = 0;
            if (channelsProvided >= rangeWidth) {
                // Done.
                break;
            }
        }
        return result;
    }

    /**
     * Create a SendingChannelRange representing a subset of the channels of this port.
     * @param startChannel The lower end of the channel range.
     * @param channelWidth The width of the range.
     * @return A new instance of PortChannelRange.
     */
    protected SendingChannelRange newDestinationRange(int startChannel, int channelWidth) {
        return new SendingChannelRange(startChannel, channelWidth);
    }

    /**
     * Create a PortChannelRange representing a subset of the channels of this port.
     * @param startChannel The lower end of the channel range.
     * @param channelWidth The width of the range.
     * @return A new instance of PortChannelRange.
     */
    protected PortChannelRange newRange(int startChannel, int channelWidth) {
        return new PortChannelRange(startChannel, channelWidth);
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
    List<PortChannelRange> dependentPorts = new ArrayList<PortChannelRange>();

    /** 
     * Upstream ports that are connected directly to this port, if there are any.
     * For an ordinary port, this set will have size 0 or 1.
     * For a multiport, it can have a larger size.
     * This initially has capacity 1 because that is by far the most common case.
     */
    List<PortChannelRange> dependsOnPorts = new ArrayList<PortChannelRange>(1);
    
    /** Indicator of whether this is a multiport. */
    boolean isMultiport = false;
    
    /** 
     * The width of this port instance.
     * For an ordinary port, this is 1.
     * For a multiport, it may be larger than 1.
     */
    int width = 1;

    //////////////////////////////////////////////////////
    //// Private fields.
    
    /** Cached list of destination ports with channel ranges. */
    private List<SendingChannelRange> eventualDestinationRanges;

    /** Cached list of source ports with channel ranges. */
    private List<PortChannelRange> eventualSourceRanges;

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
    public class SendingChannelRange extends PortChannelRange {
        
        public SendingChannelRange(int startChannel, int channelWidth) {
            super(startChannel, channelWidth);
            
            if (PortInstance.this.isMultiport) {
                destinations = new ArrayList<PortChannelRange>();
            } else {
                destinations = new ArrayList<PortChannelRange>(1);
            }
        }

        public int getNumberOfDestinationReactors() {
            if (_numberOfDestinationReactors < 0) {
                // Has not been calculate before. Calculate now.
                Set<ReactorInstance> destinations = new HashSet<ReactorInstance>();
                for (PortChannelRange destination : this.destinations) {
                    destinations.add(destination.getPortInstance().getParent());
                }
                _numberOfDestinationReactors = destinations.size();
            }
            return _numberOfDestinationReactors;
        }

        public List<PortChannelRange> destinations;
        private int _numberOfDestinationReactors = -1; // Never access this directly.
    }
    
    /**
     * Class representing a range of channels of the enclosing port instance.
     * If the enclosing port instance is not a multiport, this range will
     * be (0,1).
     */
    public class PortChannelRange implements Comparable<PortChannelRange> {
        public PortChannelRange(int startChannel, int channelWidth) {
            if (startChannel < 0 || startChannel >= width || channelWidth < 0 || startChannel + channelWidth > width) {
                throw new RuntimeException("Invalid range of port channels.");
            }
            this.startChannel = startChannel;
            this.channelWidth = channelWidth;
        }
        public int startChannel;
        public int channelWidth;
        public PortInstance getPortInstance() {
            return PortInstance.this;
        }
        /**
         * Compare the ranges by just comparing their startChannel index.
         */
        @Override
        public int compareTo(PortChannelRange o) {
            if (startChannel < o.startChannel) {
                return -1;
            } else if (startChannel == o.startChannel) {
                return 0;
            } else {
                return 1;
            }
        }
    }
}
