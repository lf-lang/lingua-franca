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


/**
 * Class representing a range of channels of a port instance.
 * If the enclosing port instance is not a multiport, this range will
 * be (0,1).
 * 
 * @author Edward A. Lee
 */
public class ChannelRange implements Comparable<ChannelRange> {
    public ChannelRange(PortInstance port, int startChannel, int channelWidth) {
        // Some targets determine widths at runtime, in which case a
        // width of 0 is reported here. Tolerate that.
        if (channelWidth != 0
                && (startChannel < 0 || startChannel >= port.width 
                || channelWidth < 0 || startChannel + channelWidth > port.width)) {
            throw new RuntimeException("Invalid range of port channels.");
        }
        this.port = port;
        this.startChannel = startChannel;
        this.channelWidth = channelWidth;
    }

    //////////////////////////////////////////////////////
    //// Public methods

    /** The port that contains these channels. */
    public final PortInstance port;
    
    /** The starting channel index (starting at 0). */
    public final int startChannel;
    
    /** The width of the range (0 means unknown). */
    public final int channelWidth;

    //////////////////////////////////////////////////////
    //// Public methods

    /**
     * Compare the ranges by just comparing their startChannel index.
     */
    @Override
    public int compareTo(ChannelRange o) {
        if (startChannel < o.startChannel) {
            return -1;
        } else if (startChannel == o.startChannel) {
            return 0;
        } else {
            return 1;
        }
    }
}
