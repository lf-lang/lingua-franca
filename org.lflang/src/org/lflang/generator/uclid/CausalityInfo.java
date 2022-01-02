/*************
Copyright (c) 2021, The University of California at Berkeley.

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

package org.lflang;

import org.lflang.generator.PortInstance;
import org.lflang.generator.TriggerInstance;

/**
 * Connectivity info class
 * 
 * @author Shaokai Lin {@literal <shaokai@eecs.berkeley.edu>}
 */
public class CausalityInfo {
    
    public String           type;
    public TriggerInstance  triggerInstance;
    public boolean          isPhysical;
    public long             delay;
    public PortInstance     upstreamPort;
    public PortInstance     downstreamPort;
    
    public CausalityInfo(
        String          _type,
        TriggerInstance _triggerInstance,
        boolean         _isPhysical, 
        long            _delay,
        PortInstance    _upstreamPort,
        PortInstance    _downstreamPort
    ){
        type            =   _type;
        triggerInstance =   _triggerInstance;
        isPhysical      =   _isPhysical;
        delay           =   _delay;
        upstreamPort    =   _upstreamPort;
        downstreamPort  =   _downstreamPort;
    }
}