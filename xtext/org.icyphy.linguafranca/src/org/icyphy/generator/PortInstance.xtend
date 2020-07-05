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
package org.icyphy.generator

import java.util.HashSet
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Variable

/** Representation of a runtime instance of a port.
 *  
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class PortInstance extends TriggerInstance<Variable> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Port definition, ReactorInstance parent) {
        this(definition, parent, -1)
    }
    
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it and
     *  the specified index in a multiport array.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     *  @param index The index, or -1 to specify that this is not
     *   in a multiport.
     */
    new(Port definition, ReactorInstance parent, int index) {
        super(definition, parent)
        
        if (parent === null) {
            throw new Exception('Cannot create a PortInstance with no parent.')
        }
        this.index = index
    }
     
    /////////////////////////////////////////////
    //// Public Fields

    /** Set of port instances that receive messages from this port. */
    public HashSet<PortInstance> dependentPorts = new HashSet<PortInstance>();
        
    /** Port that sends messages to this port, if there is one. */
    public PortInstance dependsOnPort = null;
        
    /////////////////////////////////////////////
    //// Public Methods
    
    /** Override the base class to append [index] if this port
     *  is a multiport instance.
     *  @return The full name of this instance.
     */
    override String getFullName() {
        var result = super.getFullName()
        if (this.index >= 0) {
            result += "[" + this.index + "]"
        }
        result
    }
    
    /**
     * Return the index of this port in a multiport array or -1 if
     * this port is not in a multiport array. 
     * @return The index in a multiport array.
     */
    def multiportIndex() {
        return this.index
    }
    
    /** Return true if the port is an input. */
    def isInput() {
        definition instanceof Input
    }

    /** Return true if the port is an output. */
    def isOutput() {
        definition instanceof Output
    }
    
    /** Return the number of destination reactors for this port instance. */
    def numDestinationReactors() {
        // Count the number of destination reactors that receive data from
        // this output port. Do this by building a set of the containers
        // of all dependent ports and reactions. The dependentReactions
        // includes reactions of the container that listen to this port.
        val destinationReactors = new HashSet<ReactorInstance>()
        for (destinationPort : this.dependentPorts) {
            destinationReactors.add(destinationPort.parent)
        }
        for (destinationReaction : this.dependentReactions) {
            destinationReactors.add(destinationReaction.parent)
        }
        return destinationReactors.size
    }
    
    /** Return a descriptive string. */
    override toString() {
        "PortInstance " + getFullName
    }

    /////////////////////////////////////////////
    //// Protected Fields

    /** 
     * The index in a multiport array or -1 if this port is not in
     * a multiport array.
     */
    protected int index
}