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
package org.lflang.generator

import java.util.LinkedHashSet
import java.util.Set
import org.lflang.lf.Input
import org.lflang.lf.Output
import org.lflang.lf.Port
import org.lflang.util.CollectionUtil

/** Representation of a runtime instance of a port.
 *  
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class PortInstance extends TriggerInstance<Port> {

    /** Set of port instances that receive messages from this port. */
    Set<PortInstance> dependentPorts = Set.of();

    /** 
     * Ports that sends messages to this port, if there are any.
     * For an ordinary port, this set will have size 0 or 1.
     * For a multiport, it can have a larger size.
     */
    protected Set<PortInstance> dependsOnPorts = Set.of();
    
    /** 
     * The width of this port instance.
     * For an ordinary port, this is 1.
     * For a multiport, it may be larger than 1.
     */
    protected int width = 1

    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Port definition, ReactorInstance parent) {
        super(definition, parent)
        
        if (parent === null) {
            throw new NullPointerException('Cannot create a PortInstance with no parent.')
        }
    }

    /**
     * Return the set of ports that this port depends on.
     * For ordinary ports, there is at most one.
     * For multiports, there may be more than one.
     */
    def Set<PortInstance> getDependsOnPorts() {
        return this.dependsOnPorts;
    }

    def Set<PortInstance> dependentPorts() {
        return dependentPorts;
    }

    def void addDependentPort(PortInstance dependent) {
        this.dependentPorts = CollectionUtil.plus(dependentPorts, dependent);
    }
        
    /**
     * Return the width of this port, which in this base class is 1.
     */
    def getWidth() {
        width
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
        val destinationReactors = new LinkedHashSet<ReactorInstance>()
        for (destinationPort : this.dependentPorts) {
            destinationReactors.add(destinationPort.parent)
        }
        for (destinationReaction : this.dependentReactions) {
            destinationReactors.add(destinationReaction.parent)
        }
        return destinationReactors.size
    }
    
    override toString() {
        "PortInstance " + getFullName
    }
}
