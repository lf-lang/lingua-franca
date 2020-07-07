/** A data structure for a multiport instance. */

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

import java.util.ArrayList
import org.icyphy.linguaFranca.Port

/** Representation of a runtime instance of a multiport.
 *  This contains an array of ports.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class MultiportInstance extends PortInstance {
            
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     *  @param generator The generator (for error reporting).
     */
    new(Port definition, ReactorInstance parent, GeneratorBase generator) {
        super(definition, parent)
        
        if (definition.arraySpec === null) {
            throw new Exception("Port appears to not be a multiport: " + definition.name)
        }
        
        if (definition.arraySpec.ofVariableLength) {
            generator.reportError(definition,
                    "Variable-width multiports not supported (yet): " + definition.name)
        }
        
        // FIXME: What if the width is a parameter?
        val width = definition.arraySpec.length
        
        for (var i = 0; i < width; i++) {
            val instancePort = new PortInstance(definition, parent, i)
            instances.add(instancePort)
            // Inputs arriving at the instance port trigger the reactions
            // that depend on the multiport. 
            instancePort.dependentReactions = this.dependentReactions
            // Reactions that declare that they may send outputs via
            // this port are able to send on any of the instances.
            instancePort.dependsOnReactions = this.dependsOnReactions
        }
    }
    
    /**
     * Return the specified port instance in this multiport.
     */
    def getInstance(int position) {
        if (position < 0 || position >= instances.size) {
            throw new Exception("Port index out of range.")
        }
        return instances.get(position)
    }
    
    /**
     * Return the width of this port, which is the size of the instances list.
     */
    def getWidth() {
        instances.size
    }
        
    /////////////////////////////////////////////
    //// Public Fields

    /** The array of instances. */ 
    public val instances = new ArrayList<PortInstance>()
}