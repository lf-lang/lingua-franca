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
package org.lflang.generator

import java.util.LinkedHashSet
import org.eclipse.xtend.lib.annotations.Accessors
import org.lflang.ErrorReporter
import org.lflang.lf.Port

/**
 * Representation of a runtime instance of a multiport.
 * This contains an array of ports.
 *  
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class MultiportInstance extends PortInstance {
            
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     *  @param errorReporter The error reporter.
     */
    new(Port definition, ReactorInstance parent, ErrorReporter reporter) {
        super(definition, parent)
        
        if (definition.widthSpec === null) {
            throw new Exception("Port appears to not be a multiport: " + definition.name)
        }
        
        if (definition.widthSpec.ofVariableLength) {
            reporter.reportError(definition,
                    "Variable-width multiports not supported (yet): " + definition.name)
            return
        }
        
        // The width may be given by a parameter or even sum of parameters.
        var width = 0
        for (term : definition.widthSpec.terms) {
            if (term.parameter !== null) {
                val parameterValue = parent.initialIntParameterValue(term.parameter)
                // Only a Literal is supported.
                if (parameterValue !== null) {
                    // This could throw NumberFormatException
                    width += parameterValue
                } else {
                    reporter.reportWarning(definition,
                        "Width of a multiport cannot be determined. Assuming 1."
                    )
                    width += 1
                }
            } else {
                width += term.width
            }
        }
        
        for (var i = 0; i < width; i++) {
            val instancePort = new PortInstance(definition, parent, i, this)
            instances.add(instancePort)
            // Inputs arriving at the instance port trigger the reactions
            // that depend on the multiport. 
            instancePort.dependentReactions = this.dependentReactions
            // Reactions that declare that they may send outputs via
            // this port are able to send on any of the instances.
            instancePort.dependsOnReactions = this.dependsOnReactions
        }
    }
    
    /////////////////////////////////////////////
    //// Public Fields

    /** The array of instances. */
    @Accessors(PUBLIC_GETTER)
    val instances = new LinkedHashSet<PortInstance>()

    /////////////////////////////////////////////
    //// Public Methods

    /**
     * Return the list of ports that this port depends on.
     * For ordinary ports, there is at most one.
     * For multiports, there may be more than one.
     */
    override dependsOnPorts() {
        return instances;
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
}