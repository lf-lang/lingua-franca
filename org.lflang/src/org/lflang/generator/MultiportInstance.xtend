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
            throw new InvalidSourceException("Port appears to not be a multiport: " + definition.name)
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
    }
}
