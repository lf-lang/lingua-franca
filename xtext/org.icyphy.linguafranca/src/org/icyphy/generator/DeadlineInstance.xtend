/** Instance of a timer. */

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

import org.icyphy.linguaFranca.Deadline
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.TimeUnit

import static extension org.icyphy.ASTUtils.*


/** Instance of a timer.
 * 
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class DeadlineInstance extends NamedInstance<Deadline> {
	
	public TimeValue delay = new TimeValue(0, TimeUnit.NONE)
	
	new(Deadline definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a DeadlineInstance with no parent.')
        }
        
        if (definition.delay !== null) {
            if (definition.delay.parameter !== null) {
                this.delay = parent.parameters.findFirst [
                    it.definition === definition.delay.parameter
                ].init.get(0).getTimeValue
            } else {
                this.delay = definition.delay.timeValue
            }
        }        
	}

    override getName() '''deadline'''

    override main() {
        parent.main
    }
    
    override toString() '''DeadlineInstance «getFullName»'''
    
}