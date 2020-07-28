/* Instance of an action. */

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

import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Variable
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.ActionOrigin

import static extension org.icyphy.ASTUtils.*

/**
 * Instance of an action.
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ActionInstance extends TriggerInstance<Variable> {
    
    var shutdown = false
    
    public TimeValue minDelay = new TimeValue(0, TimeUnit.NONE)
    
    public TimeValue minInterArrival = new TimeValue(0, TimeUnit.NONE)
    
    new(Action definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create an ActionInstance with no parent.')
        }
        if (definition.name.equals(LinguaFrancaPackage.Literals.TRIGGER_REF__SHUTDOWN.name)) {
        	this.shutdown = true
        }
        // For physical actions, if no MIT is specified, set it to something
        // non-zero.
        if (definition.origin == ActionOrigin.PHYSICAL) {
            this.minInterArrival = CGenerator.DEFAULT_MIN_INTER_ARRIVAL
        }
        
        if (definition.minDelay !== null) {
            if (definition.minDelay.parameter !== null) {
                val parm = definition.minDelay.parameter
                this.minDelay = parent.lookupParameterInstance(parm).init.get(0).
                    getTimeValue
            } else {
                this.minDelay = definition.minDelay.timeValue
            }
        }
        if (definition.minInterArrival !== null) {
            if (definition.minInterArrival.parameter !== null) {
                val parm = definition.minInterArrival.parameter
                this.minInterArrival = parent.lookupParameterInstance(parm).init.
                    get(0).getTimeValue
            } else {
                this.minInterArrival = definition.minInterArrival.timeValue
            }
        }
    }
    
    def isShutdown() {
    	this.shutdown
    }
}