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

import static extension org.icyphy.ASTUtils.*
import org.icyphy.linguaFranca.ActionOrigin

/**
 * Instance of an action.
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ActionInstance extends TriggerInstance<Variable> {
    
    var shutdown = false
    
    public TimeValue minDelay = new TimeValue(0, TimeUnit.NONE)
    
    public TimeValue minSpacing = null;
    
    public boolean isPhysical = false;
    
    new(Action definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create an ActionInstance with no parent.')
        }
        if (definition.name.equals(LinguaFrancaPackage.Literals.TRIGGER_REF__SHUTDOWN.name)) {
            this.shutdown = true
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
        if (definition.minSpacing !== null) {
            if (definition.minSpacing.parameter !== null) {
                val parm = definition.minSpacing.parameter
                this.minSpacing = parent.lookupParameterInstance(parm).init.
                    get(0).getTimeValue
            } else {
                this.minSpacing = definition.minSpacing.timeValue
            }
        }
        if (definition.origin === ActionOrigin.PHYSICAL) {
            isPhysical = true
        }
    }
    
    def isShutdown() {
        this.shutdown
    }
}