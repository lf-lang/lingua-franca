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

package org.lflang.generator

import org.eclipse.xtend.lib.annotations.Accessors
import org.lflang.TimeValue
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.TimeUnit

import static extension org.lflang.ASTUtils.*

/**
 * Instance of an action.
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ActionInstance extends TriggerInstance<Action> {
    
    /** The constant default for a minimum delay. */
    public static val DEFAULT_MIN_DELAY = new TimeValue(0, TimeUnit.NONE)
    
    @Accessors(PUBLIC_GETTER)
    TimeValue minDelay = DEFAULT_MIN_DELAY
    
    // TODO introduce default value?
    @Accessors(PUBLIC_GETTER)
    TimeValue minSpacing = null;
    
    @Accessors(PUBLIC_GETTER)
    String policy = null;
    
    @Accessors(PUBLIC_GETTER)
    boolean isPhysical;
    
    /**
     * Create a new timer instance.
     * If the definition is null, then this is a shutdown action.
     * @param definition The AST definition, or null for startup.
     * @param parent The parent reactor.
     */
    new(Action definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new InvalidSourceException('Cannot create an ActionInstance with no parent.')
        }
        if (definition !== null) {
            if (definition.minDelay !== null) {
                if (definition.minDelay.parameter !== null) {
                    val parm = definition.minDelay.parameter
                    this.minDelay = parent.lookupParameterInstance(parm).init.get(0).getTimeValue
                } else {
                    this.minDelay = definition.minDelay.timeValue
                }
            }
            if (definition.minSpacing !== null) {
                if (definition.minSpacing.parameter !== null) {
                    val parm = definition.minSpacing.parameter
                    this.minSpacing = parent.lookupParameterInstance(parm).init.get(0).getTimeValue
                } else {
                    this.minSpacing = definition.minSpacing.timeValue
                }
            }
            if (definition.origin === ActionOrigin.PHYSICAL) {
                isPhysical = true
            }
            policy = definition.policy
        }
    }
}
