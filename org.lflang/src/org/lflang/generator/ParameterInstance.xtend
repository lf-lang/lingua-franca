/** A data structure for a parameter instance. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator

import java.util.LinkedList
import java.util.List
import org.lflang.InferredType
import org.lflang.lf.LfFactory
import org.lflang.lf.Parameter
import org.lflang.lf.Value

import static extension org.lflang.ASTUtils.*

/** 
 * Representation of a runtime instance of a parameter.
 * Upon creation, it is checked whether this parameter is overridden by an 
 * assignment in the instantiation that this parameter instance is a result of.
 * If it is overridden, the parameter gets initialized using the value looked up
 * in the instantiation hierarchy.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ParameterInstance extends NamedInstance<Parameter> {
        
    /** 
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param instance The Instance statement in the AST.
     * @param parent The reactor instance this parameter is a part of.
     */
    new(Parameter definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a ParameterInstance with no parent.')
        }
        
        this.type = definition.inferredType
        this.init = definition.init
        
        // Check for an override.
        var assignment = parent.definition.parameters.findFirst[it.lhs === definition]
        
        while (assignment !== null) {
            // NOTE: we only allow a reference to single a parameter or 
            // a list of ordinary values.
            val ref = assignment.rhs.get(0).parameter
            if (ref !== null) {
                // Get the value from the parameter instance, not the parameter
                // so that overrides like bank_index work.
                // parent.parent will be non-null or the rhs parameter reference
                // would not have passed validation. Nevertheless, we check.
                val parentsParent = parent.parent;
                if (parentsParent !== null) {
                    val parameterInstance = parentsParent.parameters.findFirst[it.name.equals(ref.name)]
                    // Again, this result should be non-null, but we check.
                    if (parameterInstance !== null) {
                        this.init = parameterInstance.init
                    } else {
                        // Fall back on reference.
                        this.init = ref.init
                    }
                } else {
                    // Fall back on reference.
                    this.init = ref.init
                }
            } else {
                this.init = assignment.rhs    
            }
            if (parent.parent !== null) {
                assignment = parent.parent.definition.parameters.findFirst[it.lhs === ref]
            } else {
                assignment = null
            }
        }
        
        // If the parent is in a bank and the parameter name is "bank_index", then
        // override the default value provided to make it equal to the bank index.
        if (parent.bankIndex >= 0 && definition.name.equals("bank_index")) {
            val value = LfFactory.eINSTANCE.createValue
            value.literal = "" + parent.bankIndex
            val list = new LinkedList<Value>()
            list.add(value)
            this.init = list
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    public List<Value> init
    
    public InferredType type
    
    /////////////////////////////////////////////
    //// Public Methods

    /**
     * Return the name of this parameter. 
     * @return The name of this parameter.
     */
    override String getName() {
        this.definition.name
    }
	
    /**
     * {@inheritDoc}
     */
    override ReactorInstance root() {
        parent.root()
    }

    /** Return a descriptive string. */
    override toString() {
        "ParameterInstance " + getFullName
    }
}
