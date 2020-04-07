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

package org.icyphy.generator

import java.util.List
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Value

/** Representation of a runtime instance of a parameter.
 *  
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ParameterInstance extends NamedInstance<Parameter> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Parameter definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a ParameterInstance with no parent.')
        }
        
        this.type = definition.type
        this.init = definition.init.clone
        
        // Check for an override.
        for (assignment : parent.definition.parameters ?: emptyList) {
            var rhs = assignment.rhs
            if (assignment.lhs === definition) {
                // Parameter is overridden using a reference to another parameter.
                if (rhs.parameter !== null) {
                    // Find the reactor that has the parameter that the assignment refers to.
                    var reactor = rhs.parameter.eContainer as Reactor
                    // Look the up the container of the parameter (in the instance hierarchy)
                    // to find the matching instance
                    var instance = parent
                    var found = false
                    while (instance.parent !== null && !found) {
                        instance = instance.parent
                        if (instance.definition.reactorClass === reactor) {
                            found = true
                        }
                    }
                    if (!found) {
                        throw new InternalError(
                            "Incorrect reference to parameter:" +
                                definition.name);
                    }

                    val referencedParameter = instance.
                        getParameterInstance(rhs.parameter)
        
                    this.type = referencedParameter.type
                    this.init = referencedParameter.init
                } else {
                    // Parameter is overridden by a singleton value.
                    this.init.set(0, rhs)                   
                }        
            }
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    public List<Value> init
    
    public Type type
    
    /////////////////////////////////////////////
    //// Public Methods

    /** Return the name of this parameter. 
     *  @return The name of this parameter.
     */
    override String getName() {
        this.definition.name
    }

	def String getLiteralValue() {
	    // If this is an list
	    
	    // If this is a literal
	    
	    // If this is code
	    // FIXME: look in ASTUtils
	}
	
    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        parent.main
    }

    /** Return a descriptive string. */
    override toString() {
        "ParameterInstance " + getFullName
    }
}

//class ValueParameter extends ParameterInstance {
//	new(Parameter definition, ReactorInstance parent, String value, Type type) {
//		super(definition, parent)
//		this.type = definition.type
//		this.value = value
//		
//	}
//	public var value = ""
//	
//	/** The value of the parameter. This defaults to the value given
//     *  in the reactor class definition, but if the parameter is
//     *  overridden in instantiation, then that value is returned.
//     *  In both cases, the value is stripped of the code delimiters
//     *  {= ... =} if they were provided.
//     */
//	override getLiteralValue() {
//		return this.value
//	}
//}
//
//class TimeParameter extends ParameterInstance {
//	new(Parameter definition, ReactorInstance parent, TimeValue timeValue) {
//		super(definition, parent)
//		this.type = definition.type
//		this.value = timeValue
//	}
//	
//	public TimeValue value
//	
//    /** The time value of the parameter in the target language. */
//	override getLiteralValue() {
//		return parent.generator.timeInTargetLanguage(this.value)
//	}
//	
//}