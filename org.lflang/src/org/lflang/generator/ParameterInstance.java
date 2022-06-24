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

package org.lflang.generator;

import java.util.List;
import java.util.Optional;

import org.lflang.InferredType;
import org.lflang.ASTUtils;
import org.lflang.lf.Assignment;
import org.lflang.lf.Expression;
import org.lflang.lf.Parameter;

/** 
 * Representation of a compile-time instance of a parameter.
 * Upon creation, it is checked whether this parameter is overridden by an 
 * assignment in the instantiation that this parameter instance is a result of.
 * If it is overridden, the parameter gets initialized using the value looked up
 * in the instantiation hierarchy.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
public class ParameterInstance extends NamedInstance<Parameter> {
        
    /** 
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param definition The declaration in the AST.
     * @param parent The reactor instance this parameter is a part of.
     */
    public ParameterInstance(Parameter definition, ReactorInstance parent) {
        super(definition, parent);
        if (parent == null) {
            throw new InvalidSourceException("Cannot create a ParameterInstance with no parent.");
        }
        
        this.type = ASTUtils.getInferredType(definition);
    }

    /////////////////////////////////////////////
    //// Public Fields
        
    public InferredType type;
    
    /////////////////////////////////////////////
    //// Public Methods

    /**
     * Get the initial value(s) of this parameter as a list of
     * Value objects, where each Value is either an instance
     * of Time, Literal, or Code. That is, references to other
     * parameters have been replaced with their initial values.
     */
    public List<Expression> getInitialValue() {
        return parent.initialParameterValue(this.definition);
    }
    
    /**
     * Return the name of this parameter. 
     * @return The name of this parameter.
     */
    public String getName() {
        return this.definition.getName();
    }
    
    /**
     * Return the assignment that overrides this parameter in
     * the parent's instantiation or null if there is no override.
     */
    public Assignment getOverride() {
        List<Assignment> assignments = parent.definition.getParameters();
        Optional<Assignment> assignment = assignments.stream().filter(
            it -> it.getLhs() == definition
        ).findFirst();
        return assignment.orElse(null);
    }
	
    /** Return a descriptive string. */
    @Override
    public String toString() {
        return "ParameterInstance " + getFullName();
    }
}
