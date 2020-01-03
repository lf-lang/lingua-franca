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

import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.TimeUnit

/** Representation of a runtime instance of a parameter.
 *  
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
abstract class ParameterInstance extends NamedInstance<Parameter> {
        
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
    }

    /////////////////////////////////////////////
    //// Public Fields
    
//    /** The type of the parameter, stripped of the code delimiters
//     *  {= ... =} if they were provided.
//     */
    public var type = "UNTYPED"
    /////////////////////////////////////////////
    //// Public Methods

    /** Return the name of this parameter. 
     *  @return The name of this parameter.
     */
    override String getName() {
        this.definition.name
    }

	abstract def String getLiteralValue();
	
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

class ValueParameter extends ParameterInstance {
	new(Parameter definition, ReactorInstance parent, String value, String type) {
		super(definition, parent)
		this.type = type
		this.value = value
	}
	public var value = ""
	
	/** The value of the parameter. This defaults to the value given
     *  in the reactor class definition, but if the parameter is
     *  overridden in instantiation, then that value is returned.
     *  In both cases, the value is stripped of the code delimiters
     *  {= ... =} if they were provided.
     */
	override getLiteralValue() {
		return GeneratorBase.removeCodeDelimiter(this.value)
	}
}

class TimeParameter extends ParameterInstance {
	new(Parameter definition, ReactorInstance parent, int value, TimeUnit unit) {
		super(definition, parent)
		this.value = value
		this.unit = unit
		this.type = parent.generator.timeTypeInTargetLanguage()
	}
	public var value = 0
	
	public var unit = TimeUnit.NONE
	
    /** The time value of the parameter in the target language. */
	override getLiteralValue() {
		return parent.generator.timeInTargetLanguage(this.value.toString, this.unit)
	}
}
