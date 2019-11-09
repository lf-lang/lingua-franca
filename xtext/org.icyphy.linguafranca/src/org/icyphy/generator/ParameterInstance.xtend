/** A data structure for a port instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.TimeUnit

/** Representation of a runtime instance of a port.
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
	//    /** The value of the parameter. This defaults to the value given
//     *  in the reactor class definition, but if the parameter is
//     *  overridden in instantiation, then that value is returned.
//     *  In both cases, the value is stripped of the code delimiters
//     *  {= ... =} if they were provided.
//     */
	public var value = ""
	
	override getLiteralValue() {
		return this.value
	}
}

class TimeParameter extends ParameterInstance {
	new(Parameter definition, ReactorInstance parent, int value, TimeUnit unit) {
		super(definition, parent)
		this.value = value
		this.unit = unit
		this.type = parent.generator.timeTypeInTargetLanguage()
	}
	//    /** The value of the parameter. This defaults to the value given
//     *  in the reactor class definition, but if the parameter is
//     *  overridden in instantiation, then that value is returned.
//     *  In both cases, the value is stripped of the code delimiters
//     *  {= ... =} if they were provided.
//     */
	public var value = 0
	
	public var unit = TimeUnit.NONE
	
	override getLiteralValue() {
		return parent.generator.timeInTargetLanguage(this.value.toString, this.unit)
	}
}
