/** A data structure for a port instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Param

/** Representation of a runtime instance of a port.
 */
class ParameterInstance extends NamedInstance<Param> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Param definition, ReactorInstance parent) {
        super(definition, parent)
        
        if (parent === null) {
            throw new Exception('Cannot create a ParameterInstance with no parent.')
        }
        
        type = parent.generator.removeCodeDelimiter(definition.type)
        if (type.equals("time")) {
            type = parent.generator.timeTypeInTargetLanguage()
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /** The type of the parameter, stripped of the code delimiters
     *  {= ... =} if they were provided.
     */
    public var type = "UNTYPED"
    
    /** The value of the parameter. This defaults to the value given
     *  in the reactor class definition, but if the parameter is
     *  overridden in instantiation, then that value is returned.
     *  In both cases, the value is stripped of the code delimiters
     *  {= ... =} if they were provided.
     */
    var value = null as String

    /////////////////////////////////////////////
    //// Public Methods

    /** Return the name of this parameter. 
     *  @return The name of this parameter.
     */
    override String getName() {
        this.definition.name
    }

    /** Get the value of the parameter, stripped of the code delimiters
     *  {= ... =} if they were provided.
     */
    def getValue() {
        // If value has been previously determined, just return it.
        if (value !== null) {
            value
        } else {
            // Check for an override.
            if (parent.definition.parameters !== null) {
                for (assignment : parent.definition.parameters.assignments ?: emptyList) {
                    if (assignment.name == parent.definition.name) {
                        // Parameter is overridden.
                        var value = assignment.value;
                        if (assignment.unit !== null) {
                            var time = LinguaFrancaFactory.eINSTANCE.createTime()
                            time.setTime(value)
                            time.setUnit(assignment.unit)
                            value = parent.generator.timeInTargetLanguage(time)
                        }
                        return parent.generator.removeCodeDelimiter(value)
                    }
                }
            }
            // If we get here, then the parameter is not overridden.
            var value = definition.value
            if (definition.time !== null) {
                value = parent.generator.timeInTargetLanguage(definition.time)
            }
            parent.generator.removeCodeDelimiter(value)
        }
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