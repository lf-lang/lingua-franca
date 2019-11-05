/** A data structure for a port instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Variable

/** Representation of a runtime instance of a port.
 */
class PortInstance extends TriggerInstance<Variable> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Port definition, ReactorInstance parent) {
        super(definition, parent)
        
        if (parent === null) {
            throw new Exception('Cannot create a PortInstance with no parent.')
        }
    }
        
    /////////////////////////////////////////////
    //// Public Fields

    /** Set of port instances that receive messages from this port. */
    public HashSet<PortInstance> dependentPorts = new HashSet<PortInstance>();
        
    /** Port that sends messages to this port, if there is one. */
    public PortInstance dependsOnPort = null;
    
    /////////////////////////////////////////////
    //// Public Methods
    
    /** Return true if the port is an input. */
    def isInput() {
        definition instanceof Input
    }

    /** Return true if the port is an output. */
    def isOutput() {
        definition instanceof Output
    }

    /** Return a descriptive string. */
    override toString() {
        "PortInstance " + getFullName
    }
}