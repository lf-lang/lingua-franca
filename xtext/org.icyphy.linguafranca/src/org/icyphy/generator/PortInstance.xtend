/** A data structure for a port instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.linguaFranca.Port

/** Representation of a runtime instance of a port.
 */
class PortInstance extends NamedInstance<Port> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Port definition, ReactorInstance parent) {
        super(definition, parent)
    }
        
    /** Reaction instances that are triggered by this port. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** Reaction instances that may send outputs via this port. */
    public var dependsOnReactions = new HashSet<ReactionInstance>();
    
    /** Properties associated with this instance.
     *  This is used by particular code generators.
     */
    public var HashMap<String,Object> properties = new HashMap<String,Object>()
    
    /////////////////////////////////////////////
				
    /** Return the name of this port. 
     *  @return The name of this port.
     */
    override String getName() {
    	this.definition.name
    }

}