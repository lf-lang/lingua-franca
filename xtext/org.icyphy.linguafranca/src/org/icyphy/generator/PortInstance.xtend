/** A data structure for a port instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.generator.ReactionGraph.ReactionInstance
import org.icyphy.linguaFranca.Port

/** Representation of a runtime instance of a port.
 */
class PortInstance {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Port definition, ReactorInstance parent) {
        this.definition = definition
        this.parent = parent
    }
    
    /** The Instantiation AST object from which this was created. */
    public var Port definition
    
    /** Reaction instances that are triggered by this port. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** Reaction instances that may send outputs via this port. */
    public var dependsOnReactions = new HashSet<ReactionInstance>();

    /** The reactor instance that instantiated this port instance. */
    public var ReactorInstance parent
    
    /** Properties associated with this instance.
     *  This is used by particular code generators.
     */
    public var HashMap<String,Object> properties = new HashMap<String,Object>()
    
    /////////////////////////////////////////////
    
    /** Return the full name of this instance, which has the form
     *  "a.b.c", where "c" is the name of this instance, "b" is the name
     *  of its parent, and "a" is the name of its parent, stopping
     *  at the container in main.
     *  @return The full name of this instance.
     */
    def String getFullName() {
        if (parent !== null) {
            this.parent.getFullName() + '.' + definition.name
        } else {
            definition.name
        }
    }
}