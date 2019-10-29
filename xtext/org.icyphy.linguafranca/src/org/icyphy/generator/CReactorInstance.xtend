/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Variable

/** Representation of a runtime instance of a reactor.
 */
class CReactorInstance extends ReactorInstance {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     *  @param generator The generator creating this instance.
     */
    protected new(Instantiation definition, ReactorInstance parent, GeneratorBase generator) {
        super(definition, parent, generator)
    }
    
    /** Map from a reaction definition to the name of its reaction_t struct. */
    public var reactionToReactionTName = new HashMap<Reaction, String>()

    /** Map from an Input, Output, or Action to the name of its trigger_t struct. */
    public var triggerToTriggerStructName = new HashMap<Variable, String>()
}