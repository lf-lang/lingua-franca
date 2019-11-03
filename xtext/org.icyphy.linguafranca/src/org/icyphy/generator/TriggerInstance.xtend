package org.icyphy.generator

import java.util.HashSet
import org.icyphy.linguaFranca.Variable

class TriggerInstance<T extends Variable> extends NamedInstance<Variable> {

    /** Construct a new instance with the specified definition
     *  and parent. E.g., for a action instance, the definition
     *  is Action, and for a port instance, it is Port. These are
     *  nodes in the AST. This is protected because only subclasses
     *  should be constructed.
     *  @param definition The definition in the AST for this instance.
     *  @param parent The reactor instance that creates this instance.
     */
    protected new(Variable definition, ReactorInstance parent) {
        super(definition, parent)
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /** Reaction instances that are triggered by this trigger. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** Reaction instances that may send outputs via this port. */
    public var dependsOnReactions = new HashSet<ReactionInstance>();

    /////////////////////////////////////////////
    //// Public Methods

    /** Return the name of this timer. 
     *  @return The name of this timer.
     */
    override getName() {
        this.definition.name
    }

    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        this.parent.main
    }   
}