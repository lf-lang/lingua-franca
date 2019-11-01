package org.icyphy.generator

import java.util.HashSet
import org.icyphy.linguaFranca.Reaction

/** Instance of a reaction. */
class ReactionInstance extends NamedInstance<Reaction> {

    /** Create a new reaction instance from the specified definition
     *  within the specified parent. This constructor should be called
     *  only by the ReactionInstance class.
     *  @param definition A reaction definition.
     *  @param parent The parent reactor instance, which cannot be null.
     *  @param index The index of the reaction within the reactor (0 for the
     *   first reaction, 1 for the second, etc.).
     */
    protected new (Reaction definition, ReactorInstance parent, int index) {
        super(definition, parent);
        this.reactionIndex = index
    }
        
    /** The ports that this reaction may write to. */
    public HashSet<PortInstance> dependentPorts = new HashSet<PortInstance>();

    /** The reactions that depend on this reaction. */
    public HashSet<ReactionInstance> dependentReactions = new HashSet<ReactionInstance>();

    /** The ports that this reaction is triggered by or uses. */
    public HashSet<PortInstance> dependsOnPorts = new HashSet<PortInstance>();

    /** The reactions that this reaction depends on. */
    public HashSet<ReactionInstance> dependsOnReactions = new HashSet<ReactionInstance>();

    /** The level in the dependence graph. */
    public int level = 0;

    /* Index of occurrence in the reactor definition. */
    int reactionIndex = 0;

    /** Return the name of this reaction, which is 'reaction_n',
     *  where n is replaced by the reactionIndex. 
     *  @return The name of this reaction.
     */
    override String getName() {
        return "reaction_" + this.reactionIndex;
    }
    
    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        parent.main
    }
}
