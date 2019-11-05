package org.icyphy.generator

import java.util.HashSet
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Port
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
        
        // Identify the dependencies for this reaction.
        // First handle the triggers.
        for (trigger : definition.triggers) {
            if (trigger instanceof Port) {
                var portInstance = parent.getPortInstance(trigger)
                this.dependsOnPorts.add(portInstance)
                portInstance.dependentReactions.add(this)
            }
        }
        // Next handle the ports that this reaction reads.
        for (source : definition.sources) {
            if (source instanceof Port) {
                var portInstance = parent.getPortInstance(source)
                this.dependsOnPorts.add(portInstance)
                portInstance.dependentReactions.add(this)
            }
        }
        // Finally, handle the effects.
        for (effect : definition.effects) {
            if (effect.variable instanceof Port) {
                var portInstance = parent.getPortInstance(effect)
                this.dependentPorts.add(portInstance)
                portInstance.dependsOnReactions.add(this)
            } else {
                // Effect must be an Action.
                var actionInstance = parent.getActionInstance(effect.variable as Action)
                this.dependentActions.add(actionInstance)
                actionInstance.dependsOnReactions.add(this)
            }
        }
    }

    /** The actions that this reaction triggers. */
    public var dependentActions = new HashSet<ActionInstance>();
        
    /** The ports that this reaction may write to. */
    public var dependentPorts = new HashSet<PortInstance>();

    /** The reactions that depend on this reaction. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** The actions that this reaction is triggered by. */
    public var dependsOnActions = new HashSet<ActionInstance>();

    /** The ports that this reaction is triggered by or uses. */
    public var dependsOnPorts = new HashSet<PortInstance>();

    /** The timers that this reaction is triggered by. */
    public var dependsOnTimers = new HashSet<TimerInstance>();

    /** The reactions that this reaction depends on. */
    public var dependsOnReactions = new HashSet<ReactionInstance>();

    /** The level in the dependence graph. -1 indicates that the level
     *  has not yet been assigned.
     */
    public int level = -1;

    /** Index of order of occurrence within the reactor definition.
     *  The first reaction has index 0, the second index 1, etc.
     */
    public int reactionIndex = -1;

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
    
    /** Return a descriptive string. */
    override toString() {
        getName + " of " + parent.getFullName
    }
}
