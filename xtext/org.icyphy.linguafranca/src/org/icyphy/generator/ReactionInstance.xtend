package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.linguaFranca.Reaction

/**
     * Inner class for each reaction instance.
     */
    class ReactionInstance extends NamedInstance<Reaction> {

       new (Reaction definition, ReactorInstance parent) {
            super(definition, parent);
            var index = 0;
            for (Reaction r : parent.definition.getReactorClass().getReactions()) {
                if (definition == r) {
                    this.reactionIndex = index
                }
                index++
            }
        }
        
        /** The ports that this reaction may write to. */
        public HashSet<PortInstance> dependentPorts = new HashSet<PortInstance>();

        /** The ports that this reaction is triggered by or uses. */
        public HashSet<PortInstance> dependsOnPorts = new HashSet<PortInstance>();

        /** The reactions that depend on this reaction. */
        public HashSet<ReactionInstance> dependentReactions = new HashSet<ReactionInstance>();

        /** The reactions that this reaction depends on. */
        public HashSet<ReactionInstance> dependsOnReactions = new HashSet<ReactionInstance>();

        /** The level in the dependence graph. */
        public int level = 0;

        /**
         * Place to store properties specific to a particular code generator.
         */
        public HashMap<String, String> properties = new HashMap<String, String>();

        /* Index of occurence in the reactor definition. */
        int reactionIndex = 0;

        override String getName() {
            return "r_" + this.reactionIndex;
        }
    }
