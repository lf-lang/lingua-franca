package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Reaction;

import java.util.HashMap; // import the HashMap class

/**
 * LetUtils implements a hashmap of Reactions to Lets. This way we can store the Let
 * of a Reaction while traversing the Reaction graph and access it later from the ReactionIstances
 */
public class LetUtils {
    static HashMap<Reaction, TimeValue> reactionLets = new HashMap<Reaction, TimeValue>();

    public static TimeValue getReactionLet(Reaction reaction) {
        return reactionLets.getOrDefault(reaction, TimeValue.ZERO);
    }

    public static void setReactionLet(Reaction reaction, TimeValue let) {
        reactionLets.put(reaction,let);
    }
}
