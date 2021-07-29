package org.lflang.util;

import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Utilities to manipulate collections.
 */
public class CollectionUtil {

    /**
     * Returns a set which contains the elements of the given
     * set plus the given element. The returned set should be
     * considered unmodifiable.
     *
     * @param set initial set, nullable
     * @param t   new element
     * @param <T> Type of elements
     * @return A new set, or the same
     */
    public static <T> Set<T> plus(Set<T> set, T t) {
        if (set == null || set.isEmpty()) {
            return Set.of(t);
        } else if (set.size() == 1) {
            return Set.of(set.iterator().next(), t);
        } else if (set.size() == 2) {
            // make mutable
            set = new LinkedHashSet<>(set);
        } // else it's already mutable.

        set.add(t);
        return set;
    }


    /**
     * Returns a copy of the set. The returned set should be
     * considered unmodifiable.
     *
     * @param set initial set, nullable
     * @param <T> Type of elements
     * @return A new set, or the same
     */
    public static <T> Set<T> copy(Set<T> set) {
        if (set == null) {
            return null;
        } else if (set.size() <= 2) {
            return set; // it's unmodifiable
        } else {
            return new LinkedHashSet<>(set);
        }
    }

}
