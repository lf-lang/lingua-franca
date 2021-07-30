package org.lflang.util;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Function;

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
            if (set.contains(t)) {
                return set;
            }
            return Set.of(set.iterator().next(), t);
        } else if (set.size() == 2) {
            // make mutable
            set = new LinkedHashSet<>(set);
        } // else it's already mutable.

        set.add(t);
        return set;
    }


    public static <K, V> Map<K, V> plus(Map<K, V> map, K k, V v) {
        if (map == null || map.isEmpty()) {
            return Map.of(k, v);
        } else if (map.size() == 1) {
            Entry<K, V> e = map.entrySet().iterator().next();
            if (e.getKey().equals(k)) {
                return Map.of(k, v);
            }
            return Map.of(e.getKey(), e.getValue(), k, v);
        } else if (map.size() == 2) {
            // make mutable
            map = new LinkedHashMap<>(map);
        } // else it's already mutable.

        map.put(k, v);
        return map;
    }


    /**
     * Remove the given value from all the sets that are values
     * in the given map. Use this if the values of the map (the
     * sets) were build with {@link #plus(Set, Object)}.
     *
     * @param map           A *modifiable* map
     * @param valueToRemove Value to remove
     */
    public static <K, V> void removeFromValues(Map<K, Set<V>> map, V valueToRemove) {
        mapValuesInPlace(map, set -> minus(set, valueToRemove));
    }


    /**
     * Apply a transform to the values of the map. If the mapping
     * function returns null, the entry is removed.
     */
    private static <K, V> void mapValuesInPlace(Map<K, V> map, Function<V, V> mapper) {
        Iterator<Entry<K, V>> iterator = map.entrySet().iterator();
        while (iterator.hasNext()) {
            Entry<K, V> e = iterator.next();
            V existing = e.getValue();
            V mapped = mapper.apply(existing);
            if (mapped == null) {
                iterator.remove();
            } else if (mapped != existing) {
                e.setValue(mapped);
            }
        }
    }


    private static <T> Set<T> minus(Set<T> set, T t) {
        if (set == null || set.isEmpty()) {
            return null;
        } else if (set.size() == 1) {
            return set.contains(t) ? null : set;
        } else if (set.size() == 2 && set.contains(t)) {
            Iterator<T> iter = set.iterator();
            T fst = iter.next();
            T snd = iter.next();
            if (Objects.equals(fst, t)) {
                return Set.of(snd);
            } else if (Objects.equals(snd, t)) {
                return Set.of(fst);
            }
            return set;
        } // else it's mutable.

        set.remove(t);
        return set;
    }


    public static <K, V> Map<K, V> compute(Map<K, V> map, K k, BiFunction<K, V, V> computation) {
        if (map == null || map.isEmpty()) {
            return Map.of(k, computation.apply(k, null));
        } else if (map.size() == 1) {
            Entry<K, V> e = map.entrySet().iterator().next();
            if (e.getKey().equals(k)) {
                return Map.of(k, computation.apply(k, e.getValue()));
            }
            return Map.of(e.getKey(), e.getValue(), k, computation.apply(k, null));
        } else if (map.size() == 2) {
            // make mutable
            map = new LinkedHashMap<>(map);
        } // else it's already mutable.

        map.compute(k, computation);
        return map;
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
