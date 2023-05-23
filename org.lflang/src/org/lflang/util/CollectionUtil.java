package org.lflang.util;

import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Function;

/**
 * Utilities to manipulate collections.
 *
 * <p>Most of these methods are using specialized collection implementations (possibly unmodifiable)
 * for small collection sizes. No guarantee is made on the mutability of the collections returned
 * from these functions, meaning, callers should always assume they are unmodifiable. Functions that
 * take a collection parameter as input to produce a new one with a transformation require the input
 * collection to have been obtained from one of the utility functions of this class in the first
 * place.
 */
public class CollectionUtil {

  /**
   * Returns a set which contains the elements of the given set plus the given element. The returned
   * set should be considered unmodifiable.
   *
   * @param set initial set, nullable
   * @param t new element
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
      // make mutable
      set = new LinkedHashSet<>(set);
    } // else it's already mutable.

    set.add(t);
    return set;
  }

  public static <K, V> Map<K, V> plus(Map<K, V> map, K k, V v) {
    if (map == null || map.isEmpty()) {
      return Collections.singletonMap(k, v);
    } else if (map.size() == 1) {
      Entry<K, V> e = map.entrySet().iterator().next();
      if (e.getKey().equals(k)) {
        return Map.of(k, v);
      }
      // make mutable
      map = new LinkedHashMap<>(map);
    } // else it's already mutable.

    map.put(k, v);
    return map;
  }

  /**
   * Remove the given value from all the sets that are values in the given map. Use this if the
   * values of the map (the sets) were build with {@link #plus(Set, Object)}.
   *
   * <p>In {@link org.lflang.graph.DirectedGraph}, this is used to properly remove nodes from a
   * graph. There, we use maps to represent edges, where a value in a map is a set of nodes adjacent
   * to the key for that value. Hence, when a node is removed, it needs to be removed not just as a
   * key, but it also needs to be removed from the neighbors sets of any other keys that may contain
   * it.
   *
   * @param map A modifiable map
   * @param valueToRemove Value to remove
   */
  public static <K, V> void removeFromValues(Map<K, Set<V>> map, V valueToRemove) {
    mapValuesInPlace(map, set -> minus(set, valueToRemove));
  }

  /**
   * Apply a transform to the values of the map. If the mapping function returns null, the entry is
   * removed.
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

  /**
   * Returns a set that contains the elements of the given set minus one element. An empty set is
   * considered empty.
   */
  public static <T> Set<T> minus(Set<T> set, T eltToRemove) {
    if (set instanceof LinkedHashSet) {
      set.remove(eltToRemove);
      return set;
    }

    if (set == null || set.isEmpty()) {
      return Collections.emptySet();
    } else if (set.size() == 1) {
      return set.contains(eltToRemove) ? Collections.emptySet() : set;
    }
    throw new AssertionError("should be unreachable");
  }

  /**
   * Returns a map that is identical to the original map, except the value for key {@code k} is
   * transformed using the given function. The transformation function takes the key and current
   * value (null if the key is not present) as inputs, and returns the new value to associate to the
   * key (null if the mapping should be removed).
   *
   * @see Map#compute(Object, BiFunction)
   */
  public static <K, V> Map<K, V> compute(Map<K, V> map, K k, BiFunction<K, V, V> computation) {
    if (map == null || map.isEmpty()) {
      return Collections.singletonMap(k, computation.apply(k, null));
    } else if (map.size() == 1) {
      Entry<K, V> e = map.entrySet().iterator().next();
      if (e.getKey().equals(k)) {
        return Collections.singletonMap(k, computation.apply(k, e.getValue()));
      }
      // make mutable
      map = new LinkedHashMap<>(map);
    } // else it's already mutable.

    map.compute(k, computation);
    return map;
  }

  /**
   * Returns a copy of the set. The returned set should be considered unmodifiable.
   *
   * @param set initial set, nullable
   * @param <T> Type of elements
   * @return A new set, or the same
   */
  public static <T> Set<T> copy(Set<T> set) {
    if (set == null || set.size() <= 1) {
      return set; // it's unmodifiable
    } else {
      return new LinkedHashSet<>(set);
    }
  }

  /**
   * Returns an immutable Set that contains all argument values. Duplicate elements are removed
   * without error (contrary to {@link Set#of()} and friends).
   */
  @SafeVarargs
  public static <T> Set<T> immutableSetOf(T first, T... rest) {
    if (rest.length == 0) {
      return Set.of(first);
    }
    Set<T> result = new LinkedHashSet<>();
    result.add(first);
    result.addAll(Arrays.asList(rest));
    return result;
  }
}
