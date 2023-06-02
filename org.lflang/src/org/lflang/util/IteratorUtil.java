package org.lflang.util;

import java.util.Iterator;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

/**
 * A utility class for Iterator.
 *
 * @author Marten Lohstroh
 * @author Hokeun Kim
 */
public final class IteratorUtil {

  private IteratorUtil() {
    // Don't let anyone instantiate this class.
  }

  /**
   * Turn an iterator into a sequential stream.
   *
   * @param iterator The iterator to create a sequential stream for.
   * @return A stream.
   */
  public static <T> Stream<T> asStream(Iterator<T> iterator) {
    return asStream(iterator, false);
  }

  /**
   * Given an iterator of type T, turn it into a stream containing only the instances of the given
   * class of type S.
   *
   * @param <T> The type of elements the iterator iterates over.
   * @param <S> The type of class to filter out instance of.
   * @param iterator An iterator of type T.
   * @param cls A given class of type S.
   * @return A filtered stream that only has in it instances of the given class.
   */
  public static <T, S> Stream<S> asFilteredStream(Iterator<T> iterator, Class<S> cls) {
    return asStream(iterator, false).filter(cls::isInstance).map(cls::cast);
  }

  /**
   * Turn an iterator into a sequential or parallel stream.
   *
   * @param iterator The iterator to create a stream for.
   * @param parallel Whether or not the stream should be parallel.
   * @return A stream.
   */
  public static <T> Stream<T> asStream(Iterator<T> iterator, boolean parallel) {
    Iterable<T> iterable = () -> iterator;
    return StreamSupport.stream(iterable.spliterator(), parallel);
  }

  /**
   * Function to get Iterable from Iterator.
   *
   * @param iterator The iterator to get an iterable from.
   * @return An iterable.
   */
  public static <T> Iterable<T> asIterable(Iterator<T> iterator) {
    return () -> iterator;
  }
}
