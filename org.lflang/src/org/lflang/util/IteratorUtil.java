package org.lflang.util;

import java.util.Iterator;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

/**
 * A utility class for Iterator.
 *
 * @author Clément Fournier
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
