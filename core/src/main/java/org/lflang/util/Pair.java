package org.lflang.util;

/**
 * A pair whose first element has type `F` and whose second element has type `S`.
 * @ingroup Utilities
 */
public record Pair<F, S>(F first, S second) {}
