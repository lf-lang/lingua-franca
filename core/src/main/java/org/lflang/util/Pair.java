package org.lflang.util;

/** A pair whose first element has type {@code F} and whose second element has type {@code S}. */
public record Pair<F, S>(F first, S second) {}
