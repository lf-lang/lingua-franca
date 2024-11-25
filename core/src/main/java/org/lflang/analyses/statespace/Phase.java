package org.lflang.analyses.statespace;

/**
 * Common phases of a logical timeline, some of which are provided to the explorer as directives.
 */
public enum Phase {
    PREAMBLE,
    INIT, // Originated from startup triggers and initial timer firings
    PERIODIC,
    EPILOGUE,
    SYNC_BLOCK,
    INIT_AND_PERIODIC,
    SHUTDOWN_TIMEOUT,
    SHUTDOWN_STARVATION,
    ASYNC, // Originated from physical actions
}
