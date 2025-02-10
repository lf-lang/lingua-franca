package org.lflang.pretvm;

public enum ExecutionPhase {
  PREAMBLE,
  INIT,
  PERIODIC,
  EPILOGUE,
  SYNC_BLOCK,
  INIT_AND_PERIODIC,
  SHUTDOWN_TIMEOUT,
  SHUTDOWN_STARVATION,
}
