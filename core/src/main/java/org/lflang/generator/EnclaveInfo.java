package org.lflang.generator;

public class EnclaveInfo {
  public int numIsPresentFields = 0;
  public int numStartupReactions = 0;
  public int numShutdownReactions = 0;
  public int numTimerTriggers = 0;
  public int numResetReactions = 0;
  public int numWorkers = 1;
  public int numModalReactors = 0;
  public int numModalResetStates = 0;
  public int numWatchdogs = 0;

  public String traceFileName = null;
  private ReactorInstance instance;

  public EnclaveInfo(ReactorInstance inst) {
    instance = inst;
  }
}
