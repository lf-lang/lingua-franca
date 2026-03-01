package org.lflang.generator.c;

import org.lflang.generator.ReactorInstance;

/**
 * An CEnclaveInstance object is associated with each enclave. Here we store information about how
 * many timers, shutdown reactions etc. while code-generating. Each object is tied to a
 * ReactorInstance which is the top-level reactor within the enclave.
 *
 * @ingroup Generator
 */
public class CEnclaveInstance {
  public int numIsPresentFields = 0;
  public int numStartupReactions = 0;
  public int numShutdownReactions = 0;
  public int numTimerTriggers = 0;
  public int numResetReactions = 0;
  public int numWorkers = 0;
  public int numModalReactors = 0;
  public int numModalResetStates = 0;
  public int numWatchdogs = 0;

  public String getId() {
    return instance.uniqueID();
  }

  public ReactorInstance getReactorInstance() {
    return instance;
  }

  public String traceFileName = null;
  private ReactorInstance instance;

  public CEnclaveInstance(ReactorInstance inst, int numWorkers) {
    instance = inst;
    this.numWorkers = numWorkers;
  }
}
