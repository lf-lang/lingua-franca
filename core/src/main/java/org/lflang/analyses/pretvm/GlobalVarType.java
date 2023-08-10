package org.lflang.analyses.pretvm;

/**
 * Types of global variables used by the PRET VM Variable types prefixed by GLOBAL_ are accessible
 * by all workers. Variable types prefixed by WORKER_ mean that there are arrays of these variables
 * such that each worker gets its dedicated variable. For example, WORKER_COUNTER means that there
 * is an array of counter variables, one for each worker. A worker cannot modify another worker's
 * counter.
 */
public enum GlobalVarType {
  GLOBAL_TIMEOUT,
  WORKER_COUNTER,
  WORKER_OFFSET,
  EXTERN_START_TIME,
}
