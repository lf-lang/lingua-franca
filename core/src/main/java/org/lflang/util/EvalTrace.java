package org.lflang.util;

/**
 * Optional span markers for deployment-time experiments.
 *
 * <p>When the environment variable {@code LF_EVAL_TRACE} is set, {@link #begin} and {@link #end}
 * print one line {@code LFEVAL_TRACE <begin|end> <operation> <nanoTime>} to stderr. The experiment
 * controller timestamps each line on arrival with its own monotonic clock. When the variable is not
 * set, these calls do nothing.
 */
public final class EvalTrace {
  private static final boolean ENABLED = System.getenv("LF_EVAL_TRACE") != null;

  private EvalTrace() {}

  public static void begin(String operation) {
    emit("begin", operation);
  }

  public static void end(String operation) {
    emit("end", operation);
  }

  private static void emit(String kind, String operation) {
    if (!ENABLED) return;
    synchronized (System.err) {
      System.err.println("LFEVAL_TRACE " + kind + " " + operation + " " + System.nanoTime());
      System.err.flush();
    }
  }
}
