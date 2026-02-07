package org.lflang.target.property.type;

import org.lflang.target.property.type.ThreadPolicyType.ThreadPolicy;

/** Enumeration of supported thread scheduling policies. */
public class ThreadPolicyType extends OptionsType<ThreadPolicy> {

  @Override
  protected Class<ThreadPolicy> enumClass() {
    return ThreadPolicy.class;
  }

  /**
   * Enumeration of thread policies to be used to parameterize the thread scheduler.
   *
   * <p>These policies correspond to the scheduling policies available in the platform API:
   * <ul>
   *   <li>LF_SCHED_FAIR - Non real-time scheduling (SCHED_OTHER on Linux)</li>
   *   <li>LF_SCHED_TIMESLICE - Real-time, time-slicing priority-based (SCHED_RR on Linux)</li>
   *   <li>LF_SCHED_PRIORITY - Real-time, priority-only based (SCHED_FIFO on Linux)</li>
   * </ul>
   *
   * @author Shaokai Lin
   * @author Marten Lohstroh
   */
  public enum ThreadPolicy {
    LF_SCHED_FAIR("normal", "LF_SCHED_FAIR"),
    LF_SCHED_TIMESLICE("rt-rr", "LF_SCHED_TIMESLICE"),
    LF_SCHED_PRIORITY("rt-fifo", "LF_SCHED_PRIORITY");

    /** Alias used in toString method. */
    private final String alias;

    /** The C define to use when generating code. */
    public final String cDefine;

    /** Private constructor for thread policies. */
    ThreadPolicy(String alias, String cDefine) {
      this.cDefine = cDefine;
      this.alias = alias;
    }

    /** Return the alias (used in LF files). */
    @Override
    public String toString() {
      return this.alias;
    }

    /** Return the default thread policy. */
    public static ThreadPolicy getDefault() {
      return ThreadPolicy.LF_SCHED_FAIR;
    }
  }
}

