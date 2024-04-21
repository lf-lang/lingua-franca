package org.lflang.target.property.type;

import org.lflang.target.property.type.ThreadPolicyType.ThreadPolicy;

/** Enumeration of supported thread policies */
public class ThreadPolicyType extends OptionsType<ThreadPolicy> {

  @Override
  protected Class<ThreadPolicy> enumClass() {
    return ThreadPolicy.class;
  }

  /**
   * Enumeration of thread policies to be used to parameterize the thread scheduler.
   *
   * @author Shaokai Lin
   * @author Marten Lohstroh
   */
  public enum ThreadPolicy {
    NORMAL("normal", "LF_SCHED_FAIR"),
    REALTIME_RR("rt-rr", "LF_SCHED_TIMESLICE"),
    REALTIME_FIFO("rt-fifo", "LF_SCHED_PRIORITY");

    /** Alias used in toString method. */
    private final String alias;

    public final String define;

    /** Private constructor for Cmake build types. */
    ThreadPolicy(String alias, String define) {
      this.define = define;
      this.alias = alias;
    }

    /** Return the alias. */
    @Override
    public String toString() {
      return this.alias;
    }

    public static ThreadPolicy getDefault() {
      return ThreadPolicy.NORMAL;
    }
  }
}
