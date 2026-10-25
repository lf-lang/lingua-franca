package org.lflang.target.property.type;

import java.nio.file.Path;
import java.util.List;
import org.lflang.target.property.type.SchedulerType.Scheduler;

public class SchedulerType extends OptionsType<Scheduler> {

  @Override
  protected Class<Scheduler> enumClass() {
    return Scheduler.class;
  }

  /**
   * Supported schedulers.
   *
   * @author Soroush Bateni
   */
  public enum Scheduler {
    NP(false), // Non-preemptive
    GEDF_NP(true); // Global EDF non-preemptive

    /** Indicate whether the scheduler prioritizes reactions by deadline. */
    private final boolean prioritizesDeadline;

    Scheduler(boolean prioritizesDeadline) {
      this.prioritizesDeadline = prioritizesDeadline;
    }

    /** Return true if the scheduler prioritizes reactions by deadline. */
    public boolean prioritizesDeadline() {
      return this.prioritizesDeadline;
    }

    public List<Path> getRelativePaths() {
      return List.of(Path.of("scheduler_" + this + ".c"));
    }

    public static Scheduler getDefault() {
      return Scheduler.NP;
    }

    public String getSchedulerCompileDef() {
      return "SCHED_" + this.name();
    }
  }
}
