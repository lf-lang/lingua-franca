package org.lflang.target.property.type;

import com.google.common.collect.ImmutableList;
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
    ADAPTIVE(
        false,
        List.of(
            Path.of("scheduler_adaptive.c"),
            Path.of("worker_assignments.h"),
            Path.of("worker_states.h"),
            Path.of("data_collection.h"))),
    GEDF_NP(true), // Global EDF non-preemptive
    GEDF_NP_CI(true); // Global EDF non-preemptive with chain ID

    /** Indicate whether the scheduler prioritizes reactions by deadline. */
    private final boolean prioritizesDeadline;

    /** Relative paths to files required by this scheduler. */
    private final List<Path> relativePaths;

    Scheduler(boolean prioritizesDeadline) {
      this(prioritizesDeadline, null);
    }

    Scheduler(boolean prioritizesDeadline, List<Path> relativePaths) {
      this.prioritizesDeadline = prioritizesDeadline;
      this.relativePaths = relativePaths;
    }

    /** Return true if the scheduler prioritizes reactions by deadline. */
    public boolean prioritizesDeadline() {
      return this.prioritizesDeadline;
    }

    public List<Path> getRelativePaths() {
      return relativePaths != null
          ? ImmutableList.copyOf(relativePaths)
          : List.of(Path.of("scheduler_" + this + ".c"));
    }

    public static Scheduler getDefault() {
      return Scheduler.NP;
    }

    public String getSchedulerCompileDef() {
      return "SCHED_" + this.name();
    }
  }
}
