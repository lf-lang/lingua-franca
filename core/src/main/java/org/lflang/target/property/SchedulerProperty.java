package org.lflang.target.property;

import com.google.common.collect.ImmutableList;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.SchedulerProperty.SchedulerOption;
import org.lflang.target.property.type.UnionType;

public class SchedulerProperty extends AbstractTargetProperty<SchedulerOption> {

  public SchedulerProperty() {
    super(UnionType.SCHEDULER_UNION);
  }

  @Override
  public SchedulerOption initialValue() {
    return SchedulerOption.getDefault();
  }

  @Override
  public SchedulerOption fromAst(Element node, MessageReporter reporter) {
    var scheduler = fromString(ASTUtils.elementToSingleString(node), reporter);
    if (scheduler != null) {
      return scheduler;
    } else {
      return SchedulerOption.getDefault();
    }
  }

  @Override
  protected SchedulerOption fromString(String string, MessageReporter reporter) {
    return (SchedulerOption) UnionType.SCHEDULER_UNION.forName(string);
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get().toString());
  }

  @Override
  public String name() {
    return "scheduler";
  }

  @Override
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {
    if (pair != null) {
      String schedulerName = ASTUtils.elementToSingleString(pair.getValue());
      try {
        if (!SchedulerOption.valueOf(schedulerName).prioritizesDeadline()) {
          // Check if a deadline is assigned to any reaction
          // Filter reactors that contain at least one reaction that
          // has a deadline handler.
          if (ast.getReactors().stream()
              .anyMatch(
                  // Filter reactors that contain at least one reaction that
                  // has a deadline handler.
                  reactor ->
                      ASTUtils.allReactions(reactor).stream()
                          .anyMatch(reaction -> reaction.getDeadline() != null))) {
            reporter
                .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
                .warning(
                    "This program contains deadlines, but the chosen "
                        + schedulerName
                        + " scheduler does not prioritize reaction execution "
                        + "based on deadlines. This might result in a sub-optimal "
                        + "scheduling.");
          }
        }
      } catch (IllegalArgumentException e) {
        // the given scheduler is invalid, but this is already checked by
        // checkTargetProperties
      }
    }
  }

  /**
   * Supported schedulers.
   *
   * @author Soroush Bateni
   */
  public enum SchedulerOption {
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

    SchedulerOption(boolean prioritizesDeadline) {
      this(prioritizesDeadline, null);
    }

    SchedulerOption(boolean prioritizesDeadline, List<Path> relativePaths) {
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

    public static SchedulerOption getDefault() {
      return NP;
    }
  }
}
