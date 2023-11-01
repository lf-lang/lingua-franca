package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.GeneratorArguments;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.type.SchedulerType;
import org.lflang.target.property.type.SchedulerType.Scheduler;

/** Directive for specifying the use of a specific runtime scheduler. */
public final class SchedulerProperty extends TargetProperty<Scheduler, SchedulerType> {

  /** Singleton target property instance. */
  public static final SchedulerProperty INSTANCE = new SchedulerProperty();

  private SchedulerProperty() {
    super(new SchedulerType());
  }

  @Override
  public Scheduler initialValue() {
    return Scheduler.getDefault();
  }

  @Override
  public Scheduler fromAst(Element node, MessageReporter reporter) {
    var scheduler = fromString(ASTUtils.elementToSingleString(node), reporter);
    if (scheduler != null) {
      return scheduler;
    } else {
      return Scheduler.getDefault();
    }
  }

  @Override
  protected Scheduler fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public Element toAstElement(Scheduler value) {
    return ASTUtils.toElement(value.toString());
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
        if (!Scheduler.valueOf(schedulerName).prioritizesDeadline()) {
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

  @Override
  public Scheduler value(GeneratorArguments args) {
    return args.scheduler;
  }
}
