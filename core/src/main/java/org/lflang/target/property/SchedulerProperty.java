package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
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
  public void validate(TargetConfig config, MessageReporter reporter) {
    var scheduler = config.get(this);
    if (!scheduler.prioritizesDeadline()) {
      // Check if a deadline is assigned to any reaction
      // Filter reactors that contain at least one reaction that
      // has a deadline handler.
      if (ASTUtils.getAllReactors(config.getMainResource()).stream()
          .anyMatch(
              // Filter reactors that contain at least one reaction that
              // has a deadline handler.
              reactor ->
                  ASTUtils.allReactions(reactor).stream()
                      .anyMatch(reaction -> reaction.getDeadline() != null))) {
        reporter
            .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
            .warning(
                "This program contains deadlines, but the chosen "
                    + scheduler
                    + " scheduler does not prioritize reaction execution "
                    + "based on deadlines. This might result in a sub-optimal "
                    + "scheduling.");
      }
    }
  }
}
