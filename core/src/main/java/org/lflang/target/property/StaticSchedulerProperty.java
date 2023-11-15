package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.StaticSchedulerType;
import org.lflang.target.property.type.StaticSchedulerType.StaticScheduler;

/** Directive for specifying the use of a specific runtime scheduler. */
public final class StaticSchedulerProperty extends TargetProperty<StaticScheduler, StaticSchedulerType> {

  /** Singleton target property instance. */
  public static final StaticSchedulerProperty INSTANCE = new StaticSchedulerProperty();

  private StaticSchedulerProperty() {
    super(new StaticSchedulerType());
  }

  @Override
  public StaticScheduler initialValue() {
    return StaticScheduler.getDefault();
  }

  @Override
  public StaticScheduler fromAst(Element node, MessageReporter reporter) {
    var scheduler = fromString(ASTUtils.elementToSingleString(node), reporter);
    if (scheduler != null) {
      return scheduler;
    } else {
      return StaticScheduler.getDefault();
    }
  }

  @Override
  protected StaticScheduler fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public Element toAstElement(StaticScheduler value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "static-scheduler";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    // Do nothing for now.
  }
}
