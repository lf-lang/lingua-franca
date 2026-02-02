package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.SchedulerProperty.SchedulerOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.SchedulerType;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.target.property.type.StaticMapperType;
import org.lflang.target.property.type.StaticMapperType.StaticMapper;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/** Directive for specifying the use of a specific runtime scheduler. */
public final class SchedulerProperty extends TargetProperty<SchedulerOptions, UnionType> {

  /** Singleton target property instance. */
  public static final SchedulerProperty INSTANCE = new SchedulerProperty();

  private SchedulerProperty() {
    super(UnionType.SCHEDULER_UNION_OR_DICTIONARY);
  }

  @Override
  public SchedulerOptions initialValue() {
    return new SchedulerOptions(Scheduler.getDefault(), null);
  }

  @Override
  public SchedulerOptions fromAst(Element node, MessageReporter reporter) {
    Scheduler schedulerType = null;
    StaticMapper staticMapperType = null;
    String schedulerStr = ASTUtils.elementToSingleString(node);
    if (!schedulerStr.equals("")) {
      schedulerType = new SchedulerType().forName(schedulerStr);
      if (schedulerType == null) {
        reporter.nowhere().error("Invalid scheduler: " + schedulerStr);
        schedulerType = Scheduler.getDefault();
      }
      if (schedulerType == Scheduler.STATIC) staticMapperType = StaticMapper.getDefault();
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        SchedulerDictOption option =
            (SchedulerDictOption) DictionaryType.SCHEDULER_DICT.forName(entry.getName());
        if (option != null) {
          switch (option) {
            case TYPE -> {
              schedulerType =
                  new SchedulerType().forName(ASTUtils.elementToSingleString(entry.getValue()));
            }
            case MAPPER -> {
              staticMapperType =
                  new StaticMapperType().forName(ASTUtils.elementToSingleString(entry.getValue()));
              if (staticMapperType == null) staticMapperType = StaticMapper.getDefault();
            }
          }
        }
      }
    }
    return new SchedulerOptions(schedulerType, staticMapperType);
  }

  @Override
  protected SchedulerOptions fromString(String string, MessageReporter reporter) {
    Scheduler s = new SchedulerType().forName(string);
    if (s == null) return initialValue();
    return new SchedulerOptions(s);
  }

  @Override
  public Element toAstElement(SchedulerOptions value) {
    return ASTUtils.toElement(
        value.type != null ? value.type.name() : Scheduler.getDefault().name());
  }

  @Override
  public String name() {
    return "scheduler";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var scheduler = config.get(this);
    if (scheduler.type != null && !scheduler.type.prioritizesDeadline()) {
      if (ASTUtils.getAllReactors(config.getMainResource()).stream()
          .anyMatch(
              reactor ->
                  ASTUtils.allReactions(reactor).stream()
                      .anyMatch(reaction -> reaction.getDeadline() != null))) {
        reporter
            .nowhere()
            .warning(
                "This program contains deadlines, but the chosen "
                    + scheduler
                    + " scheduler does not prioritize reaction execution "
                    + "based on deadlines. This might result in a sub-optimal "
                    + "scheduling.");
      }
    }
  }

  /** Settings related to Scheduler Options. */
  public record SchedulerOptions(Scheduler type, StaticMapper staticMapper) {
    public SchedulerOptions(Scheduler type) {
      this(type, null);
    }

    public SchedulerOptions update(Scheduler newType) {
      return new SchedulerOptions(newType, this.staticMapper);
    }

    public SchedulerOptions update(StaticMapper newStaticMapper) {
      return new SchedulerOptions(this.type, newStaticMapper);
    }
  }

  /** Scheduler dictionary options. */
  public enum SchedulerDictOption implements DictionaryElement {
    TYPE("type", new SchedulerType()),
    MAPPER("mapper", new StaticMapperType());

    public final TargetPropertyType type;

    private final String description;

    private SchedulerDictOption(String alias, TargetPropertyType type) {
      this.description = alias;
      this.type = type;
    }

    @Override
    public String toString() {
      return this.description;
    }

    public TargetPropertyType getType() {
      return this.type;
    }
  }
}
