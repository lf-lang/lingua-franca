package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.SchedulerType;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.target.property.type.StaticSchedulerType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/** Directive for specifying the use of a specific runtime scheduler. */
public final class SchedulerProperty extends TargetProperty<Scheduler, UnionType> {

  /** Singleton target property instance. */
  public static final SchedulerProperty INSTANCE = new SchedulerProperty();

  private SchedulerProperty() {
    super(UnionType.SCHEDULER_UNION_OR_DICTIONARY);
  }

  @Override
  public Scheduler initialValue() {
    return Scheduler.getDefault();
  }

  @Override
  public Scheduler fromAst(Element node, MessageReporter reporter) {
    String schedulerStr = ASTUtils.elementToSingleString(node);
    // Check if the user passes in a SchedulerType or
    // DictionaryType.SCHEDULER_DICT.
    // If dict, get the scheduler from the "type" field.
    if (schedulerStr.equals("")) {
      var strMap = ASTUtils.elementToStringMaps(node);
      schedulerStr = strMap.get("type");
    }
    var scheduler = fromString(schedulerStr, reporter);
    if (scheduler != null) {
      return scheduler;
    } else {
      return Scheduler.getDefault();
    }
  }

  @Override
  protected Scheduler fromString(String string, MessageReporter reporter) {
    return Scheduler.fromString(string);
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

  /**
   * Scheduler dictionary options.
   *
   * @author Shaokai Lin
   */
  public enum SchedulerDictOption implements DictionaryElement {
    TYPE("type", new SchedulerType()),
    STATIC_SCHEDULER("static-scheduler", new StaticSchedulerType()),
    MOCASIN_MAPPING("mocasin-mapping", UnionType.FILE_OR_FILE_ARRAY);

    public final TargetPropertyType type;

    private final String description;

    private SchedulerDictOption(String alias, TargetPropertyType type) {
      this.description = alias;
      this.type = type;
    }

    /** Return the description of this dictionary element. */
    @Override
    public String toString() {
      return this.description;
    }

    /** Return the type associated with this dictionary element. */
    public TargetPropertyType getType() {
      return this.type;
    }
  }
}
