package org.lflang.target.property;

import java.util.List;
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
import org.lflang.target.property.type.StaticSchedulerType;
import org.lflang.target.property.type.StaticSchedulerType.StaticScheduler;
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
    return new SchedulerOptions(Scheduler.getDefault(), null, null);
  }

  @Override
  public SchedulerOptions fromAst(Element node, MessageReporter reporter) {
    // Check if the user passes in a SchedulerType or
    // DictionaryType.SCHEDULER_DICT.
    // If dict, parse from a map.
    Scheduler schedulerType = null;
    StaticScheduler staticSchedulerType = null;
    List<String> mocasinMapping = null;
    String schedulerStr = ASTUtils.elementToSingleString(node);
    if (!schedulerStr.equals("")) {
      schedulerType = Scheduler.fromString(schedulerStr);
      if (schedulerType == Scheduler.STATIC) staticSchedulerType = StaticScheduler.getDefault();
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        SchedulerDictOption option =
            (SchedulerDictOption) DictionaryType.SCHEDULER_DICT.forName(entry.getName());
        if (option != null) {
          switch (option) {
            case TYPE -> {
              // Parse type
              schedulerType =
                  new SchedulerType().forName(ASTUtils.elementToSingleString(entry.getValue()));
            }
            case STATIC_SCHEDULER -> {
              // Parse static scheduler
              staticSchedulerType =
                  new StaticSchedulerType()
                      .forName(ASTUtils.elementToSingleString(entry.getValue()));
              if (staticSchedulerType == null) staticSchedulerType = StaticScheduler.getDefault();
            }
            case MOCASIN_MAPPING -> {
              // Parse mocasin mapping
              mocasinMapping = ASTUtils.elementToListOfStrings(entry.getValue());
            }
          }
        }
      }
    }
    return new SchedulerOptions(schedulerType, staticSchedulerType, mocasinMapping);
  }

  @Override
  protected SchedulerOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement(SchedulerOptions value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "scheduler";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var scheduler = config.get(this);
    if (scheduler.type != null && !scheduler.type.prioritizesDeadline()) {
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
  public record SchedulerOptions(
      Scheduler type, StaticScheduler staticScheduler, List<String> mocasinMapping) {
    public SchedulerOptions(Scheduler type) {
      this(type, null, null);
    }

    public SchedulerOptions update(Scheduler newType) {
      return new SchedulerOptions(newType, this.staticScheduler, this.mocasinMapping);
    }

    public SchedulerOptions update(StaticScheduler newStaticScheduler) {
      return new SchedulerOptions(this.type, newStaticScheduler, this.mocasinMapping);
    }

    public SchedulerOptions update(List<String> newMocasinMapping) {
      return new SchedulerOptions(this.type, this.staticScheduler, newMocasinMapping);
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
