package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorArguments;
import org.lflang.target.TargetConfig;

/**
 * Whether the bin directory should have a flat or hierarchical organization. It is flat by default.
 */
public final class HierarchicalBinProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final HierarchicalBinProperty INSTANCE = new HierarchicalBinProperty();

  private HierarchicalBinProperty() {
    super();
  }

  @Override
  public String name() {
    return "hierarchical-bin";
  }

  @Override
  public void update(TargetConfig config, GeneratorArguments args, MessageReporter reporter) {
    if (args.hierarchicalBin != null) {
      config.set(this, args.hierarchicalBin);
    } else if (args.jsonObject != null) {
      config.set(this, fromJSON(args.jsonObject, reporter));
    }
  }

  @Override
  public Boolean value(GeneratorArguments args) {
    return args.hierarchicalBin;
  }
}
