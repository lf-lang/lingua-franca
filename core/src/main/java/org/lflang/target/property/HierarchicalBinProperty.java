package org.lflang.target.property;

import org.lflang.generator.GeneratorArguments;

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
  public Boolean value(GeneratorArguments args) {
    return args.hierarchicalBin;
  }
}
