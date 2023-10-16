package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/**
 * Whether the bin directory should have a flat or hierarchical organization. It is flat by default.
 */
public class HierarchicalBinProperty extends BooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public String name() {
    return "hierarchical-bin";
  }
}
