package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

public class CompilerProperty extends AbstractStringConfig {

  @Override
  public List<Target> supportedTargets() {
    return Target.ALL;
  }
}
