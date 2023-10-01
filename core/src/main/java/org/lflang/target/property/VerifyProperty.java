package org.lflang.target.property.type;

import java.util.List;
import org.lflang.Target;
import org.lflang.target.property.AbstractBooleanProperty;

public class VerifyProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C);
  }
}
