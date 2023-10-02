package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

public class SingleFileProjectProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.Rust);
  }

  @Override
  public String name() {
    return "single-file-project";
  }
}
