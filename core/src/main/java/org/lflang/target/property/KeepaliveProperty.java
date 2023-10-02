package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

public class KeepaliveProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python, Target.TS, Target.Rust);
  }

  @Override
  public String name() {
    return "keepalive";
  }
}
