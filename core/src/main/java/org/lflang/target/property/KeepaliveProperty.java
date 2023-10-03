package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive to let the execution engine remain active also if there are no more events in the event
 * queue.
 */
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
