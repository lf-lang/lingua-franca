package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * If true, configure the execution environment to keep executing if there are no more events on the
 * event queue. The default is false.
 */
public final class KeepaliveProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final KeepaliveProperty INSTANCE = new KeepaliveProperty();

  private KeepaliveProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python, Target.TS, Target.Rust);
  }

  @Override
  public String name() {
    return "keepalive";
  }
}
