package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to specify that ROS2 specific code is generated. */
public class Ros2Property extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "ros2";
  }
}
