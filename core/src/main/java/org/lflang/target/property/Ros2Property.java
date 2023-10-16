package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** If true, generate ROS2 specific code. */
public class Ros2Property extends BooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "ros2";
  }
}
