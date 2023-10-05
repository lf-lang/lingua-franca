package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** If true, generate ROS2 specific code. */
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
