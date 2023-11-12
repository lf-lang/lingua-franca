package org.lflang.target.property;

/** If true, generate ROS2 specific code. */
public final class Ros2Property extends BooleanProperty {

  /** Singleton target property instance. */
  public static final Ros2Property INSTANCE = new Ros2Property();

  private Ros2Property() {
    super();
  }

  @Override
  public String name() {
    return "ros2";
  }
}
