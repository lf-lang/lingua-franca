package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.lf.Element;
import org.lflang.lf.LfFactory;
import org.lflang.target.Target;
import org.lflang.target.property.Ros2Property;
import org.lflang.tests.TestBase;
import org.lflang.tests.Transformers;

/**
 * Run C++ tests using the ROS2 platform.
 *
 * <p>NOTE: This test does not inherit any tests because it directly extends TestBase.
 *
 * @author Christian Menard
 */
public class CppRos2Test extends TestBase {

  public CppRos2Test() {
    super(Target.CPP);
  }

  /** Run C++ tests with the ros2 target property set */
  @Test
  public void runWithRos2() {
    Assumptions.assumeTrue(isLinux(), "Only supported on Linux");
    Element trueLiteral = LfFactory.eINSTANCE.createElement();
    trueLiteral.setLiteral("true");
    runTestsForTargets(
        Message.DESC_ROS2,
        it -> true,
        Transformers::noChanges,
        config -> {
          Ros2Property.INSTANCE.override(config, true);
          return true;
        },
        TestLevel.EXECUTION,
        true);
  }
}
