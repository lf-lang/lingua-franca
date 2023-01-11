package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.lf.Element;
import org.lflang.lf.LfFactory;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * Run C++ tests using the ROS2 platform.
 *
 * NOTE: This test does not inherit any tests because it directly extends TestBase.
 *
 * @author Christian Menard
 */
public class CppRos2Test extends TestBase {

    public CppRos2Test() { super(Target.CPP); }

    /**
     * Run C++ tests with the ros2 target property set
     */
    @Test
    public void runWithRos2() {
        Assumptions.assumeTrue(isLinux(), "Only supported on Linux");
        Element trueLiteral = LfFactory.eINSTANCE.createElement();
        trueLiteral.setLiteral("true");
        runTestsForTargets(Message.DESC_ROS2, it -> true,
                           it -> ASTUtils.addTargetProperty(it.getFileConfig().resource, "ros2", trueLiteral),
                           true);
    }
}
