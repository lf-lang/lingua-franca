package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import org.lflang.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.RuntimeTest;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * Collection of Zephyr tests for the C target.
 *
 * @author Erling Rennemo Jellum <erling.r.jellum@ntnu.no>
 */
public class CArduinoTest extends RuntimeTest {

    public CArduinoTest() {
        super(Target.C);
    }

    @Test
    public void runArduinoTests() {
        Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
        super.runTestsFor(List.of(Target.C),
            Message.DESC_ARDUINO,
            TestCategory.ARDUINO::equals, Configurators::noChanges,
            false);
    }
}
