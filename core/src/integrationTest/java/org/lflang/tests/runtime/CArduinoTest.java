package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * Collection of Arduino tests for the C target.
 *
 * @author Anirudh Rengarajan <arengarajan@berkeley.edu>
 */
public class CArduinoTest extends TestBase {

  public CArduinoTest() {
    super(Target.C);
  }

  @Test
  public void buildArduinoTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_ARDUINO,
        TestCategory.ARDUINO::equals,
        Configurators::noChanges,
        TestLevel.BUILD,
        false);
  }
}
