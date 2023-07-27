package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry;

public class CVerifierTest extends TestBase {
  protected CVerifierTest() {
    super(Target.C);
  }

  @Test
  public void runVerifierTests() {
    Assumptions.assumeTrue(isLinux() || isMac(), "Verifier tests only run on Linux or macOS");

    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_VERIFIER,
        TestRegistry.TestCategory.VERIFIER::equals,
        test -> {
          test.getContext().getTargetConfig().verify = true;
          return true;
        },
        TestLevel.BUILD,
        false);
  }
}
