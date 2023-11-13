package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.property.VerifyProperty;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.Transformers;

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
        Transformers::noChanges,
        config -> {
          VerifyProperty.INSTANCE.override(config, true);
          return true;
        },
        TestLevel.BUILD,
        false);
  }
}
