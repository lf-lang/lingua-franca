package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;

/**
 * Collection of tests for the FlexPRET target.
 *
 * @ingroup Tests
 */
public class CFlexPRETTest extends TestBase {

  public CFlexPRETTest() {
    super(Target.C);
  }

  @Test
  public void buildFlexPRETConcurrent() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build concurrent tests for FlexPRET.",
        TestCategory.CONCURRENT::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatible,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildFlexPRETBasicTestsUnthreaded() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build basic tests for FlexPRET in single threaded mode.",
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatibleUnthreaded,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildFlexPRETBasicTests() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build basic tests for FlexPRET.",
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatible,
        TestLevel.BUILD,
        false);
  }
}
