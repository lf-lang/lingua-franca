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
 * Collection of Zephyr tests for the C target.
 *
 * @author Erling Rennemo Jellum
 * @ingroup Tests
 */
public class CZephyrTest extends TestBase {

  public CZephyrTest() {
    super(Target.C);
  }

  @Test
  public void buildZephyrUnthreadedTests() {
    Assumptions.assumeTrue(isLinux(), "Zephyr tests only run on Linux");
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_ZEPHYR,
        TestCategory.ZEPHYR_UNTHREADED::equals,
        Transformers::noChanges,
        Configurators::makeZephyrCompatibleUnthreaded,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildZephyrBoardsTests() {
    Assumptions.assumeTrue(isLinux(), "Zephyr tests only run on Linux");
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_ZEPHYR,
        TestCategory.ZEPHYR_BOARDS::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildZephyrThreadedTests() {
    Assumptions.assumeTrue(isLinux(), "Zephyr tests only run on Linux");
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_ZEPHYR,
        TestCategory.ZEPHYR_THREADED::equals,
        Transformers::noChanges,
        Configurators::makeZephyrCompatible,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildBasicTests() {
    Assumptions.assumeTrue(isLinux(), "Zephyr tests only run on Linux");
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_BASIC,
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::makeZephyrCompatibleUnthreaded,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildConcurrentTests() {
    Assumptions.assumeTrue(isLinux(), "Zephyr tests only run on Linux");

    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_CONCURRENT,
        TestCategory.CONCURRENT::equals,
        Transformers::noChanges,
        Configurators::makeZephyrCompatible,
        TestLevel.BUILD,
        false);
  }
}
