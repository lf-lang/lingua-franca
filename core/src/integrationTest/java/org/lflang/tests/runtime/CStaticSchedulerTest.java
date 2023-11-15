package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.Transformers;

public class CStaticSchedulerTest extends TestBase {
  protected CStaticSchedulerTest() {
    super(Target.C);
  }

  @Test
  public void runStaticSchedulerTests() {
    Assumptions.assumeTrue(
        isLinux() || isMac(), "Static scheduler tests only run on Linux or macOS");

    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_STATIC_SCHEDULER,
        TestRegistry.TestCategory.STATIC_SCHEDULER::equals,
        Transformers::noChanges,
        test -> {
          test.getContext().getTargetConfig().schedulerType = Scheduler.STATIC;
          return true;
        },
        TestLevel.EXECUTION,
        false);
  }
}
