package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SchedulerProperty.SchedulerOptions;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.target.property.type.StaticSchedulerType;
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
        config -> {
          // Execute all static tests using STATIC and LB.
          // FIXME: How to respect the config specified in the LF code?
          SchedulerProperty.INSTANCE.override(
              config,
              new SchedulerOptions(Scheduler.STATIC)
                  .update(StaticSchedulerType.StaticScheduler.LB));
          // Keep the logging level at INFO because logs from the long
          // running tests (e.g., RaceConditionCheck.lf) could overflow
          // the buffer and stall the process. 
          LoggingProperty.INSTANCE.override(config, LogLevel.INFO);
          return true;
        },
        TestLevel.EXECUTION,
        false);
  }
}
