package org.lflang.tests.runtime;

import java.util.EnumSet;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SchedulerProperty.SchedulerOptions;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;

/** */
public class CSchedulerTest extends TestBase {

  public CSchedulerTest() {
    super(Target.C);
  }

  /**
   * Swap the default runtime scheduler with other supported versions and run all the supported
   * tests. Only run tests for a specific non-default scheduler if specified using a system property
   * (e.g., -Dscheduler=GEDF_NP).
   */
  @Test
  public void runWithNonDefaultSchedulers() {
    EnumSet<TestCategory> categories = EnumSet.of(TestCategory.CONCURRENT, TestCategory.MULTIPORT);

    // Add federated and docker tests if supported
    if (!isWindows()) {
      categories.add(TestCategory.FEDERATED);
      if (isLinux()) {
        categories.add(TestCategory.DOCKER_FEDERATED);
      }
    }
    var name = System.getProperty("scheduler");

    if (name != null) {
      var option =
          EnumSet.allOf(Scheduler.class).stream().filter(it -> it.name().equals(name)).findFirst();
      if (option.isPresent()) {
        this.runTest(option.get(), categories);
      } else {
        throw new RuntimeException("Cannot find runtime scheduler called " + name);
      }
    } else {
      for (Scheduler scheduler : EnumSet.allOf(Scheduler.class)) {
        if (scheduler == Scheduler.getDefault()) continue;
        this.runTest(scheduler, categories);
      }
    }
  }

  private void runTest(Scheduler scheduler, EnumSet<TestCategory> categories) {
    this.runTestsForTargets(
        Message.DESC_SCHED_SWAPPING + scheduler.toString() + ".",
        categories::contains,
        Transformers::noChanges,
        config -> {
          SchedulerProperty.INSTANCE.override(config, new SchedulerOptions(scheduler));
          return true;
        },
        TestLevel.EXECUTION,
        true);
  }
}
