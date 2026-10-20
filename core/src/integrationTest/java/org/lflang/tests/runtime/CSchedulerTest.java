package org.lflang.tests.runtime;

import java.util.EnumSet;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;

/**
 * Collection of tests for the C schedulers.
 *
 * @ingroup Tests
 */
public class CSchedulerTest extends TestBase {

  public CSchedulerTest() {
    super(Target.C);
  }

  /**
   * Swap the default runtime scheduler with other supported versions and run the supported tests.
   * Only run tests for a specific non-default scheduler if specified using a system property (e.g.,
   * -Dscheduler=GEDF_NP).
   *
   * <p>ASYNC tests are included only for {@link Scheduler#GEDF_NP}; the adaptive scheduler does not
   * support the async/await and dynamic worker-thread APIs.
   */
  @Test
  public void runWithNonDefaultSchedulers() {
    // Only treat a non-blank -Dscheduler=... as an override. Gradle used to forward a
    // null system property as the literal string "null", which must not select a scheduler.
    var name = System.getProperty("scheduler");
    if (name != null && !name.isBlank() && !name.equals("null")) {
      var option =
          EnumSet.allOf(Scheduler.class).stream().filter(it -> it.name().equals(name)).findFirst();
      if (option.isPresent()) {
        this.runTest(option.get());
      } else {
        throw new RuntimeException("Cannot find runtime scheduler called " + name);
      }
    } else {
      for (Scheduler scheduler : EnumSet.allOf(Scheduler.class)) {
        if (scheduler == Scheduler.getDefault()) continue;
        this.runTest(scheduler);
      }
    }
  }

  /** Categories to re-run under the given non-default scheduler. */
  private EnumSet<TestCategory> categoriesFor(Scheduler scheduler) {
    EnumSet<TestCategory> categories = EnumSet.of(TestCategory.CONCURRENT, TestCategory.MULTIPORT);

    // Add federated and docker tests if supported
    if (!isWindows()) {
      categories.add(TestCategory.FEDERATED);
      if (isLinux()) {
        categories.add(TestCategory.DOCKER_FEDERATED);
      }
    }
    // ASYNC requires NP or GEDF_NP; skip ADAPTIVE (and other schedulers that lack support).
    if (scheduler == Scheduler.GEDF_NP) {
      categories.add(TestCategory.ASYNC);
    }
    return categories;
  }

  private void runTest(Scheduler scheduler) {
    this.runTestsForTargets(
        Message.DESC_SCHED_SWAPPING + scheduler.toString() + ".",
        categoriesFor(scheduler)::contains,
        Transformers::noChanges,
        config -> {
          SchedulerProperty.INSTANCE.override(config, scheduler);
          return true;
        },
        TestLevel.EXECUTION,
        true);
  }
}
