package org.lflang.tests.runtime;

import java.util.EnumSet;

import org.junit.jupiter.api.Test;

import org.lflang.Target;
import org.lflang.TargetProperty.SchedulerOption;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 */
public class CSchedulerTest extends TestBase {


    public CSchedulerTest() {
        super(Target.C);
    }

    /**
     * Swap the default runtime scheduler with other supported versions and
     * run all the supported tests. Only run tests for a specific non-default
     * scheduler if specified using a system property (e.g., -Dscheduler=GEDF_NP).
     */
    @Test
    public void runWithNonDefaultSchedulers() {
        EnumSet<TestCategory> categories = EnumSet.of(TestCategory.CONCURRENT,
                                                      TestCategory.MULTIPORT);
        
        // Add federated and docker tests if supported
        if (!isWindows()) {
            categories.add(TestCategory.FEDERATED);
            if (isLinux()) {
                categories.add(TestCategory.DOCKER_FEDERATED);
            }
        }
        var name = System.getProperty("scheduler");

        if (name != null) {
            var option = EnumSet.allOf(SchedulerOption.class).stream()
                                .filter(it -> it.name().equals(name)).findFirst();
            if (option.isPresent()) {
                this.runTest(option.get(), categories);
            } else {
                throw new RuntimeException("Cannot find runtime scheduler called " + name);
            }
        } else {
            for (SchedulerOption scheduler: EnumSet.allOf(SchedulerOption.class)) {
                if (scheduler == SchedulerOption.getDefault()) continue;
                this.runTest(scheduler, categories);
            }
        }
    }

    private void runTest(SchedulerOption scheduler, EnumSet<TestCategory> categories) {
        this.runTestsForTargets(
            Message.DESC_SCHED_SWAPPING + scheduler.toString() +".",
            categories::contains,
            test -> {
                test.getContext().getArgs()
                    .setProperty(
                        "scheduler",
                        scheduler.toString()
                    );
                return Configurators.noChanges(test);
            },
            true
        );
    }
}

