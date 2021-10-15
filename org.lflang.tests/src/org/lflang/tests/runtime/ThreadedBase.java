package org.lflang.tests.runtime;

import org.lflang.Target;

import java.util.List;

import org.junit.jupiter.api.Test;

//FIXME: Move this into TestBase because this design is not very extensible when there is no multilpe inheritance.
public abstract class ThreadedBase extends TestBase {

    public static final String RUN_WITH_FOUR_THREADS_DESC = "Description: Run non-concurrent and non-federated tests (threads = 4).";

    protected ThreadedBase(Target first) {
        super(first);
    }

    protected ThreadedBase(List<Target> targets) {
        super(targets);
    }

    @Test
    public void runWithFourThreads() {
        this.runTestsForTargets(RUN_WITH_FOUR_THREADS_DESC,
                                ConfigurationPredicates::defaultCategoryExclusion,
                                ConfigurationPredicates::useFourThreads,
                                TestLevel.EXECUTION,
                                true);
    }


}
