package org.lflang.tests.runtime;

import org.lflang.Target;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry.TestCategory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

//FIXME: Move this into TestBase because this design is not very extensible when there is no multilpe inheritance.
public abstract class ThreadedBase extends TestBase {

    public static final String RUN_WITH_FOUR_THREADS_DESC = "Description: Run non-concurrent and non-federated tests (threads = 4).";

    protected static final Function<LFTest, Boolean> useFourThreads = t -> {
        t.getContext().getArgs().setProperty("threads", "4");
        return true;
    };

    protected ThreadedBase(Target first) {
        super(first);
    }

    protected ThreadedBase(List<Target> targets) {
        super(targets);
    }

    @Test
    public void runWithFourThreads() {
        this.runTestsForTargets(RUN_WITH_FOUR_THREADS_DESC,
                defaultExcludedCategories(), useFourThreads,
                TestLevel.EXECUTION, true);
    }

    /**
     * Give a handy lambda function that, given a test category,
     * will return true if it is not one of the default excluded 
     * categories.
     */
    public static Predicate<TestCategory> defaultExcludedCategories() {
        return category -> {
            if (category != TestCategory.CONCURRENT && category != TestCategory.FEDERATED &&
                    category != TestCategory.EXAMPLE) {
                // Check if running on Windows
                if (isWindows()) {
                    // SERIALIZATION and TARGET tests are currently not
                    // supported on Windows.
                    return category != TestCategory.SERIALIZATION &&
                        category != TestCategory.TARGET;
                }
                return true;
            }
            return false;
        };
    }


}
