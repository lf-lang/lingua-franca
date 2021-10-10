package org.lflang.tests.runtime;

import org.lflang.tests.TestRegistry.TestCategory;

import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

public abstract class ThreadedBase extends TestBase {

    public final static String RUN_WITH_FOUR_THREADS_DESC = "Description: Run non-concurrent and non-federated tests (threads = 4).";
    
    @Test
    public void runWithFourThreads() {
        printTestHeader(RUN_WITH_FOUR_THREADS_DESC);
        this.runTestsAndPrintResults(this.target, defaultExcludedCategories(), it -> {
            it.getContext().getArgs().setProperty("threads", "4");
            return true;
        }, true);
    }
    
    /**
     * Give a handy lambda function that, given a test category,
     * will return true if it is not one of the default excluded 
     * categories.
     */
    public final Predicate<TestCategory> defaultExcludedCategories() {
        return (category) -> {
            if (category != TestCategory.CONCURRENT && category != TestCategory.FEDERATED &&
                    category != TestCategory.EXAMPLE) {
                // Check if running on Windows
                if (isWindows()) {
                    // SERIALIZATION and TARGET tests are currently not
                    // supported on Windows.
                    if (category != TestCategory.SERIALIZATION &&
                            category != TestCategory.TARGET) {
                        return true;
                    }
                    return false;
                }
                return true;
            }
            return false;
        };
    }
}
