package org.lflang.tests.runtime;

import org.lflang.tests.TestRegistry.TestCategory;

import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

public abstract class ThreadedBase extends TestBase {

    public final static String RUN_WITH_FOUR_THREADS_DESC = "Description: Run non-concurrent and non-federated tests (threads = 4).";
    
    @Test
    public void runWithFourThreads() {
        printTestHeader(RUN_WITH_FOUR_THREADS_DESC);
        Predicate<TestCategory> categories = (it) -> {
            if (it != TestCategory.CONCURRENT && 
                    it != TestCategory.FEDERATED &&
                    it != TestCategory.EXAMPLE) {
                // Check if running on Windows
                if (isWindows()) {
                    // SERIALIZATION and TARGET tests are currently not 
                    // supported on Windows.
                    if (it != TestCategory.SERIALIZATION &&
                            it != TestCategory.TARGET) {
                        return true;
                    }
                    return false;
                }
                return true;
            }
            return false;
        };
        this.runTestsAndPrintResults(this.target, categories, it -> {
            it.getContext().getArgs().setProperty("threads", "4");
            return true;
        }, true);
    }
}
