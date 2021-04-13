package org.icyphy.tests.runtime;

import org.icyphy.tests.TestRegistry.TestCategory;
import org.junit.jupiter.api.Test;

public abstract class ThreadedBase extends TestBase {

    public final static String RUN_WITH_FOUR_THREADS_DESC = "Description: Run non-concurrent and non-federated tests (threads = 4).";
    
    @Test
    public void runWithFourThreads() {
        printTestHeader(RUN_WITH_FOUR_THREADS_DESC);
        this.runTestsAndPrintResults(this.target,
                it -> (it != TestCategory.CONCURRENT &&
                        it != TestCategory.FEDERATED && 
                        it != TestCategory.EXAMPLE),
                it -> {
                    it.getContext().getArgs().setProperty("threads", "4");
                    return true;
                }, true);
    }
}
