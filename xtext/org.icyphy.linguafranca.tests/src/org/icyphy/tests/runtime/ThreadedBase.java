package org.icyphy.tests.runtime;

import org.icyphy.tests.TestRegistry.TestCategory;
import org.icyphy.tests.runtime.TestBase;
import org.junit.jupiter.api.Test;

public abstract class ThreadedBase extends TestBase {

    @Test
    public void runSingleThreadedTestsAsThreaded() {
        printTestHeader(
                "Description: Run non-concurrent and non-federated test (threads = 4).");
        this.runTestsAndPrintResults(this.target,
                it -> (it != TestCategory.CONCURRENT &&
                        it != TestCategory.FEDERATED && 
                        it != TestCategory.EXAMPLE),
                it -> {
                    it.properties.setProperty("threads", "4");
                    return true;
                });
    }
}
