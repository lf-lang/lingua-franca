package org.lflang.tests.runtime;

import java.util.EnumSet;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * Collection of tests for the CCpp target.
 *
 * NOTE: This test does not inherit any tests because it directly extends TestBase.
 *
 * @author Marten Lohstroh <marten@berkeley.edu>
 */
public class CCppTest extends TestBase {

    /**
     * This target selects the C target it has no tests defined for it.
     * Instead, it reconfigures existing C tests to adopt the CCpp target.
     */
    public CCppTest() {
        super(Target.C);
    }

    /**
     * Run C tests with the target CCpp.
     */
    @Test
    public void runAsCCpp() {
        Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);

        EnumSet<TestCategory> categories = EnumSet.allOf(TestCategory.class);
        categories.removeAll(EnumSet.of(
            // Don't need to test examples.
            // If any of them uses CCpp, it will
            // be tested when compileExamples is
            // run.
            TestCategory.EXAMPLE));

        runTestsForTargets(Message.DESC_AS_CCPP, categories::contains,
                           it -> ASTUtils.changeTargetName(it.fileConfig.resource,
                                                           Target.CCPP.getDisplayName()),
                           TestLevel.EXECUTION, true);
    }
}
