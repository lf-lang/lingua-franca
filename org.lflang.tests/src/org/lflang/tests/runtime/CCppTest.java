package org.lflang.tests.runtime;

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
 * @author Marten Lohstroh
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
        runTestsForTargets(Message.DESC_AS_CCPP, CCppTest::isExcludedFromCCpp,
                           it -> ASTUtils.changeTargetName(it.getFileConfig().resource,
                                                           Target.CCPP.getDisplayName()),
                           true);
    }

    /**
     * Exclusion function for runAsCCpp test
     */
    private static boolean isExcludedFromCCpp(TestCategory category) {
        boolean excluded = category == TestCategory.SERIALIZATION;
        excluded |= isWindows() && (category == TestCategory.DOCKER_FEDERATED);
        excluded |= isMac() && (category == TestCategory.DOCKER_FEDERATED || category == TestCategory.DOCKER);
        excluded |= category == TestCategory.ZEPHYR;
        excluded |= category == TestCategory.ARDUINO;
        return !excluded;
    }
}
