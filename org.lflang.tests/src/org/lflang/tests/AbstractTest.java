package org.lflang.tests;

import java.util.EnumSet;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * A collection of JUnit tests to perform on a given set of targets.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public abstract class AbstractTest extends TestBase {

    /**
     * Whether to enable {@link #runWithFourThreads()}.
     */
    protected boolean supportsThreadsOption() {
        return false;
    }

    /**
     * Construct a test instance that runs tests for a single target.
     * @param target The target to run tests for.
     */
    protected AbstractTest(Target target) {
        super(target);
    }

    /**
     * Construct a test instance that runs tests for a list of targets.
     * @param targets The targets to run tests for.
     */
    protected AbstractTest(List<Target> targets) {
        super(targets);
    }
    
    @Test
    public void runExampleTests() {
        runTestsForTargets("Description: Run example tests.",
                TestCategory.EXAMPLE_TEST::equals, Configurators::noChanges,
                TestLevel.EXECUTION, false);
    }

    @Test
    public void validateExamples() {
        runTestsForTargets("Description: Validate examples.",
                TestCategory.EXAMPLE::equals, Configurators::noChanges, TestLevel.VALIDATION,
                false);
    }

    @Test
    public void runGenericTests() {
        runTestsForTargets("Description: Run generic tests (threads = 0).",
                           TestCategory.GENERIC::equals, Configurators::useSingleThread,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runTargetSpecificTests() {
        runTestsForTargets("Description: Run target-specific tests (threads = 0).",
                           TestCategory.TARGET::equals, Configurators::useSingleThread,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runMultiportTests() {
        runTestsForTargets("Description: Run multiport tests (threads = 0).",
                           TestCategory.MULTIPORT::equals, Configurators::useSingleThread,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runGenericsTests() {
        if (supportsGenericTypes()) {
            runTestsForTargets("Description: Run tests about generics.",
                               TestCategory.GENERICS::equals, Configurators::useSingleThread,
                               TestLevel.EXECUTION, false);
        } else {
            printSkipMessage("Description: Run tests about generics.", Message.NO_GENERICS_SUPPORT);
        }
    }

    protected boolean supportsGenericTypes() {
        return false;
    }

    @Test
    public void runSerializationTests() {
        runTestsForTargets("Description: Run serialization tests (threads = 0).",
                           TestCategory.SERIALIZATION::equals, Configurators::useSingleThread,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runAsFederated() {
        EnumSet<TestCategory> categories = EnumSet.allOf(TestCategory.class);
        categories.removeAll(EnumSet.of(TestCategory.CONCURRENT,
                                        TestCategory.FEDERATED,
                                        TestCategory.EXAMPLE,
                                        TestCategory.EXAMPLE_TEST,
                                        // FIXME: also run the multiport tests once these are supported.
                                        TestCategory.MULTIPORT));

        runTestsFor(List.of(Target.C),
                    Message.DESC_AS_FEDERATED,
                    categories::contains,
                    it -> ASTUtils.makeFederated(it.fileConfig.resource),
                    TestLevel.EXECUTION,
                    true);
    }


    @Test
    public void runConcurrentTests() {
        runTestsForTargets(Message.DESC_CONCURRENT,
                           TestCategory.CONCURRENT::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);

    }

    @Test
    public void runFederatedTests() {
        runTestsForTargets(Message.DESC_FEDERATED,
                           TestCategory.FEDERATED::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);
    }


    @Test
    public void runWithFourThreads() {
        if (supportsThreadsOption()) {
            this.runTestsForTargets(
                Message.DESC_FOUR_THREADS,
                Configurators::defaultCategoryExclusion,
                Configurators::useFourThreads,
                TestLevel.EXECUTION,
                true
            );
        } else {
            printSkipMessage(Message.DESC_FOUR_THREADS, Message.NO_THREAD_SUPPORT);
        }
    }
}
