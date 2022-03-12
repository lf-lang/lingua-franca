package org.lflang.tests;

import java.util.EnumSet;
import java.util.List;

import org.junit.jupiter.api.Assumptions;
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
public abstract class RuntimeTest extends TestBase {

    /**
     * Construct a test instance that runs tests for a single target.
     *
     * @param target The target to run tests for.
     */
    protected RuntimeTest(Target target) {
        super(target);
    }

    /**
     * Construct a test instance that runs tests for a list of targets.
     * @param targets The targets to run tests for.
     */
    protected RuntimeTest(List<Target> targets) {
        super(targets);
    }

    /**
     * Whether to enable {@link #runFederatedTests()}.
     */
    protected boolean supportsFederatedExecution() {
        return false;
    }

    /**
     * Whether to enable {@link #runTypeParameterTests()}.
     */
    protected boolean supportsGenericTypes() {
        return false;
    }

    /**
     * Whether to enable {@link #runDockerTests()} and {@link #runDockerFederatedTests()}.
     */
    protected boolean supportsDockerOption() {
        return false;
    }

    @Test
    public void runExampleTests() {
        runTestsForTargets(Message.DESC_EXAMPLE_TESTS,
                TestCategory.EXAMPLE_TEST::equals, Configurators::noChanges,
                TestLevel.EXECUTION, false);
    }

    @Test
    public void validateExamples() {
        runTestsForTargets(Message.DESC_EXAMPLES,
                TestCategory.EXAMPLE::equals, Configurators::noChanges, TestLevel.VALIDATION,
                false);
    }

    @Test
    public void runGenericTests() {
        runTestsForTargets(Message.DESC_GENERIC,
                           TestCategory.GENERIC::equals, Configurators::noChanges,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runTargetSpecificTests() {
        runTestsForTargets(Message.DESC_TARGET_SPECIFIC,
                           TestCategory.TARGET::equals, Configurators::noChanges,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runMultiportTests() {
        runTestsForTargets(Message.DESC_MULTIPORT,
                           TestCategory.MULTIPORT::equals, Configurators::noChanges,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runTypeParameterTests() {
        Assumptions.assumeTrue(supportsGenericTypes(), Message.NO_GENERICS_SUPPORT);
        runTestsForTargets(Message.DESC_TYPE_PARMS,
                           TestCategory.GENERICS::equals, Configurators::noChanges,
                           TestLevel.EXECUTION, false);
    }

    @Test
    public void runAsFederated() {
        Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);

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
        Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);
        runTestsForTargets(Message.DESC_FEDERATED,
                           TestCategory.FEDERATED::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);
    }

    /**
     * Run the tests for modal reactors.
     */
    @Test
    public void runModalTests() {
        runTestsForTargets(Message.DESC_MODAL,
                           TestCategory.MODAL_MODELS::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);
    }

    /** 
      * Run docker tests, provided that the platform is Linux and the target supports Docker.
      * Skip if platform is not Linux or target does not support Docker.
      */
    @Test
    public void runDockerTests() {
        Assumptions.assumeTrue(isLinux(), Message.NO_DOCKER_TEST_SUPPORT);
        Assumptions.assumeTrue(supportsDockerOption(), Message.NO_DOCKER_SUPPORT);
        runTestsForTargets(Message.DESC_DOCKER,
                           TestCategory.DOCKER::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);
    }

    /** 
      * Run federated docker tests, provided that the platform is Linux, the target supports Docker,
      * and the target supports federated execution. If any of these requirements are not met, skip
      * the tests.
      */
    @Test
    public void runDockerFederatedTests() {
        Assumptions.assumeTrue(isLinux(), Message.NO_DOCKER_TEST_SUPPORT);
        Assumptions.assumeTrue(supportsDockerOption(), Message.NO_DOCKER_SUPPORT);
        Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);
        runTestsForTargets(Message.DESC_DOCKER_FEDERATED,
                           TestCategory.DOCKER_FEDERATED::equals, Configurators::noChanges, TestLevel.EXECUTION,
                           false);
    }


    @Test
    public void runWithThreadingOff() {
        Assumptions.assumeTrue(supportsSingleThreadedExecution(), Message.NO_SINGLE_THREADED_SUPPORT);
        this.runTestsForTargets(
            Message.DESC_SINGLE_THREADED,
            Configurators::compatibleWithThreadingOff,
            Configurators::disableThreading,
            TestLevel.EXECUTION,
            true
        );
    }
}
