package org.lflang.tests;

import java.util.EnumSet;
import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.ast.ASTUtils;
import org.lflang.target.Target;
import org.lflang.tests.TestRegistry.TestCategory;

/**
 * A collection of JUnit tests to perform on a given set of targets.
 *
 * @author Marten Lohstroh
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
   *
   * @param targets The targets to run tests for.
   */
  protected RuntimeTest(List<Target> targets) {
    super(targets);
  }

  /** Whether to enable {@link #runEnclaveTests()}. */
  protected boolean supportsEnclaves() {
    return false;
  }

  /** Whether to enable {@link #runFederatedTests()}. */
  protected boolean supportsFederatedExecution() {
    return false;
  }

  /** Whether to enable {@link #runGenericsTests()}. */
  protected boolean supportsGenericTypes() {
    return false;
  }

  /** Whether to enable {@link #runDockerTests()} and {@link #runDockerFederatedTests()}. */
  protected boolean supportsDockerOption() {
    return false;
  }

  @Test
  public void runBasicTests() {
    runTestsForTargets(
        Message.DESC_BASIC,
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runGenericsTests() {
    runTestsForTargets(
        Message.DESC_GENERICS,
        TestCategory.GENERICS::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runTargetSpecificTests() {
    runTestsForTargets(
        Message.DESC_TARGET_SPECIFIC,
        TestCategory.TARGET::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runMultiportTests() {
    runTestsForTargets(
        Message.DESC_MULTIPORT,
        TestCategory.MULTIPORT::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runAsFederated() {
    Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);

    EnumSet<TestCategory> categories = EnumSet.allOf(TestCategory.class);
    categories.removeAll(
        EnumSet.of(
            TestCategory.CONCURRENT,
            TestCategory.FEDERATED,
            // FIXME: also run the multiport tests once these are supported.
            TestCategory.MULTIPORT));

    runTestsFor(
        List.of(Target.C),
        Message.DESC_AS_FEDERATED,
        categories::contains,
        it -> ASTUtils.makeFederated(it),
        Configurators::noChanges,
        TestLevel.EXECUTION,
        true);
  }

  @Test
  public void runConcurrentTests() {
    runTestsForTargets(
        Message.DESC_CONCURRENT,
        TestCategory.CONCURRENT::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runFederatedTests() {
    Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);
    runTestsForTargets(
        Message.DESC_FEDERATED,
        TestCategory.FEDERATED::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  /** Run the tests for modal reactors. */
  @Test
  public void runModalTests() {
    runTestsForTargets(
        Message.DESC_MODAL,
        TestCategory.MODAL_MODELS::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  /** Run the tests for non-inlined reaction bodies. */
  @Test
  public void runNoInliningTests() {
    runTestsForTargets(
        Message.DESC_MODAL,
        TestCategory.NO_INLINING::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  /**
   * Run docker tests, provided that the platform is Linux and the target supports Docker. Skip if
   * platform is not Linux or target does not support Docker.
   */
  @Test
  public void runDockerTests() {
    Assumptions.assumeTrue(isLinux(), Message.NO_DOCKER_TEST_SUPPORT);
    Assumptions.assumeTrue(supportsDockerOption(), Message.NO_DOCKER_SUPPORT);
    runTestsForTargets(
        Message.DESC_DOCKER,
        TestCategory.DOCKER::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  /**
   * Run federated docker tests, provided that the platform is Linux, the target supports Docker,
   * and the target supports federated execution. If any of these requirements are not met, skip the
   * tests.
   */
  @Test
  public void runDockerFederatedTests() {
    Assumptions.assumeTrue(isLinux(), Message.NO_DOCKER_TEST_SUPPORT);
    Assumptions.assumeTrue(supportsDockerOption(), Message.NO_DOCKER_SUPPORT);
    Assumptions.assumeTrue(supportsFederatedExecution(), Message.NO_FEDERATION_SUPPORT);
    runTestsForTargets(
        Message.DESC_DOCKER_FEDERATED,
        TestCategory.DOCKER_FEDERATED::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runWithThreadingOff() {
    Assumptions.assumeTrue(supportsSingleThreadedExecution(), Message.NO_SINGLE_THREADED_SUPPORT);
    this.runTestsForTargets(
        Message.DESC_SINGLE_THREADED,
        RuntimeTest::compatibleWithThreadingOff,
        Transformers::noChanges,
        Configurators::disableThreading,
        TestLevel.EXECUTION,
        true);
  }

  /** Run enclave tests if the target supports enclaves. */
  @Test
  public void runEnclaveTests() {
    Assumptions.assumeTrue(supportsEnclaves(), Message.NO_ENCLAVE_SUPPORT);
    runTestsForTargets(
        Message.DESC_ENCLAVE,
        TestCategory.ENCLAVE::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }

  /** Given a test category, return true if it is compatible with single-threaded execution. */
  public static boolean compatibleWithThreadingOff(TestCategory category) {

    // CONCURRENT, FEDERATED, DOCKER_FEDERATED, DOCKER
    // are not compatible with single-threaded execution.
    // ARDUINO and ZEPHYR have their own test suites, so we don't need to rerun.
    boolean excluded =
        category == TestCategory.CONCURRENT
            || category == TestCategory.SERIALIZATION
            || category == TestCategory.FEDERATED
            || category == TestCategory.DOCKER_FEDERATED
            || category == TestCategory.DOCKER
            || category == TestCategory.ENCLAVE
            || category == TestCategory.ARDUINO
            || category == TestCategory.VERIFIER
            || category == TestCategory.ZEPHYR_UNTHREADED
            || category == TestCategory.ZEPHYR_BOARDS
            || category == TestCategory.ZEPHYR_THREADED;

    // SERIALIZATION and TARGET tests are excluded on Windows.
    excluded |= isWindows() && category == TestCategory.TARGET;
    return !excluded;
  }
}
