package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the C target.
 *
 * <p>Tests that are implemented in the base class are still overridden so that each test can be
 * easily invoked individually from IDEs with JUnit support like Eclipse and IntelliJ. This is
 * typically done by right-clicking on the name of the test method and then clicking "Run".*
 *
 * @author Marten Lohstroh
 * @ingroup Tests
 */
public class CTest extends RuntimeTest {

  public CTest() {
    super(Target.C);
  }

  @Override
  protected boolean supportsSingleThreadedExecution() {
    return true;
  }

  @Override
  protected boolean supportsFederatedExecution() {
    return true;
  }

  @Override
  protected boolean supportsDockerOption() {
    return true;
  }

  @Override
  protected boolean supportsEnclaves() {
    return true;
  }

  @Test
  @Override
  public void runBasicTests() {
    super.runBasicTests();
  }

  @Test
  @Override
  public void runGenericsTests() {
    super.runGenericsTests();
  }

  @Test
  @Override
  public void runEnclaveTests() {
    // Enclaves do not work on Windows. They deadlock.
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runEnclaveTests();
  }

  @Test
  @Override
  public void runTargetSpecificTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runTargetSpecificTests();
  }

  @Test
  @Override
  public void runMultiportTests() {
    super.runMultiportTests();
  }

  @Test
  @Override
  public void runWithThreadingOff() {
    super.runWithThreadingOff();
  }

  @Test
  @Disabled("TODO only 27/96 tests pass")
  @Override
  public void runAsFederated() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runAsFederated();
  }

  @Test
  @Override
  public void runConcurrentTests() {
    super.runConcurrentTests();
  }

  @Test
  @Override
  public void runFederatedTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runFederatedTests();
  }

  @Test
  public void runModalTests() {
    super.runModalTests();
  }

  @Test
  public void runNoInliningTests() {
    super.runNoInliningTests();
  }

  @Test
  @Override
  public void runDockerTests() {
    super.runDockerTests();
  }

  @Test
  @Override
  public void runDockerFederatedTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runDockerFederatedTests();
  }
}
