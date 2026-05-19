package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the Python target.
 *
 * <p>Even though all tests are implemented in the base class, we override them here so that each
 * test can be easily invoked individually from IDEs with JUnit support like Eclipse and IntelliJ.
 * This is typically done by right-clicking on the name of the test method and then clicking "Run".
 *
 * @author Marten Lohstroh
 * @ingroup Tests
 */
public class PythonTest extends RuntimeTest {

  public PythonTest() {
    super(Target.Python);
  }

  @Override
  protected void applyDefaultConfiguration(TargetConfig config) {
    super.applyDefaultConfiguration(config);
    if (System.getProperty("os.name").startsWith("Windows")) {
      // Use the RelWithDebInfo build type on Windows as the Debug/Test build type produces linker
      // Errors in CI
      BuildTypeProperty.INSTANCE.override(config, BuildType.REL_WITH_DEB_INFO);
    }
  }

  @Override
  protected boolean supportsFederatedExecution() {
    return true;
  }

  @Override
  protected boolean supportsSingleThreadedExecution() {
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
  public void runTargetSpecificTests() {
    super.runTargetSpecificTests();
  }

  @Test
  @Override
  public void runMultiportTests() {
    super.runMultiportTests();
  }

  @Test
  @Disabled("TODO")
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
