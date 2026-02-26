package org.lflang.tests.runtime;

import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the Cpp target. Even though all tests are implemented in the base class,
 * we override them here so that each test can be easily invoked individually from IDEs with JUnit
 * support like Eclipse and IntelliJ. This is typically done by right-clicking on the name of the
 * test method and then clicking "Run".
 *
 * @author Marten Lohstroh
 * @ingroup Tests
 */
public class CppTest extends RuntimeTest {

  public CppTest() {
    super(Target.CPP);
  }

  @Override
  protected boolean supportsEnclaves() {
    return true;
  }

  @Override
  protected boolean supportsDockerOption() {
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
  @Override
  public void runConcurrentTests() {
    super.runConcurrentTests();
  }

  @Test
  @Override
  public void runFederatedTests() {
    super.runFederatedTests();
  }

  @Test
  public void runRos2Tests() {}
}
