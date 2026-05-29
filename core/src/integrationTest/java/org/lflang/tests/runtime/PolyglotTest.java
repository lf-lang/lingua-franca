package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the Polyglot target.
 *
 * <p>Even though all tests are implemented in the base class, we override them here so that each
 * test can be easily invoked individually from IDEs with JUnit support like Eclipse and IntelliJ.
 * This is typically done by right-clicking on the name of the test method and then clicking "Run".
 *
 * @author Hokeun Kim
 * @ingroup Tests
 */
public class PolyglotTest extends RuntimeTest {

  public PolyglotTest() {
    super(Target.Polyglot);
  }

  @Override
  protected boolean supportsFederatedExecution() {
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
  @Disabled(
      "Polyglot tests are already federated; runAsFederated applies C target tests which is not"
          + " applicable here")
  @Override
  public void runAsFederated() {
    super.runAsFederated();
  }

  @Test
  @Override
  public void runFederatedTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    super.runFederatedTests();
  }
}
