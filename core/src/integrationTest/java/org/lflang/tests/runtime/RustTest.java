package org.lflang.tests.runtime;

import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the Rust target.
 *
 * @ingroup Tests
 */
public class RustTest extends RuntimeTest {

  public RustTest() {
    super(Target.Rust);
  }

  @Test
  @Override
  public void runGenericsTests() {
    super.runGenericsTests();
  }

  @Override
  protected boolean supportsGenericTypes() {
    return true;
  }
}
