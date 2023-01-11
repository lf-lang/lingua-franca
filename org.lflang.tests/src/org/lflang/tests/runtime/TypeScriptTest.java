package org.lflang.tests.runtime;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the TypeScript target.
 *
 * Even though all tests are implemented in the base class, we override them
 * here so that each test can be easily invoked individually from IDEs with
 * JUnit support like Eclipse and IntelliJ.
 * This is typically done by right-clicking on the name of the test method and
 * then clicking "Run".
 *
 * @author Marten Lohstroh
 */
public class TypeScriptTest extends RuntimeTest {
    public TypeScriptTest() {
        super(Target.TS);
    }

    @Override
    protected boolean supportsDockerOption() {
        return true;
    }

    @Override
    protected boolean supportsFederatedExecution() {
        return true;
    }

    @Test
    @Override
    public void runGenericTests() {
        super.runGenericTests();
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

    @Test
    @Override
    public void runAsFederated() {}
}
