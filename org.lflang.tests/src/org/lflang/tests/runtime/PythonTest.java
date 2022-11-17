/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.tests.runtime;

import java.util.Properties;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.RuntimeTest;

/**
 * Collection of tests for the Python target.
 *
 * Even though all tests are implemented in the base class, we override them
 * here so that each test can be easily invoked individually from IDEs with
 * JUnit support like Eclipse and IntelliJ.
 * This is typically done by right-clicking on the name of the test method and
 * then clicking "Run".
 *
 * @author Marten Lohstroh {@literal <marten@berkeley.edu>}
 */
public class PythonTest extends RuntimeTest {

    public PythonTest() {
        super(Target.Python);
    }

    @Override
    protected void addExtraLfcArgs(Properties args) {
        super.addExtraLfcArgs(args);
        if (System.getProperty("os.name").startsWith("Windows")) {
            // Use the RelWithDebInfo build type on Windows as the Debug/Test build type produces linker Errors in CI
            args.setProperty("build-type", "RelWithDebInfo");
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
