/* Tests for code coverage. */

/*************
 * Copyright (c) 2021, The University of California at Berkeley. Redistribution
 * and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met: 1. Redistributions
 * of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer. 2. Redistributions in binary form
 * must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.tests.compiler;

import java.util.Arrays;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import org.lflang.Target;
import org.lflang.tests.AbstractTest;

/**
 * Collection of tests intended to touch as many lines of the code generator as
 * possible for the purpose of recording code coverage. 
 * 
 * NOTE: The overrides in this file are merely a convenience in that it allows 
 * IDE users to click on the tests to execute them individually.
 * 
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 */
public class CodeGenCoverage extends AbstractTest {

    CodeGenCoverage() {
        super(Arrays.asList(Target.values()));
        this.codeCovOnly = true;
    }

    @Override
    protected boolean supportsThreadsOption() {
        return true;
    }

    @Override
    protected boolean supportsFederatedExecution() {
        return true;
    }

    @Override
    protected boolean supportsGenericTypes() {
        return true;
    }

    @Test
    @Override
    public void validateExamples() {
        super.validateExamples();
    }

    @Test
    @Override
    public void runExampleTests() {
        super.runExampleTests();
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
    public void runWithFourThreads() {
        Assumptions.assumeFalse(true, Message.NOT_FOR_CODE_COV);
    }

    @Test
    @Override
    public void runSerializationTests() {
        super.runSerializationTests();
    }

    @Test
    @Override
    public void runAsFederated() {
        Assumptions.assumeFalse(true, Message.NOT_FOR_CODE_COV);
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
}
