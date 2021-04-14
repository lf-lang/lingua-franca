/* Tests for code coverage. */

/*************
 * Copyright (c) 2021, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.tests.compiler

import org.lflang.Target
import org.junit.jupiter.api.Test
import org.lflang.tests.runtime.ThreadedBase

/**
 * Collection of tests intended to touch as many lines of the code
 * generator as possible for the purpose of recording code coverage.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class CodeGenCoverage extends ThreadedBase {

    new() {
        // Only generate code.
        this.check = false;
        this.build = false;
        this.run = false;
    }

    @Test
    override compileExamples() {
        for (target : Target.values()) {
            this.target = target;
            super.compileExamples()
        }
    }


    @Test
    override runExampleTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runExampleTests()
        }
    }

    @Test
    override runGenericTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runGenericTests()
        }
    }

    @Test
    override runTargetSpecificTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runTargetSpecificTests()
        }
    }

    @Test
    override runMultiportTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runMultiportTests()
        }
    }

    @Test
    override runWithFourThreads() {
        for (target : Target.values()) {
            this.target = target;
            super.runWithFourThreads()
        }
    }

    @Test
    override runAsFederated() {
        this.target = Target.C; // Only meaningful in C, so far.
        super.runAsFederated()
        
    }

    @Test
    override runConcurrentTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runConcurrentTests()
        }
    }

    @Test
    override runFederatedTests() {
        for (target : Target.values()) {
            this.target = target;
            super.runFederatedTests()
        }
    }
}
