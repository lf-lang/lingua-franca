/* Scoping unit tests. */

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
package org.lflang.tests.runtime

import org.lflang.Target
import org.junit.jupiter.api.Test

/**
 * Collection of tests for the Python target.
 * 
 * Even though all tests are implemented in the base class, we override them
 * here so that each test can be easily invoked individually from the Eclipse.
 * This is done by right-clicking anywhere in the header or body of the test
 * method and selecting "Run As -> JUnit Test" from the pop-up menu.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class PythonTest extends TestBase {
        
    new() {
        this.target = Target.Python
    }
    
    @Test
    override runGenericTests() {
        super.runGenericTests()
    }
    
    @Test
    override runTargetSpecificTests() {
        super.runTargetSpecificTests()
    }
    
    @Test
    override runMultiportTests() {
        super.runMultiportTests()
    }
    
    @Test
    override runAsFederated() {
        println("FIXME")
        //super.runNonFederatedTestsAsFederated()
    }
    
        
    @Test
    override runConcurrentTests() {
        super.runConcurrentTests()
    }
    
    @Test
    override runFederatedTests() {
        super.runFederatedTests()
    }
}
