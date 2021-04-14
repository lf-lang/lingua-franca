package org.lflang.tests.runtime;

import org.lflang.Target;
import org.junit.jupiter.api.Test;

/**
 * Collection of tests for the TypeScript target.
 * 
 * Even though all tests are implemented in the base class, we override them
 * here so that each test can be easily invoked individually from the Eclipse.
 * This is done by right-clicking on the name of the test method and selecting
 * "Run As -> JUnit Test" from the pop-up menu.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */

public class TypeScriptTest extends TestBase {
    TypeScriptTest() {
        this.target = Target.TS;
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
    public void runAsFederated() {
        System.out.println("FIXME");
        //super.runNonFederatedTestsAsFederated();
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
