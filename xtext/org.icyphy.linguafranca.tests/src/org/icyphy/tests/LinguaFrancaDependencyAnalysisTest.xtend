/* Dependency analysis unit tests. */

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
package org.icyphy.tests

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.icyphy.generator.CGenerator
import org.icyphy.generator.ReactorInstance
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Reactor
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import org.icyphy.ModelInfo

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

/**
 * A collection of tests to ensure dependency analysis is done correctly.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class LinguaFrancaDependencyAnalysisTest {
	@Inject extension ParseHelper<Model>
    
    /**
     * Check that circular dependencies between reactions are detected.
     */
    @Test
    def void cyclicDependency() {
        val model = '''
            target C;
            
            reactor Clock {
                timer t(0, 10 msec);
                input x:int;
                output y:int;
                reaction(t) -> y {=
                    
                =}
                reaction(x) -> y {=
                    
                =}
            }
            
            reactor A {
                input x:int;
                output y:int;
                reaction(x) -> y {=
                    
                =}
            }
            
            main reactor Loop {
                c = new Clock();
                a = new A();
                c.y -> a.x;
                a.y -> c.x;
            }
        '''.parse
        
        Assertions.assertNotNull(model)
        var Instantiation mainDef = null; 
        for (reactor : model.eAllContents.filter(Reactor).toList) {
            if (reactor.isMain) {
                // Creating an definition for the main reactor because 
                // there isn't one.
                mainDef = LinguaFrancaFactory.eINSTANCE.createInstantiation()
                mainDef.setName(reactor.name)
                mainDef.setReactorClass(reactor)
            }
        }
        
        var gen = new CGenerator()
        var message = ""
        try {
            new ReactorInstance(mainDef, null, gen)    
        } catch(Exception e) {
            message = e.message
        }
        
        Assertions.assertEquals(message, "Reactions form a cycle!")
    }

    /**
     * Check that circular instantiations are detected.
     */
    @Test
    def void circularInstantiation() {
        val model = '''target C;
    
		    reactor X {
		    	reaction() {=
		    	//
		    	=}
		    	p = new Y();
		    }
		    
		    reactor Y {
		    	q = new X();
		    }'''.parse
        
        Assertions.assertNotNull(model)
		
		var info = new ModelInfo()
		info.update(model)
        Assertions.assertTrue(info.instantiationGraph.cycles.size == 1, 
        	"Did not detect cyclic instantiation.")
    }
    
}
