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
package org.lflang.tests.compiler

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import org.lflang.DefaultErrorReporter
import org.lflang.ModelInfo
import org.lflang.generator.ReactorInstance
import org.lflang.lf.Instantiation
import org.lflang.lf.LfFactory
import org.lflang.lf.Model
import org.lflang.lf.Reactor
import org.lflang.tests.LFInjectorProvider

import static extension org.lflang.ASTUtils.*

@ExtendWith(InjectionExtension)
@InjectWith(LFInjectorProvider)

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
                mainDef = LfFactory.eINSTANCE.createInstantiation()
                mainDef.setName(reactor.name)
                mainDef.setReactorClass(reactor)
            }
        }

        val instance = new ReactorInstance(mainDef.reactorClass.toDefinition, new DefaultErrorReporter());
        Assertions.assertFalse(instance.getCycles().isEmpty());
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
		info.update(model, new DefaultErrorReporter())
		Assertions.assertTrue(info.instantiationGraph.hasCycles == true, 
        	"Did not detect cyclic instantiation.")
    }
    
}
