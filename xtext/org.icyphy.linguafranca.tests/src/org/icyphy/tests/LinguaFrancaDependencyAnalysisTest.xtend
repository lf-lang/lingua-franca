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

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)
class LinguaFrancaDependencyAnalysisTest {
	@Inject extension ParseHelper<Model>
    
    /**
     * Test whether or not cycles are detected.
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
                // Creating an definition for the main reactor because there isn't one.
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
}
