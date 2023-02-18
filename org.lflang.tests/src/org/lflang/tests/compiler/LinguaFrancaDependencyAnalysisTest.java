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
package org.lflang.tests.compiler;

import com.google.inject.Inject;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.EOF;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.DefaultErrorReporter;
import org.lflang.ModelInfo;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.tests.LFInjectorProvider;
import static org.lflang.ASTUtils.*;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)

/**
 * A collection of tests to ensure dependency analysis is done correctly.
 * @author Marten Lohstroh
 */
class LinguaFrancaDependencyAnalysisTest {
	@Inject
    ParseHelper<Model> parser;
    
    /**
     * Check that circular dependencies between reactions are detected.
     */
    @Test
    public void cyclicDependency() throws Exception {
        // Java 17:
//         String testCase = """
//             target C;
//             
//             reactor Clock {
//                 timer t(0, 10 msec);
//                 input x:int;
//                 output y:int;
//                 reaction(t) -> y {=
//                     
//                 =}
//                 reaction(x) -> y {=
//                     
//                 =}
//             }
//             
//             reactor A {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                     
//                 =}
//             }
//             
//             main reactor Loop {
//                 c = new Clock();
//                 a = new A();
//                 c.y -> a.x;
//                 a.y -> c.x;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "",
            "reactor Clock {",
            "    timer t(0, 10 msec);",
            "    input x:int;",
            "    output y:int;",
            "    reaction(t) -> y {=",
            "        ",
            "    =}",
            "    reaction(x) -> y {=",
            "        ",
            "    =}",
            "}",
            "",
            "reactor A {",
            "    input x:int;",
            "    output y:int;",
            "    reaction(x) -> y {=",
            "        ",
            "    =}",
            "}",
            "",
            "main reactor Loop {",
            "    c = new Clock();",
            "    a = new A();",
            "    c.y -> a.x;",
            "    a.y -> c.x;",
            "}"
        );
        Model model = parser.parse(testCase);
        
        Assertions.assertNotNull(model);
        Instantiation mainDef = null;
        TreeIterator<EObject> it = model.eResource().getAllContents();
        while (it.hasNext()) {
            EObject obj = it.next();
            if (!(obj instanceof Reactor)) {
                continue;
            }
            Reactor reactor = (Reactor) obj;
            if (reactor.isMain()) {
                // Creating an definition for the main reactor because 
                // there isn't one.
                mainDef = LfFactory.eINSTANCE.createInstantiation();
                mainDef.setName(reactor.getName());
                mainDef.setReactorClass(reactor);
            }
        }

        ReactorInstance instance = new ReactorInstance(toDefinition(mainDef.getReactorClass()), new DefaultErrorReporter());
        Assertions.assertFalse(instance.getCycles().isEmpty());
    }

    /**
     * Check that circular instantiations are detected.
     */
    @Test
    public void circularInstantiation() throws Exception {
        String testCase = """
             target C;

             reactor X {
                 reaction() {=
                 //
                 =}
                 p = new Y();
             }

             reactor Y {
                 q = new X();
             }
         """;
        Model model = parser.parse(testCase);
        
        Assertions.assertNotNull(model);
		
		ModelInfo info = new ModelInfo();
		info.update(model, new DefaultErrorReporter());
		Assertions.assertTrue(info.instantiationGraph.hasCycles() == true, 
        	"Did not detect cyclic instantiation.");
    }
    
}
