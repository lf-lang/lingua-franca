/* ASTUtils Unit Tests. */

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

import java.util.LinkedList
import javax.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.icyphy.ASTUtils
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.StateVar
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith

import static extension org.icyphy.ASTUtils.*

/**
 * Collection of unit tests on the ASTutils.
 * 
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)
class LinguaFrancaASTUtilsTest {
    @Inject extension ParseHelper<Model>
    
    /**
     * Test that isInititialized returns true for inititialized state variables
     */
    @Test
    def void initializedState() {
        val model = '''
            target Cpp;
            main reactor Foo {
                state a();
                state b:int(3);
                state c:int[](1,2,3);
                state d(1 sec);
                state e(1 sec, 2 sec, 3 sec);
            }
        '''.parse
        
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing: " +
                model.eResource.errors)

        for (state : model.eAllContents.filter(StateVar).toList) {
            Assertions.assertTrue(state.isInitialized)
        }
    }
    
    /**
     * Test that isInititialized returns false for uninititialized state variables
     */
    @Test
    def void uninitializedState() {
        val model = '''
            target Cpp;
            main reactor Foo {
                state a;
                state b:int;
                state c:int[];
                state d:time;
                state e:time[];
            }
        '''.parse
        
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing: " +
                model.eResource.errors)

        for (state : model.eAllContents.filter(StateVar).toList) {
            Assertions.assertFalse(state.isInitialized)
        }
    }
    
    /**
     * Test reading initial values of parameters.
     * This checks that the example given in the documentation of the
     * ASTUtils.initialValue() function behaves as stated in the docs.
     */
     @Test
     def void initialValue() {
        val model = '''
            target C;
            reactor A(x:int(1)) {}
            reactor B(y:int(2)) {
                a1 = new A(x = y);
                a2 = new A(x = -1);
            }
            reactor C(z:int(3)) {
                b1 = new B(y = z);
                b2 = new B(y = -2);
            }
        '''.parse
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
                "Encountered unexpected error while parsing: " +
                model.eResource.errors)
        
        // Find all the Instantiations.
        var a1 = null as Instantiation;
        var a2 = null as Instantiation;
        var b1 = null as Instantiation;
        var b2 = null as Instantiation;
        for (instantiation: model.eAllContents.filter(Instantiation).toList) {
            switch (instantiation.name) {
                case "a1": a1 = instantiation
                case "a2": a2 = instantiation
                case "b1": b1 = instantiation
                case "b2": b2 = instantiation
            }
        }
        
        // Construct all relevant instantiation lists.
        val list_a1 = new LinkedList<Instantiation>();
        list_a1.add(a1);
        val list_a2 = new LinkedList<Instantiation>();
        list_a2.add(a2);
        val list_a1b1 = new LinkedList<Instantiation>();
        list_a1b1.add(a1);
        list_a1b1.add(b1);
        val list_a2b1 = new LinkedList<Instantiation>();
        list_a2b1.add(a2);
        list_a2b1.add(b1);
        val list_a1b2 = new LinkedList<Instantiation>();
        list_a1b2.add(a1);
        list_a1b2.add(b2);
         val list_a2b2 = new LinkedList<Instantiation>();
        list_a2b2.add(a2);
        list_a2b2.add(b2);
        val list_b1 = new LinkedList<Instantiation>();
        list_b1.add(b1);
        val list_b2 = new LinkedList<Instantiation>();
        list_b2.add(b2);
        
        /* Check for this:
         *     initialValue(x, null) returns 1
         *     initialValue(x, [a1]) returns 2
         *     initialValue(x, [a2]) returns -1
         *     initialValue(x, [a1, b1]) returns 3
         *     initialValue(x, [a2, b1]) returns -1
         *     initialValue(x, [a1, b2]) returns -2
         *     initialValue(x, [a2, b2]) returns -1
         * 
         *     initialValue(y, null) returns 2
         *     initialValue(y, [a1]) throws an IllegalArgumentException
         *     initialValue(y, [b1]) returns 3
         *     initialValue(y, [b2]) returns -2
         */
        for (parameter : model.eAllContents.filter(Parameter).toList) {
            if (parameter.name == 'x') {
                var values = ASTUtils.initialValue(parameter, null);
                Assertions.assertEquals(values.get(0).literal, "1");
                
                values = ASTUtils.initialValue(parameter, list_a1);
                Assertions.assertEquals(values.get(0).literal, "2");

                values = ASTUtils.initialValue(parameter, list_a2);
                Assertions.assertEquals(values.get(0).literal, "-1");

                values = ASTUtils.initialValue(parameter, list_a1b1);
                Assertions.assertEquals(values.get(0).literal, "3");

                values = ASTUtils.initialValue(parameter, list_a2b1);
                Assertions.assertEquals(values.get(0).literal, "-1");

                values = ASTUtils.initialValue(parameter, list_a1b2);
                Assertions.assertEquals(values.get(0).literal, "-2");

                values = ASTUtils.initialValue(parameter, list_a2b2);
                Assertions.assertEquals(values.get(0).literal, "-1");
            } else if (parameter.name == 'y') {
                var values = ASTUtils.initialValue(parameter, null);
                Assertions.assertEquals(values.get(0).literal, "2");
                
                try {
                    values = ASTUtils.initialValue(parameter, list_a1);
                } catch (IllegalArgumentException ex) {
                    Assertions.assertTrue(ex.message.startsWith("Parameter y is not"));
                }
                
                values = ASTUtils.initialValue(parameter, list_b1);
                Assertions.assertEquals(values.get(0).literal, "3");
                
                values = ASTUtils.initialValue(parameter, list_b2);
                Assertions.assertEquals(values.get(0).literal, "-2");
            }
        }
     }
}