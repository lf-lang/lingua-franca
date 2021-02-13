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

import org.junit.jupiter.api.^extension.ExtendWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.InjectWith
import org.junit.jupiter.api.Test
import org.eclipse.xtext.testing.util.ParseHelper
import org.icyphy.linguaFranca.Model
import javax.inject.Inject
import org.junit.jupiter.api.Assertions

import static extension org.icyphy.ASTUtils.*
import org.icyphy.linguaFranca.StateVar

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
}