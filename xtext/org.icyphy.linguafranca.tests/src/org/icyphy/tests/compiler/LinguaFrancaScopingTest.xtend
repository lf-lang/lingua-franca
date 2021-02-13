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
package org.icyphy.tests

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.icyphy.linguaFranca.Model
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.^extension.ExtendWith
import org.eclipse.xtext.testing.validation.ValidationTestHelper
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.eclipse.xtext.linking.impl.XtextLinkingDiagnostic

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

/**
 * Test harness for ensuring that cross-references are 
 * established correctly and reported when faulty.
 */
class LinguaFrancaScopingTest {
    @Inject extension ParseHelper<Model>
    @Inject extension ValidationTestHelper
    
    /**
     * Ensure that invalid references to contained reactors are reported.
     */
    @Test
    def void unresolvedReactorReference() {
        val model = '''
            target C;
            reactor From {
                output y:int;
            }
            reactor To {
                input x:int;
            }
            
            main reactor WrongConnect {
                a = new From();
                d = new To();
                s.y -> d.x;
            }
        '''.parse
        
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.varRef,
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Instantiation 's'")
        model.assertError(LinguaFrancaPackage::eINSTANCE.varRef,
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'y'")
    }
    
    
    /**
     * Ensure that invalid references to ports
     * of contained reactors are reported.
     */
    @Test
    def void unresolvedHierarchicalPortReference() {
        val model = '''
            target C;
            reactor From {
                output y:int;
            }
            reactor To {
                input x:int;
            }
            
            main reactor WrongConnect {
                a = new From();
                d = new To();
                a.x -> d.y;
            }
        '''.parse

        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.varRef,
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'x'")
        model.assertError(LinguaFrancaPackage::eINSTANCE.varRef,
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'y'")
    }
    
}
