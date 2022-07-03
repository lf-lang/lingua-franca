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
package org.lflang.tests.compiler;

import com.google.inject.Inject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.lflang.lf.Model;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import org.eclipse.xtext.testing.validation.ValidationTestHelper;
import org.lflang.lf.LfPackage;
import org.eclipse.xtext.linking.impl.XtextLinkingDiagnostic;
import org.lflang.tests.LFInjectorProvider;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.generator.LFGenerator;
import com.google.inject.Provider;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)

/**
 * Test harness for ensuring that cross-references are 
 * established correctly and reported when faulty.
 */
public class LinguaFrancaScopingTest {
    @Inject 
    ParseHelper<Model> parser;
    
    @Inject
    LFGenerator generator;
    
    @Inject
    JavaIoFileSystemAccess fileAccess;
    
    @Inject
    Provider<ResourceSet> resourceSetProvider;
    
    @Inject
    ValidationTestHelper validator;
    
    /**
     * Ensure that invalid references to contained reactors are reported.
     */
    @Test
    public void unresolvedReactorReference() throws Exception {
// Java 17:
//        Model model = """
//            target C;
//            reactor From {
//                output y:int;
//            }
//            reactor To {
//                input x:int;
//            }
//            
//            main reactor {
//                a = new From();
//                d = new To();
//                s.y -> d.x;
//            }
//        """;
// Java 11:
        Model model = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target C;", 
            "reactor From {",
            "    output y:int;",
            "}",
            "reactor To {",
            "    input x:int;",
            "}",
            "",
            "main reactor {",
            "    a = new From();",
            "    d = new To();",
            "    s.y -> d.x;",
            "}"
        ));
                
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
            "Encountered unexpected error while parsing.");
        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Instantiation 's'");
        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'y'");
    }
    
    
    /**
     * Ensure that invalid references to ports
     * of contained reactors are reported.
     */
    @Test
    public void unresolvedHierarchicalPortReference() throws Exception {
        Model model = parser.parse("""
            target C;
            reactor From {
                output y:int;
            }
            reactor To {
                input x:int;
            }

            main reactor {
                a = new From();
                d = new To();
                a.x -> d.y;
            }
        """);
        
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
            "Encountered unexpected error while parsing.");
        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'x'");
        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'y'");
    }

    @Test
    public void unresolvedReferenceInTriggerClause() throws Exception {
        Model model = parser.parse("""
            target C;
            main reactor {
                reaction(unknown) {==}
            }
        """);
        
    validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'unknown'.");
    }

    @Test
    public void unresolvedReferenceInUseClause() throws Exception {
        Model model = parser.parse("""
            target C;
            main reactor {
                reaction() unknown {==}
            }
        """);
        
        
        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'unknown'.");
    }

    @Test
    public void unresolvedReferenceInEffectsClause() throws Exception {
        Model model = parser.parse("""
            target C;
            main reactor {
                reaction() -> unknown {==}
            }
        """);

        validator.assertError(model, LfPackage.eINSTANCE.getVarRef(),
            XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
            "Couldn't resolve reference to Variable 'unknown'.");
    }

}
