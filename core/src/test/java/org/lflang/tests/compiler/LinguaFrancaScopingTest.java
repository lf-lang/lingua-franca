package org.lflang.tests.compiler;

import com.google.inject.Inject;
import com.google.inject.Provider;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.linking.impl.XtextLinkingDiagnostic;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.eclipse.xtext.testing.validation.ValidationTestHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.generator.LFGenerator;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;

/**
 * Test harness for ensuring that cross-references are established correctly and reported when
 * faulty.
 * @ingroup Tests
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class LinguaFrancaScopingTest {
  @Inject ParseHelper<Model> parser;

  @Inject LFGenerator generator;

  @Inject JavaIoFileSystemAccess fileAccess;

  @Inject Provider<ResourceSet> resourceSetProvider;

  @Inject ValidationTestHelper validator;

  /** Ensure that invalid references to contained reactors are reported. */
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
    Model model =
        parser.parse(
            String.join(
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
                "}"));

    Assertions.assertNotNull(model);
    Assertions.assertTrue(
        model.eResource().getErrors().isEmpty(), "Encountered unexpected error while parsing.");
    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Instantiation 's'");
    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'y'");
  }

  /** Ensure that invalid references to ports of contained reactors are reported. */
  @Test
  public void unresolvedHierarchicalPortReference() throws Exception {
    Model model =
        parser.parse(
            """
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
    Assertions.assertTrue(
        model.eResource().getErrors().isEmpty(), "Encountered unexpected error while parsing.");
    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'x'");
    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'y'");
  }

  @Test
  public void unresolvedReferenceInTriggerClause() throws Exception {
    Model model =
        parser.parse(
            """
                target C;
                main reactor {
                    reaction(unknown) {==}
                }
            """);

    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'unknown'.");
  }

  @Test
  public void unresolvedReferenceInUseClause() throws Exception {
    Model model =
        parser.parse(
            """
                target C;
                main reactor {
                    reaction() unknown {==}
                }
            """);

    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'unknown'.");
  }

  @Test
  public void unresolvedReferenceInEffectsClause() throws Exception {
    Model model =
        parser.parse(
            """
                target C;
                main reactor {
                    reaction() -> unknown {==}
                }
            """);

    validator.assertError(
        model,
        LfPackage.eINSTANCE.getVarRef(),
        XtextLinkingDiagnostic.LINKING_DIAGNOSTIC,
        "Couldn't resolve reference to Variable 'unknown'.");
  }
}
