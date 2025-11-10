package org.lflang.tests.compiler;

import static org.lflang.ast.ASTUtils.*;

import com.google.inject.Inject;
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.DefaultMessageReporter;
import org.lflang.ModelInfo;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.tests.LFInjectorProvider;

/**
 * A collection of tests to ensure dependency analysis is done correctly.
 *
 * @author Marten Lohstroh
 * @ingroup Tests
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
class LinguaFrancaDependencyAnalysisTest {
  @Inject ParseHelper<Model> parser;

  /** Check that circular dependencies between reactions are detected. */
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
    String testCase =
        String.join(
            System.getProperty("line.separator"),
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
            "}");
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

    ReactorInstance instance =
        new ReactorInstance(toDefinition(mainDef.getReactorClass()), new DefaultMessageReporter());
    Assertions.assertFalse(instance.getCycles().isEmpty());
  }

  /** Check that circular instantiations are detected. */
  @Test
  public void circularInstantiation() throws Exception {
    String testCase =
        """
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
    info.update(model, new DefaultMessageReporter());
    Assertions.assertTrue(
        info.instantiationGraph.hasCycles() == true, "Did not detect cyclic instantiation.");
  }
}
