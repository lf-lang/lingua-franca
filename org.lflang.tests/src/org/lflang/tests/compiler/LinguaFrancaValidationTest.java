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

import static org.lflang.ASTUtils.withoutQuotes;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.eclipse.xtext.testing.validation.ValidationTestHelper;
import org.eclipse.xtext.validation.Issue;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.ArrayType;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetProperty.DictionaryType;
import org.lflang.TargetProperty.PrimitiveType;
import org.lflang.TargetProperty.TargetPropertyType;
import org.lflang.TargetProperty.UnionType;
import org.lflang.TimeValue;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Model;
import org.lflang.lf.Visibility;
import org.lflang.tests.LFInjectorProvider;

import com.google.inject.Inject;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)


/**
 * Collection of unit tests to ensure validation is done correctly.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author(Christian Menard <christian.menard@tu-dresden.de>}
 */
public class LinguaFrancaValidationTest {
    @Inject 
    ParseHelper<Model> parser;

    @Inject
    ValidationTestHelper validator;

    /**
     * Helper function to parse a Lingua Franca program and expect no errors.
     * @return A model representing the parsed string.
     */
    private Model parseWithoutError(String s) throws Exception {
        Model model = parser.parse(s);
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
            "Encountered unexpected error while parsing: " +
                model.eResource().getErrors());
        return model;
    }

    /**
     * Helper function to parse a Lingua Franca program and expect errors.
     * @return A model representing the parsed string.
     */
    private Model parseWithError(String s) throws Exception {
        Model model = parser.parse(s);
        Assertions.assertNotNull(model);
        Assertions.assertFalse(model.eResource().getErrors().isEmpty());
        return model;
    } 

    /**
     * Ensure that duplicate identifiers for actions reported.
     */
    @Test
    public void duplicateVariable() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo {
//                 logical action bar;
//                 physical action bar;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo {",
        "    logical action bar;",
        "    physical action bar;",
        "}");
        validator.assertError(parseWithoutError(testCase), 
                              LfPackage.eINSTANCE.getAction(), 
                              null, 
                              "Duplicate Variable 'bar' in Reactor 'Foo'");
    }


    /**
     * Check that reactors in C++ cannot be named preamble 
     */
    @Test
    public void disallowReactorCalledPreamble() throws Exception {
// Java 17:
//         Model model_no_errors = """
//             target Cpp;
//             main reactor {
//             }
//         """
// Java 11:
        Model model_no_errors = parser.parse(String.join(
                System.getProperty("line.separator"),
                "target Cpp;", 
                "main reactor {",
                "}"
        ));
        
// Java 17:
//         Model model_error_1 = """
//             target Cpp;
//             main reactor Preamble {
//             }
//         """
// Java 11:
        Model model_error_1 = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target Cpp;", 
            "main reactor Preamble {",
            "}"
        ));

// Java 17:
//         Model model_error_2 = """
//             target Cpp;
//             reactor Preamble {
//             }
//             main reactor Main {
//             }
//         """
// Java 11:
        Model model_error_2 = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target Cpp;", 
            "reactor Preamble {",
            "}",
            "main reactor Main {",
            "}"
        ));
                
        Assertions.assertNotNull(model_no_errors);
        Assertions.assertNotNull(model_error_1);
        Assertions.assertNotNull(model_error_2);
        Assertions.assertTrue(model_no_errors.eResource().getErrors().isEmpty(), 
            "Encountered unexpected error while parsing: " + model_no_errors.eResource().getErrors());
            Assertions.assertTrue(model_error_1.eResource().getErrors().isEmpty(), 
            "Encountered unexpected error while parsing: " + model_error_1.eResource().getErrors());
            Assertions.assertTrue(model_error_2.eResource().getErrors().isEmpty(), 
            "Encountered unexpected error while parsing: " + model_error_2.eResource().getErrors());

        validator.assertNoIssues(model_no_errors);
        validator.assertError(model_error_1, 
                              LfPackage.eINSTANCE.getReactor(), 
                              null,
                              "Reactor cannot be named 'Preamble'");
        validator.assertError(model_error_2, 
                              LfPackage.eINSTANCE.getReactor(), 
                              null,
                              "Reactor cannot be named 'Preamble'");
    }


    /**
     * Ensure that "__" is not allowed at the start of an input name.
     */
    @Test
    public void disallowUnderscoreInputs() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor {
//                 input __bar;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor {",
        "    input __bar;",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInput(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    @Test
    public void disallowMainWithDifferentNameThanFile() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor Foo {}
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "main reactor Foo {}"
        );

        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Name of main reactor must match the file name (or be omitted)");
    }
    

    /**
     * Ensure that "__" is not allowed at the start of an output name.
     */
    @Test
    public void disallowUnderscoreOutputs() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo {
//                 output __bar;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo {",
        "    output __bar;",
        "}");

        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getOutput(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }

    /**
     * Ensure that "__" is not allowed at the start of an action name.
     */
    @Test
    public void disallowUnderscoreActions() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo {
//                 logical action __bar;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo {",
        "    logical action __bar;",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getAction(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar"); 
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a timer name.
     */
    @Test
    public void disallowUnderscoreTimers() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo {
//                 timer __bar(0);
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo {",
        "    timer __bar(0);",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getTimer(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar"); 
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a parameter name.
     */
    @Test
    public void disallowUnderscoreParameters() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo(__bar) {
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo(__bar) {",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getParameter(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of an state name.
     */
    @Test
    public void disallowUnderscoreStates() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor Foo {
//                 state __bar;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor Foo {",
        "    state __bar;",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getStateVar(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor definition name.
     */
    @Test
    public void disallowUnderscoreReactorDef() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             main reactor __Foo {
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "main reactor __Foo {",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __Foo");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor instantiation name.
     */
    @Test
    public void disallowUnderscoreReactorInstantiation() throws Exception {
// Java 17:
//         String testCase = """
//             target TypeScript;
//             reactor Foo {
//             }
//             main reactor Bar {
//                 __x = new Foo();
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target TypeScript;",
        "reactor Foo {",
        "}",
        "main reactor Bar {",
        "   __x = new Foo();",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInstantiation(), null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __x");
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    public void connectionToEffectPort() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Foo {
//                 output out:int;
//             }
//             main reactor Bar {
//                 output out:int;
//                 x = new Foo();
//                 x.out -> out;
//                 reaction(startup) -> out {=                    
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "reactor Foo {",
        "   output out:int;",
        "}",
        "main reactor Bar {",
        "   output out:int;",
        "   x = new Foo();",
        "   x.out -> out;",
        "   reaction(startup) -> out {=",
        "   =}",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getConnection(), null,
            "Cannot connect: Port named 'out' is already effect of a reaction.");
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    public void connectionToEffectPort2() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Foo {
//                 input inp:int;
//                 output out:int;
//             }
//             main reactor {
//                 output out:int;
//                 x = new Foo();
//                 y = new Foo();
//
//                 y.out -> x.inp;
//                 reaction(startup) -> x.inp {=                    
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "reactor Foo {",
        "   input inp:int;",
        "   output out:int;",
        "}",
        "main reactor {",
        "   output out:int;",
        "   x = new Foo();",
        "   y = new Foo();",
        "",
        "   y.out -> x.inp;",
        "   reaction(startup) -> x.inp {=",                    
        "   =}",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getConnection(), null,
            "Cannot connect: Port named 'inp' is already effect of a reaction.");
    }
    
    /**
     * Allow connection to the port of a contained reactor if another port with same name is effect of a reaction.
     */
    @Test
    public void connectionToEffectPort3() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//
//             reactor Foo {
//                 input in:int;
//             }
//             reactor Bar {
//                 input in:int;
//                 x1 = new Foo();
//                 x2 = new Foo();
//                 in -> x1.in;
//                 reaction(startup) -> x2.in {=
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "",
        "reactor Foo {",
        "   input in:int;",
        "}",
        "reactor Bar {",
        "   input in:int;",
        "   x1 = new Foo();",
        "   x2 = new Foo();",
        "   in -> x1.in;",
        "   reaction(startup) -> x2.in {=",
        "   =}",
        "}");
        validator.assertNoErrors(parseWithoutError(testCase));
    }

    /**
     * Allow connection to the port of a contained reactor if another port with same name is effect of a reaction.
     */
    @Test
    public void connectionToEffectPort3_5() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//
//             reactor Foo {
//                 input in:int;
//             }
//             main reactor {
//                 input in:int;
//                 x1 = new Foo();
//                 x2 = new Foo();
//                 in -> x1.in;
//                 reaction(startup) -> x2.in {=
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "",
        "reactor Foo {",
        "   input in:int;",
        "}",
        "main reactor {",
        "   input in:int;",
        "   x1 = new Foo();",
        "   x2 = new Foo();",
        "   in -> x1.in;",
        "   reaction(startup) -> x2.in {=",
        "   =}",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getVariable(), null,
                "Main reactor cannot have inputs.");
    }

    /**
     * Disallow connection to the port of a contained reactor if the same port is effect of a reaction.
     */
    @Test
    public void connectionToEffectPort4() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
                
//             reactor Foo {
//                 input in:int;
//             }
//             main reactor {
//                 input in:int;
//                 x1 = new Foo();
//                 in -> x1.in;
//                 reaction(startup) -> x1.in {=
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "",
        "reactor Foo {",
        "   input in:int;",
        "}",
        "main reactor {",
        "   input in:int;",
        "   x1 = new Foo();",
        "   in -> x1.in;",
        "   reaction(startup) -> x1.in {=",
        "   =}",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getConnection(), null,
            "Cannot connect: Port named 'in' is already effect of a reaction.");
    }

    /**
     * Disallow connection of multiple ports to the same input port.
     */
    @Test
    public void multipleConnectionsToInputTest() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Source {
//                 output out:int;
//             }
//             reactor Sink {
//                 input in:int;
//             }
//             main reactor {
//                 input in:int;
//                 src = new Source();
//                 sink = new Sink();
//                 in -> sink.in;
//                 src.out -> sink.in;
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "reactor Source {",
        "   output out:int;",
        "}",
        "reactor Sink {",
        "   input in:int;",
        "}",
        "main reactor {",
        "   input in:int;",
        "   src = new Source();",
        "   sink = new Sink();",
        "   in -> sink.in;",
        "   src.out -> sink.in;",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getConnection(), null,
            "Cannot connect: Port named 'in' may only appear once on the right side of a connection.");
    }

    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    public void detectInstantiationCycle() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Contained {
//                 x = new Contained();
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
        "target C;",
        "reactor Contained {",
        "    x = new Contained();",
        "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInstantiation(),
            null, "Instantiation is part of a cycle: Contained");
    }
    
    
    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    public void detectInstantiationCycle2() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Intermediate {
//                 x = new Contained();
//             }
//
//             reactor Contained {
//                 x = new Intermediate();
//             }
//         """
// Java 11:
        Model model = parseWithoutError(String.join(
            System.getProperty("line.separator"),
            "target C;", 
            "reactor Intermediate {",
            "   x = new Contained();",
            "}",
            "",
            "reactor Contained {",
            "   x = new Intermediate();",
            "}"
        ));
        validator.assertError(model, LfPackage.eINSTANCE.getInstantiation(),
            null, "Instantiation is part of a cycle: Intermediate, Contained.");
        validator.assertError(model, LfPackage.eINSTANCE.getInstantiation(),
            null, "Instantiation is part of a cycle: Intermediate, Contained.");
    }
    
    /**
     * Detect causality loop.
     */
    @Test
    public void detectCausalityLoop() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//
//             reactor X {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                 =}
//             }
//
//             main reactor {
//                 a = new X()
//                 b = new X()
//                 a.y -> b.x
//                 b.y -> a.x
//             }
//         """
// Java 11:
        Model model = parseWithoutError(String.join(
            System.getProperty("line.separator"),
            "target C;", 
            "",
            "reactor X {",
            "   input x:int;",
            "   output y:int;",
            "   reaction(x) -> y {=",
            "   =}",
            "}",
            "",
            "main reactor {",
            "   a = new X()",
            "   b = new X()",
            "   a.y -> b.x",
            "   b.y -> a.x",
            "}"
        ));
        validator.assertError(model, LfPackage.eINSTANCE.getReaction(),
            null, "Reaction triggers involved in cyclic dependency in reactor X: x.");
        validator.assertError(model, LfPackage.eINSTANCE.getReaction(),
            null, "Reaction effects involved in cyclic dependency in reactor X: y."); 
    }
    
    /**
     * Let cyclic dependencies be broken by "after" clauses.
     */
    @Test
    public void afterBreaksCycle() throws Exception {
// Java 17:
//         String testCase = """
//             target C
//                         
//             reactor X {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                 =}
//             }
//             
//             main reactor {
//                 a = new X()
//                 b = new X()
//                 a.y -> b.x after 5 msec
//                 b.y -> a.x
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C",
            "            ",
            "reactor X {",
            "    input x:int;",
            "    output y:int;",
            "    reaction(x) -> y {=",
            "    =}",
            "}",
            "",
            "main reactor {",
            "    a = new X()",
            "    b = new X()",
            "    a.y -> b.x after 5 msec",
            "    b.y -> a.x",
            "}");
        validator.assertNoErrors(parseWithoutError(testCase));
    }


    /**
     * Let cyclic dependencies be broken by "after" clauses with zero delay.
     */
    @Test
    public void afterBreaksCycle2() throws Exception {
// Java 17:
//         String testCase = """
//             target C
//                         
//             reactor X {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                 =}
//             }
//             
//             main reactor {
//                 a = new X()
//                 b = new X()
//                 a.y -> b.x after 0 sec
//                 b.y -> a.x
//             }
//         """
// Java 11:

        String testCase = String.join(System.getProperty("line.separator"),
            "target C",
            "            ",
            "reactor X {",
            "    input x:int;",
            "    output y:int;",
            "    reaction(x) -> y {=",
            "    =}",
            "}",
            "",
            "main reactor {",
            "    a = new X()",
            "    b = new X()",
            "    a.y -> b.x after 0 sec",
            "    b.y -> a.x",
            "}");
        validator.assertNoErrors(parseWithoutError(testCase));
    }


    /**
     * Let cyclic dependencies be broken by "after" clauses with zero delay and no units.
     */
    @Test
    public void afterBreaksCycle3() throws Exception {
// Java 17:
//         String testCase = """
//             target C
//                         
//             reactor X {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                 =}
//             }
//             
//             main reactor {
//                 a = new X()
//                 b = new X()
//                 a.y -> b.x after 0
//                 b.y -> a.x
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C",
            "            ",
            "reactor X {",
            "    input x:int;",
            "    output y:int;",
            "    reaction(x) -> y {=",
            "    =}",
            "}",
            "",
            "main reactor {",
            "    a = new X()",
            "    b = new X()",
            "    a.y -> b.x after 0",
            "    b.y -> a.x",
            "}");
        validator.assertNoErrors(parseWithoutError(testCase));
    }

    /**
     * Detect missing units in "after" clauses with delay greater than zero.
     */
    @Test
    public void nonzeroAfterMustHaveUnits() throws Exception {
// Java 17:
//         String testCase = """
//             target C
//                         
//             reactor X {
//                 input x:int;
//                 output y:int;
//                 reaction(x) -> y {=
//                 =}
//             }
//             
//             main reactor {
//                 a = new X()
//                 b = new X()
//                 a.y -> b.x after 1
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C",
            "            ",
            "reactor X {",
            "    input x:int;",
            "    output y:int;",
            "    reaction(x) -> y {=",
            "    =}",
            "}",
            "",
            "main reactor {",
            "    a = new X()",
            "    b = new X()",
            "    a.y -> b.x after 1",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getTime(),
            null, "Missing or invalid time unit.");
    }


    
    /**
     * Report non-zero time value without units.
     */
    @Test
    public void nonZeroTimeValueWithoutUnits() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor {
//                 timer t(42, 1 sec);
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    timer t(42, 1 sec);",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =}",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getValue(), null, "Missing time unit.");
    }    
    
    /**
     * Report reference to non-time parameter in time argument.
     */
    @Test
    public void parameterTypeMismatch() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor (p:int(0)) {
//                 timer t(p, 1 sec);
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor (p:int(0)) {",
            "    timer t(p, 1 sec);",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =}",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getValue(),
            null, "Parameter is not of time type");
    }
    
    /**
     * Report inappropriate literal in time argument.
     */
    @Test
    public void targetCodeInTimeArgument() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor {
//                 timer t({=foo()=}, 1 sec);
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    timer t({=foo()=}, 1 sec);",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =}",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getValue(),
            null, "Invalid time literal");
    }  
    

    /**
     * Report overflowing deadline.
     */
    @Test
    public void overflowingDeadlineC() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor {
//             timer t;
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =} deadline (40 hours) {=
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "timer t;",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =} deadline (40 hours) {=",
            "    =}",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getDeadline(), null,
            "Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE +
            " nanoseconds.");
    }  

    
    /**
     * Report overflowing parameter.
     */
    @Test
    public void overflowingParameterC() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             main reactor(d:time(40 hours)) {
//             timer t;
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =} deadline (d) {=
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor(d:time(40 hours)) {",
            "timer t;",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =} deadline (d) {=",
            "    =}",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getParameter(), null,
            "Time value used to specify a deadline exceeds the maximum of " +
            TimeValue.MAX_LONG_DEADLINE + " nanoseconds.");
    }  
    
    
    /**
     * Report overflowing assignment.
     */
    @Test
    public void overflowingAssignmentC() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Print(d:time(39 hours)) {
//                 timer t;
//                 reaction(t) {=
//                     printf("Hello World.\\n");
//                 =} deadline (d) {=
//                 =}
//             }
//             main reactor {
//                 p = new Print(d=40 hours);
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "reactor Print(d:time(39 hours)) {",
            "    timer t;",
            "    reaction(t) {=",
            "        printf(\"Hello World.\\n\");",
            "    =} deadline (d) {=",
            "    =}",
            "}",
            "main reactor {",
            "    p = new Print(d=40 hours);",
            "}");
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getAssignment(), null,
            "Time value used to specify a deadline exceeds the maximum of " +
            TimeValue.MAX_LONG_DEADLINE + " nanoseconds.");
    }  

    /**
     * Report missing trigger.
     */
    @Test
    public void missingTrigger() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor X {
//                 reaction() {=
//                     //
//                 =}
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "reactor X {",
            "    reaction() {=",
            "        //",
            "    =}",
            "}");
        validator.assertWarning(parseWithoutError(testCase), LfPackage.eINSTANCE.getReaction(), null,
            "Reaction has no trigger.");
    }
        
    /**
     * Test warnings and errors for the target dependent preamble visibility qualifiers 
     */
    @Test
    public void testPreambleVisibility() throws Exception {
        for (Target target : Target.values()) {
            for (Visibility visibility : Visibility.values()) {
// Java 17:
//         Model model_reactor_scope = """
//             target %s;
//             reactor Foo {
//                 %spreamble {==}
//             }
//         """.formatted(target, visibility != java.beans.Visibility.NONE ? visibility + " " : "");
// Java 11:
                Model model_reactor_scope = parseWithoutError(String.join(System.getProperty("line.separator"),
                    String.format("target %s;", target),
                    "reactor Foo {",
                    String.format("    %spreamble {==}", visibility != Visibility.NONE ? visibility + " " : ""),
                    "}"));

// Java 17:
//         Model model_file_scope = """
//             target %s;
//             %spreamble {==}
//             reactor Foo {
//             }
//         """.formatted(target, visibility != java.beans.Visibility.NONE ? visibility + " " : "");
// Java 11:
                Model model_file_scope = parseWithoutError(String.join(System.getProperty("line.separator"),
                    String.format("target %s;", target),
                    String.format("    %spreamble {==}", visibility != Visibility.NONE ? visibility + " " : ""),
                    "reactor Foo {",
                    "}"));

// Java 17:
//         Model model_no_preamble = """
//             target %s;
//             reactor Foo {
//             }
//         """.formatted(target);
// Java 11:
                Model model_no_preamble = parseWithoutError(String.join(System.getProperty("line.separator"),
                    String.format("target %s;", target),
                    "reactor Foo {",
                    "}"));
                
                validator.assertNoIssues(model_no_preamble);
                
                if (target == Target.CPP) {
                    if (visibility == Visibility.NONE) {
                        validator.assertError(model_file_scope, LfPackage.eINSTANCE.getPreamble(), null,
                            "Preambles for the C++ target need a visibility qualifier (private or public)!");
                        validator.assertError(model_reactor_scope, LfPackage.eINSTANCE.getPreamble(), null,
                            "Preambles for the C++ target need a visibility qualifier (private or public)!");  
                    } else {
                        validator.assertNoIssues(model_file_scope);
                        validator.assertNoIssues(model_reactor_scope);
                    }
                } else {
                    if (visibility == Visibility.NONE) {
                        validator.assertNoIssues(model_file_scope);
                        validator.assertNoIssues(model_reactor_scope);
                    } else {
                        validator.assertWarning(model_file_scope, LfPackage.eINSTANCE.getPreamble(), null,
                            String.format("The %s qualifier has no meaning for the %s target. It should be removed.", visibility, target.name()));
                        validator.assertWarning(model_reactor_scope, LfPackage.eINSTANCE.getPreamble(), null,
                            String.format("The %s qualifier has no meaning for the %s target. It should be removed.", visibility, target.name()));
                    }
                }
            }
        }
    }
    
    
    /**
     * Tests for state and parameter declarations, including native lists.
     */
    @Test
    public void stateAndParameterDeclarationsInC() throws Exception {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Bar(a(0),                // ERROR: type missing
//                         b:int,               // ERROR: uninitialized
//                         t:time(42),          // ERROR: units missing
//                         x:int(0),
//                         h:time("bla"),       // ERROR: not a type 
//                         q:time(1 msec, 2 msec),  // ERROR: not a list
//                         y:int(t)             // ERROR: init using parameter
//             ) {
//                 state offset:time(42);       // ERROR: units missing
//                 state w:time(x);             // ERROR: parameter is not a time
//                 state foo:time("bla");       // ERROR: assigned value not a time
//                 timer tick(1);               // ERROR: not a time
//             }
//         """
// Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "reactor Bar(a(0),              // ERROR: type missing",
            "            b:int,             // ERROR: uninitialized",
            "            t:time(42),        // ERROR: units missing",
            "            x:int(0),",
            "            h:time(\"bla\"),   // ERROR: not a type ",
            "            q:time(1 msec, 2 msec),  // ERROR: not a list",
            "            y:int(t)           // ERROR: init using parameter",
            ") {",
            "    state offset:time(42);     // ERROR: units missing",
            "    state w:time(x);           // ERROR: parameter is not a time",
            "    state foo:time(\"bla\");   // ERROR: assigned value not a time",
            "    timer tick(1);             // ERROR: not a time",
            "}");
        Model model = parseWithoutError(testCase);

        
		validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Type declaration missing.");
        validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Missing time unit.");
        validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Invalid time literal.");
        validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Time parameter cannot be initialized using a list.");   
        validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Parameter cannot be initialized using parameter.");
        validator.assertError(model, LfPackage.eINSTANCE.getStateVar(), null,
            "Referenced parameter does not denote a time.");
        validator.assertError(model, LfPackage.eINSTANCE.getStateVar(), null,
            "Invalid time literal.");
        validator.assertError(model, LfPackage.eINSTANCE.getParameter(), null,
            "Uninitialized parameter.");
       	validator.assertError(model, LfPackage.eINSTANCE.getValue(), null,
            "Missing time unit.");
    }  
    
    
    /**
     * Recognize valid IPV4 addresses, report invalid ones.
     */
    @Test
    public void recognizeIPV4() throws Exception {
        List<String> correct = List.of("127.0.0.1", "10.0.0.1", "192.168.1.1", "0.0.0.0", "192.168.1.1");
        List<String> parseError = List.of("10002.3.4", "1.2.3.4.5");
        List<String> validationError = List.of("256.0.0.0", "260.0.0.0");

        // Correct IP addresses.
        for (String addr : correct) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
        }

        // IP addresses that don't parse.
        for (String addr : parseError) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            parseWithError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
        }

        // IP addresses that parse but are invalid.
        for (String addr : validationError) {
// Java 17:
//         Model model = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            Model model = parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
            validator.assertWarning(model, LfPackage.eINSTANCE.getHost(), null, "Invalid IP address.");
        }
    }
    
    /**
     * Recognize valid IPV6 addresses, report invalid ones.
     */
    @Test
    public void recognizeIPV6() throws Exception {
        List<String> correct = List.of("1:2:3:4:5:6:7:8", "1:2:3:4:5:6:7::", "1:2:3:4:5:6::8",
            "1:2:3:4:5::8", "1:2:3:4::8", "1:2:3::8", "1:2::8", "1::8", "::8",
            "::", "1::3:4:5:6:7:8", "1::4:5:6:7:8", "1::5:6:7:8", "1::6:7:8",
            "1::7:8", "1::8", "1::", "1:2:3:4:5::7:8", "1:2:3:4::6:7:8",
            "1:2:3::5:6:7:8", "1:2::4:5:6:7:8", "1::3:4:5:6:7:8",
            "::2:3:4:5:6:7:8", "fe80::7:8", "fe80::7:8%eth0", "fe80::7:8%1",
            "::255.255.255.255", "::ffff:255.255.255.255",
            "::ffff:0:255.255.255.0", "::ffff:00:255.255.255.0",
            "::ffff:000:255.255.255.0", "::ffff:0000:255.255.255.0",
            "::ffff:0.0.0.0", "::ffff:1.2.3.4", "::ffff:10.0.0.1",
            "1:2:3:4:5:6:77:88", "ffff:ffff:ffff:ffff:ffff:ffff:ffff:ffff",
            "2001:db8:3:4::192.0.2.33", "64:ff9b::192.0.2.33", "0:0:0:0:0:0:10.0.0.1");
        
        List<String> validationError = List.of("1:2:3:4:5:6:7:8:9", "1:2:3:4:5:6::7:8",
            "1:2:3:4:5:6:7:8:", "::1:2:3:4:5:6:7:8", "1:2:3:4:5:6:7:8::",
            "1:2:3:4:5:6:7:88888", "2001:db8:3:4:5::192.0.2.33",
            "fe08::7:8interface", "fe08::7:8interface", "fe08::7:8i");
            
        List<String> parseError = List.of("fe08::7:8%", ":1:2:3:4:5:6:7:8");
        
        // Correct IP addresses.
        for (String addr : correct) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at [foo@%s]:4242 {
//                 y = new Y() at [%s]:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            Model model = parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor at [foo@%s]:4242 {", addr),
                    String.format("    y = new Y() at [%s]:2424; ", addr),
                    "}")
            );
            validator.assertNoIssues(model);
        }

                
        // IP addresses that don't parse.
        for (String addr : parseError) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at [foo@%s]:4242 {
//                 y = new Y() at [%s]:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            parseWithError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at [foo@%s]:4242 {", addr),
                    String.format("    y = new Y() at [%s]:2424; ", addr),
                    "}")
            );
        }

        // IP addresses that parse but are invalid.
        for (String addr : validationError) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at [foo@%s]:4242 {
//                 y = new Y() at [%s]:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            Model model = parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at [foo@%s]:4242 {", addr),
                    String.format("    y = new Y() at [%s]:2424; ", addr),
                    "}")
            );
            validator.assertWarning(model, LfPackage.eINSTANCE.getHost(), null, "Invalid IP address.");
        }
    }
    
    /**
     * Recognize valid host names and fully qualified names, report invalid ones.
     */
    @Test
    public void recognizeHostNames() throws Exception {
        
        List<String> correct = List.of("localhost"); // FIXME: add more
    
        List<String> validationError = List.of("x.y.z"); // FIXME: add more
        
        List<String> parseError = List.of("..xyz"); // FIXME: add more


        // Correct names.
        for (String addr : correct) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
        }
            
        // Names that don't parse.
        for (String addr : parseError) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            parseWithError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
        }
            
        // Names that parse but are invalid.
        for (String addr : validationError) {
// Java 17:
//         String testCase = """
//             target C;
//             reactor Y {}
//             federated reactor X at foo@%s:4242 {
//                 y = new Y() at %s:2424; 
//             }
//         """.formatted(addr, addr);
// Java 11:
            Model model = parseWithoutError(
                String.join(System.getProperty("line.separator"),
                    "target C;",
                    "reactor Y {}",
                    String.format("federated reactor X at foo@%s:4242 {", addr),
                    String.format("    y = new Y() at %s:2424; ", addr),
                    "}")
            );
            validator.assertWarning(model, LfPackage.eINSTANCE.getHost(), null, 
                "Invalid host name or fully qualified domain name.");
        }
    }
    
    /**
     * Maps a type to a list of known good values.
     */
    Map<PrimitiveType, List<String>> primitiveTypeToKnownGood = Map.of(
        PrimitiveType.BOOLEAN, List.of("true", "\"true\"", "false", "\"false\""),
        PrimitiveType.INTEGER, List.of("0", "1", "\"42\"", "\"-1\"", "-2"),
        PrimitiveType.NON_NEGATIVE_INTEGER, List.of("0", "1", "42"),
        PrimitiveType.TIME_VALUE, List.of("1 msec", "2 sec"),
        PrimitiveType.STRING, List.of("1", "\"foo\"", "bar"),
        PrimitiveType.FILE, List.of("valid.file", "something.json", "\"foobar.proto\"")
    );

    /**
     * Maps a type to a list of known bad values.
     */
    Map<PrimitiveType, List<String>> primitiveTypeToKnownBad = Map.of(
        PrimitiveType.BOOLEAN, List.of("1 sec", "foo", "\"foo\"", "[1]", "{baz: 42}", "'c'"),
        PrimitiveType.INTEGER, List.of("foo", "\"bar\"", "1 sec", "[1, 2]", "{foo: \"bar\"}", "'c'"),
        PrimitiveType.NON_NEGATIVE_INTEGER, List.of("-42", "foo", "\"bar\"", "1 sec", "[1, 2]", "{foo: \"bar\"}", "'c'"),
        PrimitiveType.TIME_VALUE, List.of("foo", "\"bar\"", "\"3 sec\"", "\"4 weeks\"", "[1, 2]", "{foo: \"bar\"}", "'c'"),
        PrimitiveType.STRING, List.of("1 msec", "[1, 2]", "{foo: \"bar\"}", "'c'")
    );

    /**
     * Maps a type to a list, each entry of which represents a list with 
     * three entries: a known wrong value, the suffix to add to the reported
     * name, and the type that it should be.
     */
    Map<TargetPropertyType, List<List<Object>>> compositeTypeToKnownBad = Map.of(
        ArrayType.STRING_ARRAY, List.of(
            List.of("[1 msec]", "[0]", PrimitiveType.STRING),
            List.of("[foo, {bar: baz}]", "[1]", PrimitiveType.STRING),
            List.of("{bar: baz}", "", ArrayType.STRING_ARRAY)
        ),
        UnionType.STRING_OR_STRING_ARRAY, List.of(
            List.of("[1 msec]", "[0]", PrimitiveType.STRING),
            List.of("[foo, {bar: baz}]", "[1]", PrimitiveType.STRING),
            List.of("{bar: baz}", "", UnionType.STRING_OR_STRING_ARRAY)
        ),
        UnionType.FILE_OR_FILE_ARRAY, List.of(
            List.of("[1 msec]", "[0]", PrimitiveType.FILE),
            List.of("[foo, {bar: baz}]", "[1]", PrimitiveType.FILE),
            List.of("{bar: baz}", "", UnionType.FILE_OR_FILE_ARRAY)
        ),
        UnionType.DOCKER_UNION, List.of(
            List.of("foo", "", UnionType.DOCKER_UNION),
            List.of("[1]", "", UnionType.DOCKER_UNION),
            List.of("{bar: baz}", "", DictionaryType.DOCKER_DICT),
            List.of("{FROM: [1, 2, 3]}", ".FROM", PrimitiveType.STRING)
        ),
        UnionType.TRACING_UNION, List.of(
            List.of("foo", "", UnionType.TRACING_UNION),
            List.of("[1]", "", UnionType.TRACING_UNION),
            List.of("{bar: baz}", "", DictionaryType.TRACING_DICT),
            List.of("{trace-file-name: [1, 2, 3]}", ".trace-file-name", PrimitiveType.STRING)
        )
    );
    
    /**
     * Given an array type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    private List<String> synthesizeExamples(ArrayType type, boolean correct) {
        Map<PrimitiveType, List<String>> values = correct ? primitiveTypeToKnownGood : primitiveTypeToKnownBad;
        List<String> examples = new LinkedList<>();
        if (correct) {
            // Produce an array that has an entry for each value.
            List<String> entries = values.get(type.type);
            if (!(entries == null || entries.isEmpty())) {
                examples.add(String.format("[%s]", String.join(", ", entries)));
            }
        }
        return examples;
    }
    
    /**
     * Given an union type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    private List<String> synthesizeExamples(UnionType type, boolean correct) {
        List<String> examples = new LinkedList<>();
        if (correct) {
            for (Enum<?> it : type.options) {
                if (it instanceof TargetPropertyType) {
                    synthesizeExamples((TargetPropertyType) it, correct).forEach(ex -> examples.add(ex));
                } else {
                    examples.add(it.toString());
                }
            }
        } else {
            // Return some obviously bad examples for the common
            // case where the options are from an ordinary Enum<?>.
            if (!type.options.stream().anyMatch(it -> (it instanceof TargetPropertyType))) {
                return List.of("foo", "\"bar\"", "1", "-1", "{x: 42}", "[1, 2, 3]");
            }
        }
        return examples;
    }
    
    /**
     * Given an union type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    private List<String> synthesizeExamples(DictionaryType type, boolean correct) {
        List<String> examples = new LinkedList<>();
        // Produce a set of singleton dictionaries. 
        // If incorrect examples are wanted, garble the key.
        for (DictionaryElement option : type.options) {
            synthesizeExamples(option.getType(), correct).forEach(it -> examples.add("{" + option + (!correct ? "iamwrong: " : ": ") + it + "}"));
        }
        return examples;
    }
    
    /**
     * Synthesize a list of values that either conform to the given type or
     * do not, depending on whether the second argument 'correct' is true.
     * Return an empty set if it is too complicated to generate examples 
     * (e.g., because the resulting errors are more sophisticated).
     * 
     * Not all cases are covered by this function. Currently, the only cases not
     * covered are known bad examples for composite types, which should be added
     * to the compositeTypeToKnownBad map.
     * 
     * @param correct True to synthesize correct examples automatically, otherwise
     *  synthesize incorrect examples.
     */
    private List<String> synthesizeExamples(TargetPropertyType type, boolean correct) {
        if (type instanceof PrimitiveType) {
            Map<PrimitiveType, List<String>> values = correct ? primitiveTypeToKnownGood : primitiveTypeToKnownBad;
            List<String> examples = values.get(type);
            Assertions.assertNotNull(examples);
            return examples;
        } else {
            if (type instanceof UnionType) {
                return synthesizeExamples((UnionType) type, correct);
            } else if (type instanceof ArrayType) {
                return synthesizeExamples((ArrayType) type, correct);
            } else if (type instanceof DictionaryType) {
                return synthesizeExamples((DictionaryType) type, correct);
            } else {
                Assertions.fail("Encountered an unknown type: " + type);
            }
        }
        return new LinkedList<>();
    }
    
    /**
     * Create an LF program with the given key and value as a target property,
     * parse it, and return the resulting model.
     */
    private Model createModel(TargetProperty key, String value) throws Exception {
        String target = key.supportedBy.get(0).getDisplayName();
        System.out.println(String.format("%s: %s", key, value));
// Java 17:
//         Model model = """
//             target %s {%s: %s};
//             reactor Y {}
//             main reactor {
//                 y = new Y() 
//             }
//         """.formatted(target, key, value);
// Java 11:
        return parseWithoutError(String.join(System.getProperty("line.separator"),
            String.format("target %s {%s: %s};", target, key, value),
            "reactor Y {}",
            "main reactor {",
            "    y = new Y() ",
            "}"));
    }

    /**
     * Perform checks on target properties.
     */
    @Test
    public void checkTargetProperties() throws Exception {
        for (TargetProperty prop : TargetProperty.getOptions()) {
            if (prop == TargetProperty.CARGO_DEPENDENCIES) {
                // we test that separately as it has better error messages
                return;
            }
            System.out.println(String.format("Testing target property %s which is %s", prop, prop.type));
            System.out.println("====");
            System.out.println("Known good assignments:");
            List<String> knownCorrect = synthesizeExamples(prop.type, true);
            
            for (String it : knownCorrect) {
                Model model = createModel(prop, it);
                validator.assertNoErrors(model);
                // Also make sure warnings are produced when files are not present.
                if (prop.type == PrimitiveType.FILE) {
                    validator.assertWarning(model, LfPackage.eINSTANCE.getKeyValuePair(),
                    null, String.format("Could not find file: '%s'.", withoutQuotes(it)));
                }
            }
            
            // Extra checks for filenames. (This piece of code was commented out in the original xtend file)
            // Temporarily disabled because we need a more sophisticated check that looks for files in different places.
//            if (prop.type == prop.type == ArrayType.FILE_ARRAY ||
//                prop.type == UnionType.FILE_OR_FILE_ARRAY) {
//                val model = prop.createModel(
//                    synthesizeExamples(ArrayType.FILE_ARRAY, true).get(0))
//                primitiveTypeToKnownGood.get(PrimitiveType.FILE).forEach [
//                    model.assertWarning(
//                        LfPackage.eINSTANCE.keyValuePair,
//                        null, '''Could not find file: '«it.withoutQuotes»'.''')
//                ]
//            }
            
            System.out.println("Known bad assignments:");
            List<String> knownIncorrect = synthesizeExamples(prop.type, false);
            if (!(knownIncorrect == null || knownIncorrect.isEmpty())) {
                for (String it : knownIncorrect) {
                    validator.assertError(createModel(prop, it), 
                        LfPackage.eINSTANCE.getKeyValuePair(), null, 
                        String.format("Target property '%s' is required to be %s.", prop.toString(), prop.type));
                }
            } else {
                // No type was synthesized. It must be a composite type.
                List<List<Object>> list = compositeTypeToKnownBad.get(prop.type);
                if (list == null) {
                    System.out.println(String.format("No known incorrect values provided for target property '%s'. Aborting test.", prop));
                    Assertions.assertTrue(false);
                } else {
                    for (List<Object> it : list) {
                        validator.assertError(createModel(prop, it.get(0).toString()),
                            LfPackage.eINSTANCE.getKeyValuePair(), null,
                            String.format("Target property '%s%s' is required to be %s.", prop.toString(), it.get(1), it.get(2)));
                    }
                }
            }    
            System.out.println("====");
        }
        System.out.println("Done!");
    }


    @Test
    public void checkCargoDependencyProperty() throws Exception {
        TargetProperty prop = TargetProperty.CARGO_DEPENDENCIES;
        List<String> knownCorrect = List.of("{}", "{ dep: \"8.2\" }", "{ dep: { version: \"8.2\"} }", "{ dep: { version: \"8.2\", features: [\"foo\"]} }");
        for (String it : knownCorrect) {
            validator.assertNoErrors(createModel(prop, it));
        }

        //                                               vvvvvvvvvvv
        validator.assertError(createModel(prop, "{ dep: {/*empty*/} }"),
            LfPackage.eINSTANCE.getKeyValuePairs(), null, "Must specify one of 'version', 'path', or 'git'"
        );
        
        //                                                vvvvvvvvvvv
        validator.assertError(createModel(prop, "{ dep: { unknown_key: \"\"} }"),
            LfPackage.eINSTANCE.getKeyValuePair(), null, "Unknown key: 'unknown_key'"
        );

        //                                                          vvvv
        validator.assertError(createModel(prop, "{ dep: { features: \"\" } }"),
            LfPackage.eINSTANCE.getElement(), null, "Expected an array of strings for key 'features'"
        );
    }

    @Test
    public void testImportedCyclicReactor() throws Exception {
        // File tempFile = File.createTempFile("lf-validation", ".lf");
        // tempFile.deleteOnExit();
        // // Java 17:
        // //         String fileToBeImported = """
        // //             target C;
        // //             reactor A {
        // //                 a = new A();
        // //             }
        // //         """
        // // Java 11:
        // String fileToBeImported = String.join(System.getProperty("line.separator"),
        //     "target C;",
        //     "reactor A {",
        //     "    a = new A();",
        //     "}"
        // );
        // BufferedWriter writer = new BufferedWriter(new FileWriter(tempFile));
        // writer.write(fileToBeImported);
        // writer.close();

        // // Java 17:
        // //         String testCase = """
        // //             target C;
        // //             import A from ...
        // //             main reactor {
        // //             }
        // //         """
        // // Java 11:
        // String testCase = String.join(System.getProperty("line.separator"),
        //     "target C;",
        //     String.format("import A from \"%s\"", tempFile.getAbsolutePath()),
        //     "main reactor {",
        //     "}"
        // );
        // Model model = parseWithoutError(testCase);
        // TODO: Uncomment the lines below and resolve the weird error. (java.lang.IllegalArgumentException: resolve against non-hierarchical or relative base)
        // validator.assertError(model, LfPackage.eINSTANCE.getImportedReactor(), null, "Imported reactor 'A' has cyclic instantiation in it.");
    }

    @Test
    public void testUnusedImport() throws Exception {
        // File tempFile = File.createTempFile("lf-validation", ".lf");
        // tempFile.deleteOnExit();
        // // Java 17:
        // //         String fileToBeImported = """
        // //             target C;
        // //             reactor A {}
        // //         """
        // // Java 11:
        // String fileToBeImported = String.join(System.getProperty("line.separator"),
        //     "target C;",
        //     "reactor A{}"
        // );
        // BufferedWriter writer = new BufferedWriter(new FileWriter(tempFile));
        // writer.write(fileToBeImported);
        // writer.close();

        // // Java 17:
        // //         String testCase = """
        // //             target C;
        // //             import A from ...
        // //             main reactor {}
        // //         """
        // // Java 11:
        // String testCase = String.join(System.getProperty("line.separator"),
        //     "target C;",
        //     String.format("import A from \"%s\"", tempFile.getAbsolutePath()),
        //     "main reactor{}"
        // );
        // Model model = parseWithoutError(testCase);
        // TODO: Uncomment the lines below and resolve the weird error. (java.lang.IllegalArgumentException: resolve against non-hierarchical or relative base)
        // validator.assertWarning(model, LfPackage.eINSTANCE.getImport(), null, "Unused import.");
        // validator.assertWarning(parseWithoutError(testCase), LfPackage.eINSTANCE.getImportedReactor(), null, "Unused reactor class.");
    }

    @Test
    public void testMissingInputType() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 input i;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    input i;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInput(), null,
            "Input must have a type.");
    }

    @Test
    public void testMissingOutputType() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 output i;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    output i;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getOutput(), null,
            "Output must have a type.");
    }

    @Test
    public void testMissingStateType() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 state i;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    state i;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getStateVar(), null,
            "State must have a type.");
    }

    @Test
    public void testListWithParam() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor (A:int(1)) { state i:int(A, 2, 3) }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor (A:int(1)) { state i:int(A, 2, 3) }"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getStateVar(), null,
            "List items cannot refer to a parameter.");
    }

    @Test
    public void testCppMutableInput() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target Cpp;
        //             main reactor {
        //                 mutable input i:int;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target Cpp;",
            "main reactor {",
            "    mutable input i:int;",
            "}"
        );
        validator.assertWarning(parseWithoutError(testCase), LfPackage.eINSTANCE.getInput(), null,
            "The mutable qualifier has no meaning for the C++ target and should be removed. " +
            "In C++, any value can be made mutable by calling get_mutable_copy().");
    }

    @Test
    public void testOverflowingSTP() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 reaction(startup) {==} STP(2147483648) {==}
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    reaction(startup) {==} STP(2147483648) {==}",
            "}"
        );

        // TODO: Uncomment and fix failing test. See issue #903 on Github.
        // validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getSTP(), null,
            // "STP offset exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.");
    }

    @Test
    public void testOverflowingDeadline() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 reaction(startup) {==} STP(2147483648) {==}
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    reaction(startup) {==} deadline(2147483648) {==}",
            "}"
        );

        // TODO: Uncomment and fix failing test. See issue #903 on Github.
        // validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getDeadline(), null,
            // "Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.");
    }

    @Test
    public void testInvalidTargetParam() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C { beefyDesktop: true }
        //             main reactor {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C { beefyDesktop: true }",
            "main reactor {}"
        );
        List<Issue> issues = validator.validate(parseWithoutError(testCase));
        Assertions.assertTrue(issues.size() == 1 && issues.get(0).getMessage().contains("Unrecognized target parameter: beefyDesktop"));
    }

    @Test
    public void testTargetParamNotSupportedForTarget() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target Python { build: "" }
        //             main reactor {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target Python { build: \"\" }",
            "main reactor {}"
        );
        List<Issue> issues = validator.validate(parseWithoutError(testCase));
        Assertions.assertTrue(issues.size() == 1 && issues.get(0).getMessage().contains("The target parameter: build" +
            " is not supported by the Python target and will thus be ignored."));
    }

    @Test
    public void testUnnamedReactor() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             reactor {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "reactor {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Reactor must be named.");
    }

    @Test
    public void testMainHasInput() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 input x:int;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    input x:int;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInput(), null,
            "Main reactor cannot have inputs.");
    }

    @Test
    public void testFederatedHasInput() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             federated reactor {
        //                 input x:int;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "federated reactor {",
            "    input x:int;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getInput(), null,
            "Main reactor cannot have inputs.");
    }

    @Test
    public void testMainHasOutput() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {
        //                 output x:int;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    output x:int;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getOutput(), null,
            "Main reactor cannot have outputs.");
    }

    @Test
    public void testFederatedHasOutput() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             federated reactor {
        //                 output x:int;
        //             }
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "federated reactor {",
            "    output x:int;",
            "}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getOutput(), null,
            "Main reactor cannot have outputs.");
    }

    @Test
    public void testMultipleMainReactor() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor A {}
        //             main reactor A {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor A {}",
            "main reactor A {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Multiple definitions of main or federated reactor.");
    }

    @Test
    public void testMultipleMainReactorUnnamed() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor {}
        //             main reactor {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor {}",
            "main reactor {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Multiple definitions of main or federated reactor.");
    }

    @Test
    public void testMultipleFederatedReactor() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             federated reactor A {}
        //             federated reactor A {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "federated reactor A {}",
            "federated reactor A {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Multiple definitions of main or federated reactor.");
    }

    @Test
    public void testMultipleMainOrFederatedReactor() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             federated reactor A {}
        //             federated reactor A {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor A {}",
            "federated reactor A {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getReactor(), null,
            "Multiple definitions of main or federated reactor.");
    }

    @Test
    public void testMainReactorHasHost() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target C;
        //             main reactor at 127.0.0.1{}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target C;",
            "main reactor at 127.0.0.1{}"
        );
        // TODO: Uncomment and fix test
        // List<Issue> issues = validator.validate(parseWithoutError(testCase));
        // Assertions.assertTrue(issues.size() == 1 && 
        //     issues.get(0).getMessage().contains("Cannot assign a host to reactor '") &&
        //     issues.get(0).getMessage().contains("' because it is not federated."));
    }

    @Test
    public void testUnrecognizedTarget() throws Exception {
        // Java 17:
        //         String testCase = """
        //             target Pjthon;
        //             main reactor {}
        //         """
        // Java 11:
        String testCase = String.join(System.getProperty("line.separator"),
            "target Pjthon;",
            "main reactor {}"
        );
        validator.assertError(parseWithoutError(testCase), LfPackage.eINSTANCE.getTargetDecl(), null,
            "Unrecognized target: Pjthon");
    }

    
}


