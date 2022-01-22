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
import java.util.List;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.eclipse.xtext.testing.validation.ValidationTestHelper;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.DictionaryType;
import org.lflang.TargetProperty.PrimitiveType;
import org.lflang.TargetProperty.TargetPropertyType;
import org.lflang.TimeValue;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Model;
import org.lflang.lf.Visibility;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import static extension org.lflang.ASTUtils.*;
import org.lflang.TargetProperty.UnionType;
import org.lflang.TargetProperty.ArrayType;
import org.lflang.tests.LFInjectorProvider;

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
    private Model parseWithoutError(String s) {
        Model model = parser.parse(s);
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing: " +
                model.eResource.errors);
        return model;
    }

    /**
     * Helper function to parse a Lingua Franca program and expect errors.
     * @return A model representing the parsed string.
     */
    private Model parseWithError(String s) {
        Model model = parser.parse(s);
        Assertions.assertNotNull(model);
        Assertions.assertFalse(model.eResource.errors.isEmpty);
        return model;
    } 


    /**
     * Ensure that duplicate identifiers for actions reported.
     */
    @Test
    public void duplicateVariable() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.action, null,
        "Duplicate Variable 'bar' in Reactor 'Foo'");
    }


    /**
     * Check that reactors in C++ cannot be named preamble 
     */
    @Test
    public void disallowReactorCalledPreamble() {
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
        Assertions.assertTrue(model_no_errors.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_no_errors.eResource.errors);
            Assertions.assertTrue(model_error_1.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_error_1.eResource.errors);
            Assertions.assertTrue(model_error_2.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_error_2.eResource.errors);

        model_no_errors.assertNoIssues();
        model_error_1.assertError(LfPackage::eINSTANCE.reactor, null,
            "Reactor cannot be named 'Preamble'");
        model_error_2.assertError(LfPackage::eINSTANCE.reactor, null,
            "Reactor cannot be named 'Preamble'");
    }


    /**
     * Ensure that "__" is not allowed at the start of an input name.
     */
    @Test
    public void disallowUnderscoreInputs() {
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

        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.input, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    @Test
    public void disallowMainWithDifferentNameThanFile() {
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

        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.reactor, null,
            "Name of main reactor must match the file name (or be omitted)");
    }
    

    /**
     * Ensure that "__" is not allowed at the start of an output name.
     */
    @Test
    public void disallowUnderscoreOutputs() {
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

        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.output, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }

    /**
     * Ensure that "__" is not allowed at the start of an action name.
     */
    @Test
    public void disallowUnderscoreActions() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.action, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a timer name.
     */
    @Test
    public void disallowUnderscoreTimers() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.timer, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a parameter name.
     */
    @Test
    public void disallowUnderscoreParameters() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.parameter, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of an state name.
     */
    @Test
    public void disallowUnderscoreStates() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.stateVar, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor definition name.
     */
    @Test
    public void disallowUnderscoreReactorDef() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.reactor, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __Foo");
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor instantiation name.
     */
    @Test
    public void disallowUnderscoreReactorInstantiation() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.instantiation, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __x");
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    public void connectionToEffectPort() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'out' is already effect of a reaction.");
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    public void connectionToEffectPort2() {
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
        "   x = new Foo();"
        "   y = new Foo();",
        "",
        "   y.out -> x.inp;",
        "   reaction(startup) -> x.inp {=",                    
        "   =}",
        "}");
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'inp' is already effect of a reaction.");
    }
    
    /**
     * Allow connection to the port of a contained reactor if another port with same name is effect of a reaction.
     */
    @Test
    public void connectionToEffectPort3() {
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
        "   x1 = new Foo();"
        "   x2 = new Foo();",
        "   in -> x1.in;",
        "   reaction(startup) -> x2.in {=",
        "   =}",
        "}");
        parseWithoutError(testCase).assertNoErrors();
    }

    /**
     * Disallow connection to the port of a contained reactor if the same port is effect of a reaction.
     */
    @Test
    public void connectionToEffectPort4() {
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
        "   x1 = new Foo();"
        "   in -> x1.in;",
        "   reaction(startup) -> x1.in {=",
        "   =}",
        "}");
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'in' is already effect of a reaction.");
    }

    /**
     * Disallow connection of multiple ports to the same input port.
     */
    @Test
    public void multipleConnectionsToInputTest() {
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
        "}"
        "main reactor {",
        "   input in:int;",
        "   src = new Source();",
        "   sink = new Sink();",
        "   in -> sink.in;",
        "   src.out -> sink.in;",
        "}");
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'in' may only appear once on the right side of a connection.");
    }

    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    public void detectInstantiationCycle() {
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
        parseWithoutError(testCase).assertError(LfPackage::eINSTANCE.instantiation,
            null, "Instantiation is part of a cycle: Contained");
    }
    
    
    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    public void detectInstantiationCycle2() {
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
        model.assertError(LfPackage::eINSTANCE.instantiation,
            null, "Instantiation is part of a cycle: Intermediate, Contained.");
        model.assertError(LfPackage::eINSTANCE.instantiation,
            null, "Instantiation is part of a cycle: Intermediate, Contained.");
    }
    
    /**
     * Detect causality loop.
     */
    @Test
    public void detectCausalityLoop() {
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
            "   input x:int",
            "   output y:int;",
            "   reaction(x) -> y {=",
            "   =}",
            "}",
            "",
            "main reactor {",
            "   a = new X()",
            "   a = new X()",
            "   a.y -> b.x",
            "   b.y -> a.x",
            "}"
        ));
        model.assertError(LfPackage::eINSTANCE.reaction,
            null, "Reaction triggers involved in cyclic dependency in reactor X: x.");
        model.assertError(LfPackage::eINSTANCE.reaction,
            null, "Reaction effects involved in cyclic dependency in reactor X: y.");
            
    }
    
    /**
     * Let cyclic dependencies be broken by "after" clauses.
     */
    @Test
    public void afterBreaksCycle() {
        parseWithoutError('''
            target C
            
            reactor X {
                input x:int;
                output y:int;
                reaction(x) -> y {=
                =}
            }
            
            main reactor {
                a = new X()
                b = new X()
                a.y -> b.x after 5 msec
                b.y -> a.x
            }
            
        ''').assertNoErrors()
            
    }


    /**
     * Let cyclic dependencies be broken by "after" clauses with zero delay.
     */
    @Test
    public void afterBreaksCycle2() {
        parseWithoutError('''
            target C
            
            reactor X {
                input x:int;
                output y:int;
                reaction(x) -> y {=
                =}
            }
            
            main reactor {
                a = new X()
                b = new X()
                a.y -> b.x after 0 sec
                b.y -> a.x
            }
            
        ''').assertNoErrors()
            
    }


    /**
     * Let cyclic dependencies be broken by "after" clauses with zero delay and no units.
     */
    @Test
    public void afterBreaksCycle3() {
        parseWithoutError('''
            target C
            
            reactor X {
                input x:int;
                output y:int;
                reaction(x) -> y {=
                =}
            }
            
            main reactor {
                a = new X()
                b = new X()
                a.y -> b.x after 0
                b.y -> a.x
            }
            
        ''').assertNoErrors()
            
    }

    /**
     * Detect missing units in "after" clauses with delay greater than zero.
     */
    @Test
    public void nonzeroAfterMustHaveUnits() {
        parseWithoutError('''
            target C
            
            reactor X {
                input x:int;
                output y:int;
                reaction(x) -> y {=
                =}
            }
            
            main reactor {
                a = new X()
                b = new X()
                a.y -> b.x after 1
            }
            
        ''').assertError(LfPackage::eINSTANCE.time,
            null, 'Missing time unit.')
            
    }


    
    /**
     * Report non-zero time value without units.
     */
    @Test
    public void nonZeroTimeValueWithoutUnits() {
        parseWithoutError('''
            target C;
              main reactor {
                  timer t(42, 1 sec);
                  reaction(t) {=
                      printf("Hello World.\n");
                  =}
             }
        ''').assertError(LfPackage::eINSTANCE.value, null, "Missing time unit.")
    }    
    
    /**
     * Report reference to non-time parameter in time argument.
     */
    @Test
    public void parameterTypeMismatch() {
        parseWithoutError('''
            target C;
              main reactor (p:int(0)) {
                  timer t(p, 1 sec);
                  reaction(t) {=
                      printf("Hello World.\n");
                  =}
             }
        ''').assertError(LfPackage::eINSTANCE.value,
            null, 'Parameter is not of time type')
        
    }
    
    /**
     * Report inappropriate literal in time argument.
     */
    @Test
    public void targetCodeInTimeArgument() {
        parseWithoutError('''
            target C;
            main reactor {
                timer t({=foo()=}, 1 sec);
                reaction(t) {=
                    printf("Hello World.\n");
                =}
            }
        ''').assertError(LfPackage::eINSTANCE.value,
            null, 'Invalid time literal')
    }  
    

    /**
     * Report overflowing deadline.
     */
    @Test
    public void overflowingDeadlineC() {
        parseWithoutError('''
            target C;
            main reactor {
            timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (40 hours) {=
                =}
            }
        ''').assertError(LfPackage::eINSTANCE.deadline, null,
            "Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE +
                " nanoseconds.")
    }  

    
    /**
     * Report overflowing parameter.
     */
    @Test
    public void overflowingParameterC() {
        parseWithoutError('''
            target C;
            main reactor(d:time(40 hours)) {
            timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (d) {=
                =}
            }
        ''').assertError(LfPackage::eINSTANCE.parameter, null,
            "Time value used to specify a deadline exceeds the maximum of " +
                TimeValue.MAX_LONG_DEADLINE + " nanoseconds.")
    }  
    
    
    /**
     * Report overflowing assignment.
     */
    @Test
    public void overflowingAssignmentC() {
        parseWithoutError('''
            target C;
            reactor Print(d:time(39 hours)) {
                timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (d) {=
                =}
            }
            main reactor {
                p = new Print(d=40 hours);
            }
        ''').assertError(LfPackage::eINSTANCE.assignment, null,
            "Time value used to specify a deadline exceeds the maximum of " +
                        TimeValue.MAX_LONG_DEADLINE + " nanoseconds.")
    }  

    /**
     * Report missing trigger.
     */
    @Test
    public void missingTrigger() {
        parseWithoutError('''
		target C;
		reactor X {
		   	reaction() {=
		   		//
		   	=}
		}
        ''').assertWarning(LfPackage::eINSTANCE.reaction, null,
            "Reaction has no trigger.")
    }
        
    /**
     * Test warnings and errors for the target dependent preamble visibility qualifiers 
     */
    @Test
    public void testPreambleVisibility() {
        for (target : Target.values) {
            for (visibility : Visibility.values) {
                val model_reactor_scope = parseWithoutError('''
                    target «target»;
                    reactor Foo {
                        «IF visibility != Visibility.NONE»«visibility» «ENDIF»preamble {==}
                    }
                ''')
                
                val model_file_scope = parseWithoutError('''
                    target «target»;
                    «IF visibility != Visibility.NONE»«visibility» «ENDIF»preamble {==}
                    reactor Foo {
                    }
                ''')
                
                val model_no_preamble = parseWithoutError('''
                    target «target»;
                    reactor Foo {
                    }
                ''')
                
                model_no_preamble.assertNoIssues
                
                if (target == Target.CPP) {
                    if (visibility == Visibility.NONE) {
                        model_file_scope.assertError(LfPackage::eINSTANCE.preamble, null,
                            "Preambles for the C++ target need a visibility qualifier (private or public)!")
                        model_reactor_scope.assertError(LfPackage::eINSTANCE.preamble, null,
                            "Preambles for the C++ target need a visibility qualifier (private or public)!")        
                    } else {
                        model_file_scope.assertNoIssues
                        model_reactor_scope.assertNoIssues
                    }
                } else {
                    if (visibility == Visibility.NONE) {
                        model_file_scope.assertNoIssues
                        model_reactor_scope.assertNoIssues        
                    } else {
                        model_file_scope.assertWarning(LfPackage::eINSTANCE.preamble, null,
                            '''The «visibility» qualifier has no meaning for the «target.name» target. It should be removed.''')
                        model_reactor_scope.assertWarning(LfPackage::eINSTANCE.preamble, null,
                            '''The «visibility» qualifier has no meaning for the «target.name» target. It should be removed.''')
                    }
                }
            }
        }
    }
    
    
    /**
     * Tests for state and parameter declarations, including native lists.
     */
    @Test
    public void stateAndParameterDeclarationsInC() {
        val model = parseWithoutError('''
			target C;
			reactor Bar(a(0),			// ERROR: type missing
						b:int,			// ERROR: uninitialized
						t:time(42), 	// ERROR: units missing
						x:int(0),
						h:time("bla"), 	// ERROR: not a type 
						q:time(1 msec, 2 msec),  // ERROR: not a list
						y:int(t)		// ERROR: init using parameter
			) {
				state offset:time(42); 	// ERROR: units missing
				state w:time(x);		// ERROR: parameter is not a time
				state foo:time("bla");	// ERROR: assigned value not a time
				timer tick(1);			// ERROR: not a time
			}
        ''')

		model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Type declaration missing.")
        model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Missing time unit.")
        model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Invalid time literal.")
        model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Time parameter cannot be initialized using a list.")    
        model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Parameter cannot be initialized using parameter.")
        model.assertError(LfPackage::eINSTANCE.stateVar, null,
            "Referenced parameter does not denote a time.")
        model.assertError(LfPackage::eINSTANCE.stateVar, null,
            "Invalid time literal.")
        model.assertError(LfPackage::eINSTANCE.parameter, null,
            "Uninitialized parameter.")
       	model.assertError(LfPackage::eINSTANCE.value, null,
            "Missing time unit.")
    }  
    
    
    /**
     * Recognize valid IPV4 addresses, report invalid ones.
     */
    @Test
    public void recognizeIPV4() {
        
        val correct = #["127.0.0.1", "10.0.0.1", "192.168.1.1", "0.0.0.0",
            "192.168.1.1"]
        val parseError = #["10002.3.4", "1.2.3.4.5"]
        val validationError = #["256.0.0.0", "260.0.0.0"]

        // Correct IP addresses.
        correct.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''')
        ]

        // IP addresses that don't parse.
        parseError.forEach [ addr |
            parseWithError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''')
        ]

        // IP addresses that parse but are invalid.
        validationError.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''').assertWarning(LfPackage::eINSTANCE.host, null,
                "Invalid IP address.")
        ]
    }
    
    /**
     * Recognize valid IPV6 addresses, report invalid ones.
     */
    @Test
    public void recognizeIPV6() {
        
        val correct = #["1:2:3:4:5:6:7:8", "1:2:3:4:5:6:7::", "1:2:3:4:5:6::8",
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
            "2001:db8:3:4::192.0.2.33", "64:ff9b::192.0.2.33", "0:0:0:0:0:0:10.0.0.1"] 
        
        val validationError = #["1:2:3:4:5:6:7:8:9", "1:2:3:4:5:6::7:8",
            "1:2:3:4:5:6:7:8:", "::1:2:3:4:5:6:7:8", "1:2:3:4:5:6:7:8::",
            "1:2:3:4:5:6:7:88888", "2001:db8:3:4:5::192.0.2.33",
            "fe08::7:8interface", "fe08::7:8interface", "fe08::7:8i"]
            
        val parseError = #["fe08::7:8%", ":1:2:3:4:5:6:7:8"]
        
        // Correct IP addresses.
        correct.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''').assertNoIssues()
        ]
        
        // IP addresses that don't parse.
        parseError.forEach [ addr |
            parseWithError('''
                target C;
                reactor Y {}
                federated reactor at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''')
        ]
        
        // IP addresses that parse but are invalid.
        validationError.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''').assertWarning(LfPackage::eINSTANCE.host, null,
                "Invalid IP address.")
        ]
    }
    
    /**
     * Recognize valid host names and fully qualified names, report invalid ones.
     */
    @Test
    public void recognizeHostNames() {
        
        val correct = #["localhost"] // FIXME: add more
    
        val validationError = #["x.y.z"] // FIXME: add more
        
        val parseError = #["..xyz"] // FIXME: add more
                
        // Correct names.
        correct.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''').assertNoIssues()
        ]
        
        // Names that don't parse.
        parseError.forEach [ addr |
            parseWithError('''
                target C;
                reactor Y {}
                federated reactor at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''')
        ]
        
        // Names that parse but are invalid.
        validationError.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''').assertWarning(LfPackage::eINSTANCE.host, null,
                "Invalid host name or fully qualified domain name.")
        ]
    }
    
    /**
     * Maps a type to a list of known good values.
     */
    val primitiveTypeToKnownGood = #{
            PrimitiveType.BOOLEAN -> #["true", "\"true\"", "false", "\"false\""],
            PrimitiveType.INTEGER -> #["0", "1", "\"42\"", "\"-1\"", "-2"],
            PrimitiveType.NON_NEGATIVE_INTEGER -> #["0", "1", "42"],
            PrimitiveType.TIME_VALUE -> #["1 msec", "2 sec"],
            PrimitiveType.STRING -> #["1", "\"foo\"", "bar"],
            PrimitiveType.FILE -> #["valid.file", "something.json", "\"foobar.proto\""]
        }
    
    /**
     * Maps a type to a list of known bad values.
     */
    val primitiveTypeToKnownBad = #{
            PrimitiveType.BOOLEAN -> #["1 sec", "foo", "\"foo\"", "[1]", "{baz: 42}", "'c'"],
            PrimitiveType.INTEGER -> #["foo", "\"bar\"", "1 sec", "[1, 2]", "{foo: \"bar\"}", "'c'"],
            PrimitiveType.NON_NEGATIVE_INTEGER -> #["-42", "foo", "\"bar\"", "1 sec", "[1, 2]", "{foo: \"bar\"}", "'c'"],
            PrimitiveType.TIME_VALUE -> #["foo", "\"bar\"", "\"3 sec\"", "\"4 weeks\"", "[1, 2]", "{foo: \"bar\"}", "'c'"],
            PrimitiveType.STRING -> #["1 msec", "[1, 2]", "{foo: \"bar\"}", "'c'"]
        }
    
    /**
     * Maps a type to a list, each entry of which represents a list with 
     * three entries: a known wrong value, the suffix to add to the reported
     * name, and the type that it should be.
     */
    val compositeTypeToKnownBad = #{
        ArrayType.STRING_ARRAY -> #[
            #["[1 msec]", "[0]", PrimitiveType.STRING],
            #["[foo, {bar: baz}]", "[1]", PrimitiveType.STRING],
            #["{bar: baz}", "", ArrayType.STRING_ARRAY]
        ],
        UnionType.STRING_OR_STRING_ARRAY -> #[
            #["[1 msec]", "[0]", PrimitiveType.STRING],
            #["[foo, {bar: baz}]", "[1]", PrimitiveType.STRING],
            #["{bar: baz}", "", UnionType.STRING_OR_STRING_ARRAY]
        ],
        UnionType.FILE_OR_FILE_ARRAY -> #[
            #["[1 msec]", "[0]", PrimitiveType.FILE],
            #["[foo, {bar: baz}]", "[1]", PrimitiveType.FILE],
            #["{bar: baz}", "", UnionType.FILE_OR_FILE_ARRAY]
        ],
        UnionType.DOCKER_UNION -> #[
            #["foo", "", UnionType.DOCKER_UNION],
            #["[1]", "", UnionType.DOCKER_UNION],
            #["{bar: baz}", "", DictionaryType.DOCKER_DICT],
            #["{FROM: [1, 2, 3]}", ".FROM", PrimitiveType.STRING]
        ],
        UnionType.TRACING_UNION -> #[
            #["foo", "", UnionType.TRACING_UNION],
            #["[1]", "", UnionType.TRACING_UNION],
            #["{bar: baz}", "", DictionaryType.TRACING_DICT],
            #["{trace-file-name: [1, 2, 3]}", ".trace-file-name", PrimitiveType.STRING]
        ]
    }
    
    /**
     * Given an array type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    def List<String> synthesizeExamples(ArrayType type, boolean correct) {
        val values = correct ? primitiveTypeToKnownGood : primitiveTypeToKnownBad
        val examples = newLinkedList
                if (correct) {
                    // Produce an array that has an entry for each value.
                    val entries = values.get(type.type)
                    if (!entries.nullOrEmpty) {
                        examples.add('''[«entries.join(', ')»]''')
                    }
                }
                return examples
    }
    
    /**
     * Given an union type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    def List<String> synthesizeExamples(UnionType type, boolean correct) {
        val examples = newLinkedList
        if (correct) {
            type.options.forEach [
                if (it instanceof TargetPropertyType) {
                    synthesizeExamples(it, correct).forEach [
                        examples.add(it)
                    ]
                } else {
                    // It must be a plain Enum<?>
                    examples.add(it.toString())
                }
            ]
        } else {
            // Return some obviously bad examples for the common
            // case where the options are from an ordinary Enum<?>.
            if (!type.options.exists[it instanceof TargetPropertyType]) {
                return #["foo", "\"bar\"", "1", "-1",
                    "{x: 42}", "[1, 2, 3]"]
            }
        }
        return examples
    }
    
    /**
     * Given an union type, return a list of good or bad examples, 
     * depending on the value of the second parameter.
     */
    def List<String> synthesizeExamples(DictionaryType type, boolean correct) {
        val examples = newLinkedList
        // Produce a set of singleton dictionaries. 
        // If incorrect examples are wanted, garble the key.
        type.options.forEach [ option |
            synthesizeExamples(option.type, correct).forEach [
                examples.add('''{«option»«!correct? "iamwrong"»: «it»}''')
            ]
        ]
        return examples
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
    def List<String> synthesizeExamples(TargetPropertyType type, boolean correct) {
        if (type instanceof PrimitiveType) {
            val values = correct ? primitiveTypeToKnownGood : primitiveTypeToKnownBad
            val examples = values.get(type).toList
            assertNotNull(examples)
            return examples
        } else {
            if (type instanceof UnionType) {
                return synthesizeExamples(type, correct)
            } else if (type instanceof ArrayType) {
                return synthesizeExamples(type, correct)
            } else if (type instanceof DictionaryType) {
                return synthesizeExamples(type, correct)
            } else {
                fail("Encountered an unknown type: " + type)
            }
        }
        return #[]
    }
    
    /**
     * Create an LF program with the given key and value as a target property,
     * parse it, and return the resulting model.
     */
    def createModel(TargetProperty key, String value) {
        val target = key.supportedBy.get(0).displayName
        println('''«key»: «value»''')
        return parseWithoutError('''
                target «target» {«key»: «value»};
                reactor Y {}
                main reactor {
                    y = new Y() 
                }
            ''')
    }

    /**
     * Perform checks on target properties.
     */
    @Test
    public void checkTargetProperties() {
        
        for (prop : TargetProperty.options) {
            if (prop == TargetProperty.CARGO_DEPENDENCIES) {
                // we test that separately as it has better error messages
                return
            }
            println('''Testing target property «prop» which is «prop.type»''')
            println("====")
            println("Known good assignments:")
            val knownCorrect = synthesizeExamples(prop.type, true)
            knownCorrect.forEach [
                val model = prop.createModel(it)
                model.assertNoErrors()
                // Also make sure warnings are produced when files are not present.
                if (prop.type == PrimitiveType.FILE) {
                    model.assertWarning(
                        LfPackage::eINSTANCE.keyValuePair,
                        null, '''Could not find file: '«it.withoutQuotes»'.''')
                }
            ]
            // Extra checks for filenames.
            // Temporarily disabled because we need a more sophisticated check that looks for files in different places.
//            if (prop.type == prop.type == ArrayType.FILE_ARRAY ||
//                prop.type == UnionType.FILE_OR_FILE_ARRAY) {
//                val model = prop.createModel(
//                    synthesizeExamples(ArrayType.FILE_ARRAY, true).get(0))
//                primitiveTypeToKnownGood.get(PrimitiveType.FILE).forEach [
//                    model.assertWarning(
//                        LfPackage::eINSTANCE.keyValuePair,
//                        null, '''Could not find file: '«it.withoutQuotes»'.''')
//                ]
//            }
            
            println("Known bad assignments:") 
            val knownIncorrect = synthesizeExamples(prop.type, false)
            if (!knownIncorrect.isNullOrEmpty) {
                knownIncorrect.forEach [
                    prop.createModel(it).assertError(
                        LfPackage::eINSTANCE.keyValuePair,
                        null, '''Target property '«prop.toString»' is required to be «prop.type».''')
                ]
            } else {
                // No type was synthesized. It must be a composite type.
                val list = compositeTypeToKnownBad.get(prop.type)
                if (list === null) {
                    println('''No known incorrect values provided for target property '«prop»'. Aborting test.''')
                    assertTrue(false)
                } else {
                    list.forEach [
                        prop.createModel(it.get(0).toString).
                            assertError(
                                LfPackage::eINSTANCE.keyValuePair,
                                null, '''Target property '«prop.toString»«it.get(1)»' is required to be «it.get(2)».''')
                    ]
                }
            }            
            println("====")
        }
        println("Done!")
    }


    @Test
    public void checkCargoDependencyProperty() {
         val prop = TargetProperty.CARGO_DEPENDENCIES
         val knownCorrect = #[ "{}", "{ dep: \"8.2\" }", "{ dep: { version: \"8.2\"} }", "{ dep: { version: \"8.2\", features: [\"foo\"]} }" ]
         knownCorrect.forEach [
            prop.createModel(it).assertNoErrors()
        ]

        //                       vvvvvvvvvvv
        prop.createModel("{ dep: {/*empty*/} }")
            .assertError(LfPackage::eINSTANCE.keyValuePairs, null, "Must specify one of 'version', 'path', or 'git'")

        //                         vvvvvvvvvvv
        prop.createModel("{ dep: { unknown_key: \"\"} }")
            .assertError(LfPackage::eINSTANCE.keyValuePair, null, "Unknown key: 'unknown_key'")

        //                                   vvvv
        prop.createModel("{ dep: { features: \"\" } }")
            .assertError(LfPackage::eINSTANCE.element, null, "Expected an array of strings for key 'features'")
    }
}



