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
import org.eclipse.xtext.testing.validation.ValidationTestHelper
import org.icyphy.Targets
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Visibility
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import java.text.Normalizer
import org.eclipse.xtext.diagnostics.Diagnostic

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)
/**
 * Collection of unit tests to ensure validation is done correctly.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author(Christian Menard <christian.menard@tu-dresden.de>}
 */
class LinguaFrancaValidationTest {
	@Inject extension ParseHelper<Model>
    @Inject extension ValidationTestHelper

    /**
     * Helper function to parse a Lingua Franca program and expect no errors.
     * @return A model representing the parsed string.
     */
    def parseWithoutError(String s) {
        val model = s.parse
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing: " +
                model.eResource.errors)
        return model
    }
    
    /**
     * Helper function to parse a Lingua Franca program and expect errors.
     * @return A model representing the parsed string.
     */
    def parseWithError(String s) {
        val model = s.parse
        Assertions.assertNotNull(model)
        Assertions.assertFalse(model.eResource.errors.isEmpty)
        return model
    } 

    /**
     * Ensure that duplicate identifiers for actions reported.
     */
    @Test
    def void duplicateVariable() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                logical action bar;
                physical action bar;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.action, null,
        "Duplicate Variable 'bar' in Reactor 'Foo'")
    }
    
    /**
     * Check that reactors in C++ cannot be named preamble 
     */
    @Test
    def void disallowReactorCalledPreamble() {
        val model_no_errors = '''
            target Cpp;
            main reactor Foo {
            }
        '''.parse
        
        val model_error_1 = '''
            target Cpp;
            main reactor Preamble {
            }
        '''.parse
        
        val model_error_2 = '''
            target Cpp;
            reactor Preamble {
            }
            main reactor Main {
            }
        '''.parse
        
        Assertions.assertNotNull(model_no_errors)
        Assertions.assertNotNull(model_error_1)
        Assertions.assertNotNull(model_error_2)
        Assertions.assertTrue(model_no_errors.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_no_errors.eResource.errors)
            Assertions.assertTrue(model_error_1.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_error_1.eResource.errors)
            Assertions.assertTrue(model_error_2.eResource.errors.isEmpty, 
            "Encountered unexpected error while parsing: " + model_error_2.eResource.errors)

        model_no_errors.assertNoIssues
        model_error_1.assertError(LinguaFrancaPackage::eINSTANCE.reactor, null,
            "Reactor cannot be named 'Preamble'")
        model_error_2.assertError(LinguaFrancaPackage::eINSTANCE.reactor, null,
            "Reactor cannot be named 'Preamble'")
    }
 
    /**
     * Ensure that "__" is not allowed at the start of an input name.
     */
    @Test
    def void disallowUnderscoreInputs() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                input __bar;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.input, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of an output name.
     */
    @Test
    def void disallowUnderscoreOutputs() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                output __bar;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.output, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of an action name.
     */
    @Test
    def void disallowUnderscoreActions() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                logical action __bar;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.action, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a timer name.
     */
    @Test
    def void disallowUnderscoreTimers() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                timer __bar(0);
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.timer, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a parameter name.
     */
    @Test
    def void disallowUnderscoreParameters() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo(__bar) {
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of an state name.
     */
    @Test
    def void disallowUnderscoreStates() {
        parseWithoutError('''
            target TypeScript;
            main reactor Foo {
                state __bar;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.stateVar, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __bar")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor definition name.
     */
    @Test
    def void disallowUnderscoreReactorDef() {
        parseWithoutError('''
            target TypeScript;
            main reactor __Foo {
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.reactor, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __Foo")
    }
    
    /**
     * Ensure that "__" is not allowed at the start of a reactor instantiation name.
     */
    @Test
    def void disallowUnderscoreReactorInstantiation() {
        parseWithoutError('''
            target TypeScript;
            reactor Foo {
            }
            main reactor Bar {
                __x = new Foo();
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.instantiation, null,
            "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": __x")
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    def void connectionToEffectPort() {
        parseWithoutError('''
            target C;
            reactor Foo {
                output out:int;
            }
            main reactor Bar {
                output out:int;
                x = new Foo();
                x.out -> out;
                reaction(startup) -> out {=                    
                =}
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'out' is already effect of a reaction.")
    }
    
    /**
     * Disallow connection to port that is effect of reaction.
     */
    @Test
    def void connectionToEffectPort2() {
        parseWithoutError('''
            target C;
            reactor Foo {
                input inp:int;
                output out:int;
            }
            main reactor Bar {
                output out:int;
                x = new Foo();
                y = new Foo();
                
                y.out -> x.inp;
                reaction(startup) -> x.inp {=                    
                =}
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'inp' is already effect of a reaction.")
    }
    
    /**
	 * Allow connection to the port of a contained reactor if another port with same name is effect of a reaction.
	 */
	@Test
	def void connectionToEffectPort3() {
		parseWithoutError('''
			target C;
			
			reactor Foo {
			    input in:int;
			}
			main reactor Bar {
			    input in:int;
			    x1 = new Foo();
			    x2 = new Foo();
			    in -> x1.in;
			    reaction(startup) -> x2.in {=                    
			    =}
			}
		''').assertNoErrors()
	}

	/**
	 * Disallow connection to the port of a contained reactor if the same port is effect of a reaction.
	 */
	@Test
	def void connectionToEffectPort4() {
		parseWithoutError('''
			target C;
			
			reactor Foo {
			    input in:int;
			}
			main reactor Bar {
			    input in:int;
			    x1 = new Foo();
			    in -> x1.in;
			    reaction(startup) -> x1.in {=                    
			    =}
			}
		''').assertError(LinguaFrancaPackage::eINSTANCE.connection, null,
			"Cannot connect: Port named 'in' is already effect of a reaction.")
	}
	
    /**
     * Disallow connection of multiple ports to the same input port.
     */
    @Test
    def void multipleConnectionsToInputTest() {
        parseWithoutError('''
            target C;
            
            reactor Source {
                output out:int;
            }
            reactor Sink {
            	input in:int;
            }

            main reactor Bar {
            	input in:int;
            	   src = new Source();
            	   sink = new Sink();

                in -> sink.in;
                src.out -> sink.in;
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.connection, null,
            "Cannot connect: Port named 'in' may only be connected to a single upstream port.")
    }
    
    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    def void detectInstantiationCycle() {
        parseWithoutError('''
            target C;
            
            reactor Contained {
                x = new Contained();
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.instantiation,
            null, 'Instantiation is part of a cycle: Contained')
    }
    
    
    /**
     * Detect cycles in the instantiation graph.
     */
    @Test
    def void detectInstantiationCycle2() {
        val model = parseWithoutError('''
            target C;
            reactor Intermediate {
                x = new Contained();
            }
            
            reactor Contained {
                x = new Intermediate();
            }
        ''')
        model.assertError(LinguaFrancaPackage::eINSTANCE.instantiation,
            null, 'Instantiation is part of a cycle: Contained')
        model.assertError(LinguaFrancaPackage::eINSTANCE.instantiation,
            null, 'Instantiation is part of a cycle: Intermediate')
    }
    
    /**
     * Report non-zero time value without units.
     */
    @Test
    def void nonZeroTimeValueWithoutUnits() {
        parseWithoutError('''
            target C;
              main reactor HelloWorld {
                  timer t(42, 1 sec);
                  reaction(t) {=
                      printf("Hello World.\n");
                  =}
             }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.value,
            null, "Missing time units. Should be one of " +
                                TimeUnit.VALUES.filter[it != TimeUnit.NONE])
    }    
    
    /**
     * Report reference to non-time parameter in time argument.
     */
    @Test
    def void parameterTypeMismatch() {
        parseWithoutError('''
            target C;
              main reactor HelloWorld(p:int(0)) {
                  timer t(p, 1 sec);
                  reaction(t) {=
                      printf("Hello World.\n");
                  =}
             }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.value,
            null, 'Parameter is not of time type')
        
    }
    
    /**
     * Report inappropriate literal in time argument.
     */
    @Test
    def void targetCodeInTimeArgument() {
        parseWithoutError('''
            target C;
            main reactor HelloWorld {
                timer t({=foo()=}, 1 sec);
                reaction(t) {=
                    printf("Hello World.\n");
                =}
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.value,
            null, 'Invalid time literal')
    }  
    

    /**
     * Report overflowing deadline.
     */
    @Test
    def void overflowingDeadlineC() {
        parseWithoutError('''
            target C;
            main reactor HelloWorld {
            timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (40 hours) {=
                =}
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.deadline, null,
            "Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE +
                " nanoseconds.")
    }  

    
    /**
     * Report overflowing parameter.
     */
    @Test
    def void overflowingParameterC() {
        parseWithoutError('''
            target C;
            main reactor HelloWorld(d:time(40 hours)) {
            timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (d) {=
                =}
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Time value used to specify a deadline exceeds the maximum of " +
                TimeValue.MAX_LONG_DEADLINE + " nanoseconds.")
    }  
    
    
    /**
     * Report overflowing assignment.
     */
    @Test
    def void overflowingAssignmentC() {
        parseWithoutError('''
            target C;
            reactor Print(d:time(39 hours)) {
                timer t;
                reaction(t) {=
                    printf("Hello World.\n");
                =} deadline (d) {=
                =}
            }
            main reactor HelloWorld {
                p = new Print(d=40 hours);
            }
        ''').assertError(LinguaFrancaPackage::eINSTANCE.assignment, null,
            "Time value used to specify a deadline exceeds the maximum of " +
                        TimeValue.MAX_LONG_DEADLINE + " nanoseconds.")
    }  

    /**
     * Report missing trigger.
     */
    @Test
    def void missingTrigger() {
        parseWithoutError('''
		target C;
		reactor X {
		   	reaction() {=
		   		//
		   	=}
		}
        ''').assertWarning(LinguaFrancaPackage::eINSTANCE.reaction, null,
            "Reaction has no trigger.")
    }
        
    /**
     * Test warnings and errors for the target dependent preamble visibility qualifiers 
     */
    @Test
    def void testPreambleVisibility() {
        for (target : Targets.values) {
            for (visibility : Visibility.values) {
                val model_reactor_scope = parseWithoutError('''
                    target «target.name»;
                    reactor Foo {
                        «IF visibility != Visibility.NONE»«visibility» «ENDIF»preamble {==}
                    }
                ''')
                
                val model_file_scope = parseWithoutError('''
                    target «target.name»;
                    «IF visibility != Visibility.NONE»«visibility» «ENDIF»preamble {==}
                    reactor Foo {
                    }
                ''')
                
                val model_no_preamble = parseWithoutError('''
                    target «target.name»;
                    reactor Foo {
                    }
                ''')
                
                model_no_preamble.assertNoIssues
                
                if (target == Targets.CPP) {
                    if (visibility == Visibility.NONE) {
                        model_file_scope.assertError(LinguaFrancaPackage::eINSTANCE.preamble, null,
                            "Preambles for the C++ target need a visibility qualifier (private or public)!")
                        model_reactor_scope.assertError(LinguaFrancaPackage::eINSTANCE.preamble, null,
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
                        model_file_scope.assertWarning(LinguaFrancaPackage::eINSTANCE.preamble, null,
                            '''The «visibility» qualifier has no meaning for the «target.name» target. It should be removed.''')
                        model_reactor_scope.assertWarning(LinguaFrancaPackage::eINSTANCE.preamble, null,
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
    def void stateAndParameterDeclarationsInC() {
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

		model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Type declaration missing.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Missing time units. Should be one of " +
            	TimeUnit.VALUES.filter[it != TimeUnit.NONE])
        model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Invalid time literal.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Time parameter cannot be initialized using a list.")    
        model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Parameter cannot be initialized using parameter.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.stateVar, null,
            "Referenced parameter does not denote a time.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.stateVar, null,
            "Invalid time literal.")
        model.assertError(LinguaFrancaPackage::eINSTANCE.parameter, null,
            "Uninitialized parameter.")
       	model.assertError(LinguaFrancaPackage::eINSTANCE.value, null,
            "Missing time units. Should be one of " +
            	TimeUnit.VALUES.filter[it != TimeUnit.NONE])
    }  
    
    
    /**
     * Recognize valid IPV4 addresses, report invalid ones.
     */
    @Test
    def void recognizeIPV4() {
        
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
            ''').assertWarning(LinguaFrancaPackage::eINSTANCE.host, null,
                "Invalid IP address.")
        ]
    }
    
    /**
     * Recognize valid IPV6 addresses, report invalid ones.
     */
    @Test
    def void recognizeIPV6() {
        
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
                federated reactor X at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''').assertNoIssues()
        ]
        
        // IP addresses that don't parse.
        parseError.forEach [ addr |
            parseWithError('''
                target C;
                reactor Y {}
                federated reactor X at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''')
        ]
        
        // IP addresses that parse but are invalid.
        validationError.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor X at [foo@«addr»]:4242 {
                    y = new Y() at [«addr»]:2424; 
                }
            ''').assertWarning(LinguaFrancaPackage::eINSTANCE.host, null,
                "Invalid IP address.")
        ]
    }
    
    /**
     * Recognize valid host names and fully qualified names, report invalid ones.
     */
    @Test
    def void recognizeHostNames() {
        
        val correct = #["localhost"] // FIXME: add more
    
        val validationError = #["x.y.z"] // FIXME: add more
        
        val parseError = #["..xyz"] // FIXME: add more
                
        // Correct names.
        correct.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''').assertNoIssues()
        ]
        
        // Names that don't parse.
        parseError.forEach [ addr |
            parseWithError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''')
        ]
        
        // Names that parse but are invalid.
        validationError.forEach [ addr |
            parseWithoutError('''
                target C;
                reactor Y {}
                federated reactor X at foo@«addr»:4242 {
                    y = new Y() at «addr»:2424; 
                }
            ''').assertWarning(LinguaFrancaPackage::eINSTANCE.host, null,
                "Invalid host name or fully qualified domain name.")
        ]
    }
 }
