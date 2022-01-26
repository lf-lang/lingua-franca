/* Generator for the Python target. */

/*************
 * Copyright (c) 2019, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.generator.python

import java.io.File
import java.nio.file.Path
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.List
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.JavaAstUtils
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetConfig.Mode
import org.lflang.TargetProperty.CoordinationType
import org.lflang.federated.FedFileConfig
import org.lflang.federated.FederateInstance
import org.lflang.federated.PythonGeneratorExtension
import org.lflang.federated.launcher.FedPyLauncher
import org.lflang.federated.serialization.FedNativePythonSerialization
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorResult
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.JavaGeneratorUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.ParameterInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance
import org.lflang.generator.SubContext
import org.lflang.generator.c.CGenerator
import org.lflang.generator.c.CUtil
import org.lflang.lf.Action
import org.lflang.lf.Assignment
import org.lflang.lf.Delay
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Model
import org.lflang.lf.Output
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.ReactorDecl
import org.lflang.lf.StateVar
import org.lflang.lf.TriggerRef
import org.lflang.lf.Value
import org.lflang.lf.VarRef

import static extension org.lflang.ASTUtils.*
import static extension org.lflang.JavaAstUtils.*

/** 
 * Generator for Python target. This class generates Python code defining each reactor
 * class given in the input .lf file and imported .lf files.
 * 
 * Each class will contain all the reaction functions defined by the user in order, with the necessary ports/actions given as parameters.
 * Moreover, each class will contain all state variables in native Python format.
 * 
 * A backend is also generated using the CGenerator that interacts with the C code library (see CGenerator.xtend).
 * The backend is responsible for passing arguments to the Python reactor functions.
 * 
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
class PythonGenerator extends CGenerator {

    // Used to add statements that come before reactor classes and user code
    var pythonPreamble = new StringBuilder()

    // Used to add module requirements to setup.py (delimited with ,)
    var pythonRequiredModules = new StringBuilder()

    var PythonTypes types;

    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        this(fileConfig, errorReporter, new PythonTypes(errorReporter))
    }

    private new(FileConfig fileConfig, ErrorReporter errorReporter, PythonTypes types) {
        super(fileConfig, errorReporter, false, types)
        // set defaults
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags = newArrayList // -Wall -Wconversion"
        targetConfig.linkerFlags = ""
        this.types = types
    }

    /** 
     * Generic struct for ports with primitive types and
     * statically allocated arrays in Lingua Franca.
     * This template is defined as
     *   typedef struct {
     *       PyObject* value;
     *       bool is_present;
     *       int num_destinations;
     *       FEDERATED_CAPSULE_EXTENSION
     *   } generic_port_instance_struct;
     * 
     * @see reactor-c-py/lib/pythontarget.h
     */
    val generic_port_type = "generic_port_instance_struct"

    /** 
     * Generic struct for ports with dynamically allocated
     * array types (a.k.a. token types) in Lingua Franca.
     * This template is defined as
     *   typedef struct {
     *       PyObject_HEAD
     *       PyObject* value;
     *       bool is_present;
     *       int num_destinations;
     *       lf_token_t* token;
     *       int length;
     *       FEDERATED_CAPSULE_EXTENSION
     *   } generic_port_instance_with_token_struct;
     * 
     * @see reactor-c-py/lib/pythontarget.h
     */
    val generic_port_type_with_token = "generic_port_instance_with_token_struct"

    /**
     * Generic struct for actions.
     * This template is defined as
     *   typedef struct {
     *      trigger_t* trigger;
     *      PyObject* value;
     *      bool is_present;
     *      bool has_value;
     *      lf_token_t* token;
     *      FEDERATED_CAPSULE_EXTENSION
     *   } generic_action_instance_struct;
     * 
     * @see reactor-c-py/lib/pythontarget.h
     */
    val generic_action_type = "generic_action_instance_struct"

    /** Returns the Target enum for this generator */
    override getTarget() {
        return Target.Python
    }

    val protoNames = new HashSet<String>()

    // //////////////////////////////////////////
    // // Public methods
    override printInfo() {
        println("Generating code for: " + fileConfig.resource.getURI.toString)
        println('******** Mode: ' + fileConfig.context.mode)
        println('******** Generated sources: ' + fileConfig.getSrcGenPath)
    }

    /**
     * Print information about necessary steps to install the supporting
     * Python C extension for the generated program.
     * 
     * @note Only needed if no-compile is set to true
     */
    def printSetupInfo() {
        println('''
            
            #####################################
            To compile and install the generated code, do:
                
                cd «fileConfig.srcGenPath»«File.separator»
                python3 -m pip install --force-reinstall .
        ''');
    }

    /**
     * Print information on how to execute the generated program.
     */
    def printRunInfo() {
        println('''
            
            #####################################
            To run the generated program, use:
                
                python3 «fileConfig.srcGenPath»«File.separator»«topLevelName».py
            
            #####################################
        ''');
    }

    /**
     * Print information on how to execute the generated federation.
     */
    def printFedRunInfo() {
        println('''
            
            #####################################
            To run the generated program, run:
                
                bash «fileConfig.binPath»/«fileConfig.name»
            
            #####################################
        ''');
    }

    override getTargetTypes() {
        return types;
    }

    // //////////////////////////////////////////
    // // Protected methods
    /**
     * Override to convert some C types to their
     * Python equivalent.
     * Examples:
     * true/false -> True/False
     * @param v A value
     * @return A value string in the target language
     */
    private def getPythonTargetValue(Value v) {
        var String returnValue = "";
        switch (v.toText) {
            case "false": returnValue = "False"
            case "true": returnValue = "True"
            default: returnValue = v.targetValue
        }

        // Parameters in Python are always prepended with a 'self.'
        // predicate. Therefore, we need to append the returned value
        // if it is a parameter.
        if (v.parameter !== null) {
            returnValue = "self." + returnValue;
        }

        return returnValue;
    }

    /**
     * Create a list of state initializers in target code.
     * 
     * @param state The state variable to create initializers for
     * @return A list of initializers in target code
     */
    protected def List<String> getPythonInitializerList(StateVar state) {
        if (!state.isInitialized) {
            return null
        }

        var list = new ArrayList<String>();

        for (i : state?.init) {
            if (i.parameter !== null) {
                list.add(i.parameter.name)
            } else if (state.isOfTimeType) {
                list.add(i.targetTime)
            } else {
                list.add(i.pythonTargetValue)
            }
        }
        return list
    }

    /**
     * Create a Python tuple for parameter initialization in target code.
     * 
     * @param p The parameter instance to create initializers for
     * @return Initialization code
     */
    protected def String getPythonInitializer(StateVar state) throws Exception {
        if (state.init.size > 1) {
            // state variables are initialized as mutable lists
            return state.init.join('[', ', ', ']', [it.pythonTargetValue])
        } else if (state.isInitialized) {
            return state.init.get(0).getPythonTargetValue
        } else {
            return "None"
        }

    }

    /**
     * Return a Python expression that can be used to initialize the specified
     * parameter instance. If the parameter initializer refers to other
     * parameters, then those parameter references are replaced with
     * accesses to the Python reactor instance class of the parents of 
     * those parameters.
     * 
     * @param p The parameter instance to create initializer for
     * @return Initialization code
     */
    protected def String getPythonInitializer(ParameterInstance p) {
        // Handle overrides in the intantiation.
        // In case there is more than one assignment to this parameter, we need to
        // find the last one.
        var lastAssignment = null as Assignment;
        for (assignment : p.parent.definition.parameters) {
            if (assignment.lhs == p.definition) {
                lastAssignment = assignment;
            }
        }

        var list = new LinkedList<String>();
        if (lastAssignment !== null) {
            // The parameter has an assignment.
            // Right hand side can be a list. Collect the entries.
            for (value : lastAssignment.rhs) {
                if (value.parameter !== null) {
                    // The parameter is being assigned a parameter value.
                    // Assume that parameter belongs to the parent's parent.
                    // This should have been checked by the validator.
                    list.add(PyUtil.reactorRef(p.parent.parent) + "." + value.parameter.name);
                } else {
                    list.add(value.targetTime)
                }
            }
        } else {
            for (i : p.parent.initialParameterValue(p.definition)) {
                list.add(i.getPythonTargetValue)
            }
        }

        if (list.size == 1) {
            return list.get(0)
        } else {
            return list.join('(', ', ', ')', [it])
        }

    }

    /**
     * Create a Python list for parameter initialization in target code.
     * 
     * @param p The parameter to create initializers for
     * @return Initialization code
     */
    protected def String getPythonInitializer(Parameter p) {
        if (p.init.size > 1) {
            // parameters are initialized as immutable tuples
            return p.init.join('(', ', ', ')', [it.pythonTargetValue])
        } else {
            return p.init.get(0).pythonTargetValue
        }
    }

    /**
     * Generate parameters and their respective initialization code for a reaction function
     * The initialization code is put at the beginning of the reaction before user code
     * @param parameters The parameters used for function definition
     * @param inits The initialization code for those paramters
     * @param decl Reactor declaration
     * @param reaction The reaction to be used to generate parameters for
     */
    def generatePythonReactionParametersAndInitializations(StringBuilder parameters, StringBuilder inits,
        ReactorDecl decl, Reaction reaction) {
        val reactor = decl.toDefinition
        var generatedParams = new LinkedHashSet<String>()

        // Handle triggers
        for (TriggerRef trigger : reaction.triggers ?: emptyList) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    if (trigger.variable instanceof Input) {
                        if ((trigger.variable as Input).isMutable) {
                            generatedParams.add('''mutable_«trigger.variable.name»''')

                            // Create a deep copy                            
                            if (JavaAstUtils.isMultiport(trigger.variable as Input)) {
                                inits.
                                    append('''«trigger.variable.name» = [Make() for i in range(len(mutable_«trigger.variable.name»))]
                                    ''')
                                inits.append('''for i in range(len(mutable_«trigger.variable.name»)):
                                ''')
                                inits.
                                    append('''    «trigger.variable.name»[i].value = copy.deepcopy(mutable_«trigger.variable.name»[i].value)
                                    ''')
                            } else {
                                inits.append('''«trigger.variable.name» = Make()
                                ''')
                                inits.
                                    append('''«trigger.variable.name».value = copy.deepcopy(mutable_«trigger.variable.name».value)
                                    ''')
                            }
                        } else {
                            generatedParams.add(trigger.variable.name)
                        }
                    } else {
                        // Handle contained reactors' ports
                        generatedParams.add('''«trigger.container.name»_«trigger.variable.name»''')
                        generatePythonPortVariableInReaction(trigger, inits)
                    }

                } else if (trigger.variable instanceof Action) {
                    generatedParams.add(trigger.variable.name)
                }
            }
        }

        // Handle non-triggering inputs
        if (reaction.triggers === null || reaction.triggers.size === 0) {
            for (input : reactor.inputs ?: emptyList) {
                generatedParams.add(input.name)
                if (input.isMutable) {
                    // Create a deep copy
                    inits.append('''«input.name» = copy.deepcopy(«input.name»)
                    ''')
                }
            }
        }
        for (src : reaction.sources ?: emptyList) {
            if (src.variable instanceof Output) {
                // Output of a contained reactor
                generatedParams.add('''«src.container.name»_«src.variable.name»''')
                generatePythonPortVariableInReaction(src, inits)
            } else {
                generatedParams.add(src.variable.name)
                if (src.variable instanceof Input) {
                    if ((src.variable as Input).isMutable) {
                        // Create a deep copy
                        inits.append('''«src.variable.name» = copy.deepcopy(«src.variable.name»)
                        ''')
                    }
                }
            }
        }

        // Handle effects
        for (effect : reaction.effects ?: emptyList) {
            if (effect.variable instanceof Input) {
                generatedParams.add('''«effect.container.name»_«effect.variable.name»''')
                generatePythonPortVariableInReaction(effect, inits)
            } else {
                generatedParams.add(effect.variable.name)
                if (effect.variable instanceof Port) {
                    if (JavaAstUtils.isMultiport(effect.variable as Port)) {
                        // Handle multiports           
                    }
                }
            }
        }

        // Fill out the StrinBuilder parameters
        for (s : generatedParams) {
            parameters.append(''', «s»''')
        }

    }
    
    /**
     * Generate into the specified string builder (<code>inits<code>) the code to
     * initialize local variable for <code>port<code> so that it can be used in the body of
     * the Python function.
     * @param port The port to generate code for.
     * @param inits The generated code will be put in <code>inits<code>.
     */
    protected def StringBuilder generatePythonPortVariableInReaction(VarRef port, StringBuilder inits) {
        if (port.container.widthSpec !== null) {
            // It's a bank
            inits.append('''
                «port.container.name» = [None] * len(«port.container.name»_«port.variable.name»)
                for i in range(len(«port.container.name»_«port.variable.name»)):
                    «port.container.name»[i] = Make()
                    «port.container.name»[i].«port.variable.name» = «port.container.name»_«port.variable.name»[i]
            ''')
            
        } else {
            inits.append('''«port.container.name» = Make()
            ''')
            inits.append('''«port.container.name».«port.variable.name» = «port.container.name»_«port.variable.name»
            ''')
        }
    }

    /**
     * Handle initialization for state variable
     * @param state a state variable
     */
    def String getTargetInitializer(StateVar state) {
        if (!state.isInitialized) {
            return '''None'''
        }

        '''«FOR init : state.pythonInitializerList SEPARATOR ", "»«init»«ENDFOR»'''
    }

    /**
     * Wrapper function for the more elaborate generatePythonReactorClass that keeps track
     * of visited reactors to avoid duplicate generation
     * @param instance The reactor instance to be generated
     * @param pythonClasses The class definition is appended to this string builder
     * @param federate The federate instance for the reactor instance
     * @param instantiatedClasses A list of visited instances to avoid generating duplicates
     */
    def generatePythonReactorClass(ReactorInstance instance, StringBuilder pythonClasses, FederateInstance federate) {
        var instantiatedClasses = new ArrayList<String>()
        generatePythonReactorClass(instance, pythonClasses, federate, instantiatedClasses)
    }

    /**
     * Generate a Python class corresponding to decl
     * @param instance The reactor instance to be generated
     * @param pythonClasses The class definition is appended to this string builder
     * @param federate The federate instance for the reactor instance
     * @param instantiatedClasses A list of visited instances to avoid generating duplicates
     */
    def void generatePythonReactorClass(ReactorInstance instance, StringBuilder pythonClasses,
        FederateInstance federate, ArrayList<String> instantiatedClasses) {
        if (instance !== this.main && !federate.contains(instance)) {
            return
        }

        // Invalid use of the function
        if (instantiatedClasses === null) {
            return
        }

        val decl = instance.definition.reactorClass
        val className = instance.definition.reactorClass.name

        // Do not generate code for delay reactors in Python
        if (className.contains(GEN_DELAY_CLASS_NAME)) {
            return
        }

        if (federate.contains(instance) && !instantiatedClasses.contains(className)) {

            pythonClasses.append('''
                                
                # Python class for reactor «className»
                class _«className»:
            ''');

            // Generate preamble code
            pythonClasses.append('''
                
                    «generatePythonPreamblesForReactor(decl.toDefinition)»
            ''')

            val reactor = decl.toDefinition

            // Handle runtime initializations
            pythonClasses.append('''    
                «'    '»def __init__(self, **kwargs):
            ''')

            pythonClasses.append(generateParametersAndStateVariables(decl))
            
            var reactionToGenerate = reactor.allReactions
            
            if (reactor.isFederated) {
                // Filter out reactions that are automatically generated in C in the top level federated reactor
                reactionToGenerate.removeIf([
                    if (!federate.contains(it)) return true;
                    if (federate.networkReactions.contains(it)) return true
                    return false

                ])
            }
            
            var reactionIndex = 0
            for (reaction : reactionToGenerate) {
                val reactionParameters = new StringBuilder() // Will contain parameters for the function (e.g., Foo(x,y,z,...)
                val inits = new StringBuilder() // Will contain initialization code for some parameters
                generatePythonReactionParametersAndInitializations(reactionParameters, inits, reactor, reaction)
                pythonClasses.append('''    def «pythonReactionFunctionName(reactionIndex)»(self«reactionParameters»):
                ''')
                pythonClasses.append('''        «inits»
                ''')
                pythonClasses.append('''        «reaction.code.toText»
                ''')
                pythonClasses.append('''        return 0
                
                ''')

                // Now generate code for the deadline violation function, if there is one.
                if (reaction.deadline !== null) {
                    pythonClasses.
                        append('''    «generateDeadlineFunctionForReaction(reaction, reactionIndex, reactionParameters.toString)»
                        ''')
                }

                reactionIndex = reactionIndex + 1;
            }
            instantiatedClasses.add(className)
        }

        for (child : instance.children) {
            generatePythonReactorClass(child, pythonClasses, federate, instantiatedClasses)
        }
    }

    /**
     * Generate code that instantiates and initializes parameters and state variables for a reactor 'decl'.
     * 
     * @param decl The reactor declaration
     * @return The generated code as a StringBuilder
     */
    protected def StringBuilder generateParametersAndStateVariables(ReactorDecl decl) {
        val reactor = decl.toDefinition
        var StringBuilder temporary_code = new StringBuilder()

        temporary_code.append('''        #Define parameters and their default values
        ''')

        for (param : decl.toDefinition.allParameters) {
            if (!types.getTargetType(param).equals("PyObject*")) {
                // If type is given, use it
                temporary_code.
                    append('''        self._«param.name»:«types.getPythonType(param.inferredType)» = «param.pythonInitializer»
                    ''')
            } else {
                // If type is not given, just pass along the initialization
                temporary_code.append('''        self._«param.name» = «param.pythonInitializer»
                ''')

            }
        }

        // Handle parameters that are set in instantiation
        temporary_code.append('''        # Handle parameters that are set in instantiation
        ''')
        temporary_code.append('''        self.__dict__.update(kwargs)
        
        ''')

        temporary_code.append('''        # Define state variables
        ''')
        // Next, handle state variables
        for (stateVar : reactor.allStateVars) {
            if (stateVar.isInitialized) {
                // If initialized, pass along the initialization directly if it is present
                temporary_code.append('''        self.«stateVar.name» = «stateVar.pythonInitializer»
                ''')
            } else {
                // If neither the type nor the initialization is given, use None
                temporary_code.append('''        self.«stateVar.name» = None
                ''')
            }
        }
        
        
        temporary_code.append('''
        
        ''')

        // Next, create getters for parameters
        for (param : decl.toDefinition.allParameters) {
            if (!param.name.equals("bank_index")) {
                temporary_code.append('''    @property
                ''')
                temporary_code.append('''    def «param.name»(self):
                ''')
                temporary_code.append('''        return self._«param.name» # pylint: disable=no-member
                
                ''')
            }
        }

        // Create a special property for bank_index
        temporary_code.append('''    @property
        ''')
        temporary_code.append('''    def bank_index(self):
        ''')
        temporary_code.append('''        return self._bank_index # pylint: disable=no-member
        
        ''')

        return temporary_code;
    }

    /**
     * Generate the function that is executed whenever the deadline of the reaction
     * with the given reaction index is missed
     * @param reaction The reaction to generate deadline miss code for
     * @param reactionIndex The agreed-upon index of the reaction in the reactor (should match the C generated code)
     * @param reactionParameters The parameters to the deadline violation function, which are the same as the reaction function
     */
    def generateDeadlineFunctionForReaction(Reaction reaction, int reactionIndex, String reactionParameters) '''
        «val deadlineFunctionName = 'deadline_function_' + reactionIndex»
        
        def «deadlineFunctionName»(self «reactionParameters»):
            «reaction.deadline.code.toText»
            return 0
        
    '''

    /**
     * Generates preambles defined by user for a given reactor.
     * The preamble code is put inside the reactor class.
     */
    def generatePythonPreamblesForReactor(Reactor reactor) '''
        «FOR p : reactor.preambles ?: emptyList»
            # From the preamble, verbatim:
            «p.code.toText»
            # End of preamble.
        «ENDFOR»
    '''

    /**
     * Instantiate classes in Python.
     * Instances are always instantiated as a list of className = [_className, _className, ...] depending on the size of the bank.
     * If there is no bank or the size is 1, the instance would be generated as className = [_className]
     * @param instance The reactor instance to be instantiated
     * @param pythonClassesInstantiation The class instantiations are appended to this string builder
     * @param federate The federate instance for the reactor instance
     */
    def void generatePythonClassInstantiation(ReactorInstance instance, StringBuilder pythonClassesInstantiation,
        FederateInstance federate) {
        // If this is not the main reactor and is not in the federate, nothing to do.
        if (instance !== this.main && !federate.contains(instance)) {
            return
        }

        val className = instance.definition.reactorClass.name

        // Do not instantiate delay reactors in Python
        if (className.contains(GEN_DELAY_CLASS_NAME)) {
            return
        }

        if (federate.contains(instance) && instance.width > 0) {
            // For each reactor instance, create a list regardless of whether it is a bank or not.
            // Non-bank reactor instances will be a list of size 1.         var reactorClass = instance.definition.reactorClass
            var fullName = instance.fullName
            pr(pythonClassesInstantiation, '''
                
                # Start initializing «fullName» of class «className»
                for «PyUtil.bankIndexName(instance)» in range(«instance.width»):
            ''')
            indent(pythonClassesInstantiation);
            pr(pythonClassesInstantiation, '''
                «PyUtil.reactorRef(instance)» = \
                    _«className»(
                        _bank_index = «PyUtil.bankIndex(instance)»,
                        «FOR param : instance.parameters»
                            «IF !param.name.equals("bank_index")»
                                _«param.name»=«param.pythonInitializer»,
                            «ENDIF»«ENDFOR»
                        )
            ''')
        }

        for (child : instance.children) {
            generatePythonClassInstantiation(child, pythonClassesInstantiation, federate)
        }
        unindent(pythonClassesInstantiation);
    }

    /**
     * Generate code to instantiate a Python list that will hold the Python 
     * class instance of reactor <code>instance<code>. Will recursively do 
     * the same for the children of <code>instance<code> as well.
     * 
     * @param instance The reactor instance for which the Python list will be created.
     * @param pythonClassesInstantiation StringBuilder to hold the generated code. 
     * @param federate Will check if <code>instance<code> (or any of its children) belong to 
     *  <code>federate<code> before generating code for them.
     */
    def void generateListsToHoldClassInstances(
        ReactorInstance instance,
        StringBuilder pythonClassesInstantiation,
        FederateInstance federate
    ) {
        if(federate !== null && !federate.contains(instance)) return;
        pr(pythonClassesInstantiation, '''
            «PyUtil.reactorRefName(instance)» = [None] * «instance.totalWidth»
        ''')
        for (child : instance.children) {
            generateListsToHoldClassInstances(child, pythonClassesInstantiation, federate);
        }
    }

    /**
     * Generate all Python classes if they have a reaction
     * @param federate The federate instance used to generate classes
     */
    def generatePythonReactorClasses(FederateInstance federate) {

        var StringBuilder pythonClasses = new StringBuilder()
        var StringBuilder pythonClassesInstantiation = new StringBuilder()

        // Generate reactor classes in Python
        this.main.generatePythonReactorClass(pythonClasses, federate)

        // Create empty lists to hold reactor instances
        this.main.generateListsToHoldClassInstances(pythonClassesInstantiation, federate)

        // Instantiate generated classes
        this.main.generatePythonClassInstantiation(pythonClassesInstantiation, federate)

        '''«pythonClasses»
        
        ''' + '''# Instantiate classes
        ''' + '''«pythonClassesInstantiation»
        '''
    }

    /**
     * Generate the Python code constructed from reactor classes and user-written classes.
     * @return the code body 
     */
    def generatePythonCode(FederateInstance federate) '''
        # List imported names, but do not use pylint's --extension-pkg-allow-list option
        # so that these names will be assumed present without having to compile and install.
        from LinguaFranca«topLevelName» import (  # pylint: disable=no-name-in-module
            Tag, action_capsule_t, compare_tags, get_current_tag, get_elapsed_logical_time,
            get_elapsed_physical_time, get_logical_time, get_microstep, get_physical_time,
            get_start_time, port_capsule, port_instance_token, request_stop, schedule_copy,
            start
        )
        from LinguaFrancaBase.constants import BILLION, FOREVER, NEVER, instant_t, interval_t
        from LinguaFrancaBase.functions import (
            DAY, DAYS, HOUR, HOURS, MINUTE, MINUTES, MSEC, MSECS, NSEC, NSECS, SEC, SECS, USEC,
            USECS, WEEK, WEEKS
        )
        from LinguaFrancaBase.classes import Make
        import sys
        import copy
        
        «pythonPreamble.toString»
        
        «generatePythonReactorClasses(federate)»
        
        «PythonMainGenerator.generateCode()»
    '''

    /**
     * Generate the setup.py required to compile and install the module.
     * Currently, the package name is based on filename which does not support sharing the setup.py for multiple .lf files.
     * TODO: use an alternative package name (possibly based on folder name)
     * 
     * If the LF program itself is threaded or if tracing is enabled, NUMBER_OF_WORKERS is added as a macro
     * so that platform-specific C files will contain the appropriate functions.
     */
    def generatePythonSetupFile() '''
        from setuptools import setup, Extension
        
        linguafranca«topLevelName»module = Extension("LinguaFranca«topLevelName»",
                                                   sources = ["«topLevelName».c", «FOR src : targetConfig.compileAdditionalSources SEPARATOR ", "» "«src»"«ENDFOR»],
                                                   define_macros=[('MODULE_NAME', 'LinguaFranca«topLevelName»')«IF (targetConfig.threads !== 0 || (targetConfig.tracing !== null))», 
                                                       ('NUMBER_OF_WORKERS', '«targetConfig.threads»')«ENDIF»])
            
        setup(name="LinguaFranca«topLevelName»", version="1.0",
                ext_modules = [linguafranca«topLevelName»module],
                install_requires=['LinguaFrancaBase' «pythonRequiredModules»],)
        '''

    /**
     * Generate the necessary Python files
     * @param fsa The file system access (used to write the result).
     * @param federate The federate instance
     */
    def generatePythonFiles(IFileSystemAccess2 fsa, FederateInstance federate) {
        var file = new File(fileConfig.getSrcGenPath.toFile, topLevelName + ".py")
        if (file.exists) {
            file.delete
        }
        // Create the necessary directories
        if (!file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }
        val codeMaps = #{file.toPath -> CodeMap.fromGeneratedCode(generatePythonCode(federate).toString)}
        JavaGeneratorUtils.writeSourceCodeToFile(codeMaps.get(file.toPath).generatedCode, file.absolutePath)

        val setupPath = fileConfig.getSrcGenPath.resolve("setup.py")
        // Handle Python setup
        System.out.println("Generating setup file to " + setupPath)
        file = setupPath.toFile
        if (file.exists) {
            // Append
            file.delete
        }

        // Create the setup file
        JavaGeneratorUtils.writeSourceCodeToFile(generatePythonSetupFile, setupPath.toString)

        return codeMaps
    }

    /**
     * Execute the command that compiles and installs the current Python module
     */
    def pythonCompileCode(LFGeneratorContext context) {
        // if we found the compile command, we will also find the install command
        val installCmd = commandFactory.createCommand(
            '''python3''', #["-m", "pip", "install", "--force-reinstall", "."], fileConfig.srcGenPath)

        if (installCmd === null) {
            errorReporter.reportError(
                "The Python target requires Python >= 3.6, pip >= 20.0.2, and setuptools >= 45.2.0-1 to compile the generated code. " +
                    "Auto-compiling can be disabled using the \"no-compile: true\" target property.")
            return
        }

        // Set compile time environment variables
        installCmd.setEnvironmentVariable("CC", targetConfig.compiler) // Use gcc as the compiler
        installCmd.setEnvironmentVariable("LDFLAGS", targetConfig.linkerFlags) // The linker complains about including pythontarget.h twice (once in the generated code and once in pythontarget.c)
        // To avoid this, we force the linker to allow multiple definitions. Duplicate names would still be caught by the 
        // compiler.
        if (installCmd.run(context.cancelIndicator) == 0) {
            println("Successfully installed python extension.")
        } else {
            errorReporter.reportError("Failed to install python extension due to the following errors:\n" +
                installCmd.getErrors())
        }
    }

    /** 
     * Generate top-level preambles and #include of pqueue.c and either reactor.c or reactor_threaded.c
     *  depending on whether threads are specified in target directive.
     *  As a side effect, this populates the runCommand and compileCommand
     *  private variables if such commands are specified in the target directive.
     */
    override generatePreamble() {

        val models = new LinkedHashSet<Model>

        for (r : this.reactors ?: emptyList) {
            // The following assumes all reactors have a container.
            // This means that generated reactors **have** to be
            // added to a resource; not doing so will result in a NPE.
            models.add(r.toDefinition.eContainer as Model)
        }
        // Add the main reactor if it is defined
        if (this.mainDef !== null) {
            models.add(this.mainDef.reactorClass.toDefinition.eContainer as Model)
        }
        for (m : models) {
            for (p : m.preambles) {
                pythonPreamble.append('''«p.code.toText»
                ''')
            }
        }

        pr(CGenerator.defineLogLevel(this))

        if (isFederated) {
            // FIXME: Instead of checking
            // #ifdef FEDERATED, we could
            // use #if (NUMBER_OF_FEDERATES > 1)
            // To me, the former is more accurate.
            pr('''
                #define FEDERATED
            ''')
            if (targetConfig.coordination === CoordinationType.CENTRALIZED) {
                // The coordination is centralized.
                pr('''
                    #define FEDERATED_CENTRALIZED
                ''')
            } else if (targetConfig.coordination === CoordinationType.DECENTRALIZED) {
                // The coordination is decentralized
                pr('''
                    #define FEDERATED_DECENTRALIZED
                ''')
            }
        }

        // Handle target parameters.
        // First, if there are federates, then ensure that threading is enabled.
        if (isFederated) {
            for (federate : federates) {
                // The number of threads needs to be at least one larger than the input ports
                // to allow the federate to wait on all input ports while allowing an additional
                // worker thread to process incoming messages.
                if (targetConfig.threads < federate.networkMessageActions.size + 1) {
                    targetConfig.threads = federate.networkMessageActions.size + 1;
                }
            }
        }

        includeTargetLanguageHeaders()

        pr("#include \"core/mixed_radix.h\"");

        pr('#define NUMBER_OF_FEDERATES ' + federates.size);

        // Handle target parameters.
        // First, if there are federates, then ensure that threading is enabled.
        if (targetConfig.threads === 0 && isFederated) {
            targetConfig.threads = 1
        }

        super.includeTargetLanguageSourceFiles()

        super.parseTargetParameters()
    }

    /**
     * Add necessary code to the source and necessary build supports to
     * enable the requested serializations in 'enabledSerializations'
     */
    override enableSupportForSerializationIfApplicable(CancelIndicator cancelIndicator) {
        if (!targetConfig.protoFiles.isNullOrEmpty) {
            // Enable support for proto serialization
            enabledSerializers.add(SupportedSerializers.PROTO)
        }
        for (serialization : enabledSerializers) {
            switch (serialization) {
                case NATIVE: {
                    val pickler = new FedNativePythonSerialization();
                    pr(pickler.generatePreambleForSupport.toString);
                }
                case PROTO: {
                    // Handle .proto files.
                    for (name : targetConfig.protoFiles) {
                        this.processProtoFile(name, cancelIndicator)
                        val dotIndex = name.lastIndexOf('.')
                        var rootFilename = name
                        if (dotIndex > 0) {
                            rootFilename = name.substring(0, dotIndex)
                        }
                        pythonPreamble.append('''
                            import «rootFilename»_pb2 as «rootFilename»
                        ''')
                        protoNames.add(rootFilename)
                    }
                }
                case ROS2: {
                    // FIXME: Not supported yet
                }
            }
        }
    }

    /**
     * Process a given .proto file.
     * 
     * Run, if possible, the proto-c protocol buffer code generator to produce
     * the required .h and .c files.
     * @param filename Name of the file to process.
     */
    override processProtoFile(String filename, CancelIndicator cancelIndicator) {
        val protoc = commandFactory.createCommand("protoc",
            #['''--python_out=«this.fileConfig.getSrcGenPath»''', filename], fileConfig.srcPath)
        // val protoc = createCommand("protoc", #['''--python_out=src-gen/«topLevelName»''', topLevelName], codeGenConfig.outPath)
        if (protoc === null) {
            errorReporter.reportError("Processing .proto files requires libprotoc >= 3.6.1")
            return
        }
        val returnCode = protoc.run(cancelIndicator)
        if (returnCode == 0) {
            pythonRequiredModules.append(''', 'google-api-python-client' ''')
        } else {
            errorReporter.reportError("protoc returns error code " + returnCode)
        }
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @param isPhysical Indicates whether or not the connection is physical
     * @param serializer The serializer used on the connection.
     */
    override generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        int receivingBankIndex,
        int receivingChannelIndex,
        InferredType type,
        boolean isPhysical,
        SupportedSerializers serializer
    ) {
        var result = new StringBuilder();
        result.append('''
            // Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.         
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();
        ''')
        result.append(PythonGeneratorExtension.generateNetworkReceiverBody(
            action,
            sendingPort,
            receivingPort,
            receivingPortID,
            sendingFed,
            receivingFed,
            receivingBankIndex,
            receivingChannelIndex,
            type,
            isPhysical,
            serializer,
            this
        ));
        result.append('''
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
        ''');
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The variable reference to the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @param serializer The serializer used on the connection.
     */
    override generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay,
        SupportedSerializers serializer
    ) {
        var result = new StringBuilder();
        result.append('''
            // Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.         
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();
        ''')
        result.append(PythonGeneratorExtension.generateNetworkSenderBody(
            sendingPort,
            receivingPort,
            receivingPortID,
            sendingFed,
            sendingBankIndex,
            sendingChannelIndex,
            receivingFed,
            type,
            isPhysical,
            delay,
            serializer,
            this
        ));
        result.append('''
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
        ''');
        return result.toString();
    }

    /**
     * Create a launcher script that executes all the federates and the RTI.
     * 
     * @param coreFiles The files from the core directory that must be
     *  copied to the remote machines.
     */
    override createFederatedLauncher(ArrayList<String> coreFiles) {
        val launcher = new FedPyLauncher(
            targetConfig,
            fileConfig,
            errorReporter
        );
        launcher.createLauncher(
            coreFiles,
            federates,
            federationRTIProperties
        );
    }

    /**
     * Generate the aliases for inputs, outputs, and struct type definitions for 
     * actions of the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     */
    override generateAuxiliaryStructs(
        ReactorDecl decl,
        FederateInstance federate
    ) {
        val reactor = decl.toDefinition
        // First, handle inputs.
        for (input : reactor.allInputs) {
            if (federate === null || federate.contains(input as Port)) {
                if (input.inferredType.isTokenType) {
                    pr(input, code, '''
                        typedef «generic_port_type_with_token» «variableStructType(input, decl)»;
                    ''')
                } else {
                    pr(input, code, '''
                        typedef «generic_port_type» «variableStructType(input, decl)»;
                    ''')
                }

            }

        }
        // Next, handle outputs.
        for (output : reactor.allOutputs) {
            if (federate === null || federate.contains(output as Port)) {
                if (output.inferredType.isTokenType) {
                    pr(output, code, '''
                        typedef «generic_port_type_with_token» «variableStructType(output, decl)»;
                    ''')
                } else {
                    pr(output, code, '''
                        typedef «generic_port_type» «variableStructType(output, decl)»;
                    ''')
                }

            }
        }
        // Finally, handle actions.
        for (action : reactor.allActions) {
            if (federate === null || federate.contains(action)) {
                pr(action, code, '''
                    typedef «generic_action_type» «variableStructType(action, decl)»;
                ''')
            }

        }
    }

    /**
     * For the specified action, return a declaration for action struct to
     * contain the value of the action.
     * This will return an empty string for an action with no type.
     * @param action The action.
     * @return A string providing the value field of the action struct.
     */
    override valueDeclaration(Action action) {
        return "PyObject* value;"
    }

    /** Add necessary include files specific to the target language.
     *  Note. The core files always need to be (and will be) copied 
     *  uniformly across all target languages.
     */
    override includeTargetLanguageHeaders() {
        pr('''#define _LF_GARBAGE_COLLECTED''')
        if (targetConfig.tracing !== null) {
            var filename = "";
            if (targetConfig.tracing.traceFileName !== null) {
                filename = targetConfig.tracing.traceFileName;
            }
            pr('#define LINGUA_FRANCA_TRACE ' + filename)
        }

        pr('#include "pythontarget.c"')
        if (targetConfig.tracing !== null) {
            pr('#include "core/trace.c"')
        }
    }

    /**
     * Return true if the host operating system is compatible and
     * otherwise report an error and return false.
     */
    override isOSCompatible() {
        if (JavaGeneratorUtils.isHostWindows) {
            if (isFederated) {
                errorReporter.reportError(
                    "Federated LF programs with a Python target are currently not supported on Windows. Exiting code generation."
                )
                // Return to avoid compiler errors
                return false
            }
        }
        return true;
    }

    /** Generate C code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, LFGeneratorContext context) {

        // If there are federates, assign the number of threads in the CGenerator to 1        
        if (isFederated) {
            targetConfig.threads = 1;
        }

        // Prevent the CGenerator from compiling the C code.
        // The PythonGenerator will compiler it.
        val compileStatus = targetConfig.noCompile;
        targetConfig.noCompile = true;
        targetConfig.useCmake = false; // Force disable the CMake because 
        // it interferes with the Python target functionality
        val cGeneratedPercentProgress = (IntegratedBuilder.VALIDATED_PERCENT_PROGRESS + 100) / 2
        super.doGenerate(resource, fsa, new SubContext(
            context,
            IntegratedBuilder.VALIDATED_PERCENT_PROGRESS,
            cGeneratedPercentProgress
        ))
        val compilingFederatesContext = new SubContext(context, cGeneratedPercentProgress, 100)

        targetConfig.noCompile = compileStatus

        if (errorsOccurred) {
            context.unsuccessfulFinish()
            return;
        }

        var baseFileName = topLevelName
        // Keep a separate file config for each federate
        val oldFileConfig = fileConfig;
        var federateCount = 0;
        val codeMaps = new HashMap<Path, CodeMap>
        for (federate : federates) {
            federateCount++
            if (isFederated) {
                topLevelName = baseFileName + '_' + federate.name
                fileConfig = new FedFileConfig(fileConfig, federate.name);
            }
            // Don't generate code if there is no main reactor
            if (this.main !== null) {
                val codeMapsForFederate = generatePythonFiles(fsa, federate)
                codeMaps.putAll(codeMapsForFederate)
                if (!targetConfig.noCompile) {
                    compilingFederatesContext.reportProgress(
                        String.format("Validating %d/%d sets of generated files...", federateCount, federates.size()),
                        100 * federateCount / federates.size()
                    )
                    // If there are no federates, compile and install the generated code
                    new PythonValidator(fileConfig, errorReporter, codeMaps, protoNames).doValidate(context)
                    if (!errorsOccurred() && context.mode != Mode.LSP_MEDIUM) {
                        compilingFederatesContext.reportProgress(
                            String.format("Validation complete. Compiling and installing %d/%d Python modules...",
                                federateCount, federates.size()),
                            100 * federateCount / federates.size()
                        )
                        pythonCompileCode(context) // Why is this invoked here if the current federate is not a parameter?
                    }
                } else {
                    printSetupInfo();
                }

                if (!isFederated) {
                    printRunInfo();
                }
            }
            fileConfig = oldFileConfig;
        }
        if (isFederated) {
            printFedRunInfo();
        }
        // Restore filename
        topLevelName = baseFileName
        if (errorReporter.getErrorsOccurred()) {
            context.unsuccessfulFinish()
        } else if (!isFederated) {
            context.finish(GeneratorResult.Status.COMPILED, '''«topLevelName».py''', fileConfig.srcGenPath, fileConfig,
                codeMaps, "python3")
        } else {
            context.finish(GeneratorResult.Status.COMPILED, fileConfig.name, fileConfig.binPath, fileConfig, codeMaps,
                "bash")
        }
    }

    /**
     * Copy Python specific target code to the src-gen directory
     * Also, copy all files listed in the target property `files` into the
     * src-gen folder of the main .lf file.
     * 
     * @param targetConfig The targetConfig to read the `files` from.
     * @param fileConfig The fileConfig used to make the copy and resolve paths.
     */
    override copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {
        super.copyUserFiles(targetConfig, fileConfig);
        // Copy the required target language files into the target file system.
        // This will also overwrite previous versions.
        fileConfig.copyFileFromClassPath(
            "/lib/py/reactor-c-py/include/pythontarget.h",
            fileConfig.getSrcGenPath.resolve("pythontarget.h").toString
        )
        fileConfig.copyFileFromClassPath(
            "/lib/py/reactor-c-py/lib/pythontarget.c",
            fileConfig.getSrcGenPath.resolve("pythontarget.c").toString
        )
        fileConfig.copyFileFromClassPath(
            "/lib/c/reactor-c/include/ctarget.h",
            fileConfig.getSrcGenPath.resolve("ctarget.h").toString
        )
    }

    /** Return the function name in Python
     *  @param reactor The reactor
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    def pythonReactionFunctionName(int reactionIndex) {
        "reaction_function_" + reactionIndex
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    override generateDelayBody(Action action, VarRef port) {
        val ref = JavaAstUtils.generateVarRef(port);
        // Note that the action.type set by the base class is actually
        // the port type.
        if (action.inferredType.isTokenType) {
            '''
                if («ref»->is_present) {
                    // Put the whole token on the event queue, not just the payload.
                    // This way, the length and element_size are transported.
                    schedule_token(«action.name», 0, «ref»->token);
                }
            '''
        } else {
            '''
                // Create a token.
                #if NUMBER_OF_WORKERS > 0
                // Need to lock the mutex first.
                lf_mutex_lock(&mutex);
                #endif
                lf_token_t* t = create_token(sizeof(PyObject*));
                #if NUMBER_OF_WORKERS > 0
                lf_mutex_unlock(&mutex);
                #endif
                t->value = self->_lf_«ref»->value;
                t->length = 1; // Length is 1
                
                // Pass the token along
                schedule_token(«action.name», 0, t);
            '''
        }
    }

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port. This realizes
     * the receiving end of a logical delay specified with the 'after'
     * keyword.
     * @param action The action that triggers the reaction
     * @param port The port to write to.
     */
    override generateForwardBody(Action action, VarRef port) {
        val outputName = JavaAstUtils.generateVarRef(port)
        if (action.inferredType.isTokenType) {
            super.generateForwardBody(action, port)
        } else {
            '''
                SET(«outputName», «action.name»->token->value);
            '''
        }
    }
    
    
    /**
     * Generate necessary Python-specific initialization code for <code>reaction<code> that belongs to reactor 
     * <code>decl<code>.
     * 
     * @param reaction The reaction to generate Python-specific initialization for.
     * @param decl The reactor to which <code>reaction<code> belongs to.
     * @param pyObjectDescriptor For each port object created, a Python-specific descriptor will be added to this that
     *  then can be used as an argument to <code>Py_BuildValue<code> 
     *  (@see <a href="https://docs.python.org/3/c-api/arg.html#c.Py_BuildValue">docs.python.org/3/c-api</a>).
     * @param pyObjects A "," delimited list of expressions that would be (or result in a creation of) a PyObject.
     */
    protected def void generatePythonInitializationForReaction(
        Reaction reaction,
        ReactorDecl decl,
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects
    ) {
        var actionsAsTriggers = new LinkedHashSet<Action>();
        val Reactor reactor = decl.toDefinition;

        // Next, add the triggers (input and actions; timers are not needed).
        // TODO: handle triggers
        for (TriggerRef trigger : reaction.triggers ?: emptyList) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    generatePortVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, trigger, decl)
                } else if (trigger.variable instanceof Action) {
                    actionsAsTriggers.add(trigger.variable as Action)
                    generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                        trigger.variable as Action, decl)
                }
            }
        }
        if (reaction.triggers === null || reaction.triggers.size === 0) {
            // No triggers are given, which means react to any input.
            // Declare an argument for every input.
            // NOTE: this does not include contained outputs. 
            for (input : reactor.inputs) {
                generateInputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, input, decl)
            }
        }

        // Next add non-triggering inputs.
        for (VarRef src : reaction.sources ?: emptyList) {
            if (src.variable instanceof Port) {
                generatePortVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, src, decl)
            } else if (src.variable instanceof Action) {
                // TODO: handle actions
                actionsAsTriggers.add(src.variable as Action)
                generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects, src.variable as Action,
                    decl)
            }
        }

        // Next, handle effects
        if (reaction.effects !== null) {
            for (effect : reaction.effects) {
                if (effect.variable instanceof Action) {
                    // It is an action, not an output.
                    // If it has already appeared as trigger, do not redefine it.
                    if (!actionsAsTriggers.contains(effect.variable)) {
                        generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                            effect.variable as Action, decl)
                    }
                } else {
                    if (effect.variable instanceof Output) {
                        generateOutputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                            effect.variable as Output, decl)
                    } else if (effect.variable instanceof Input) {
                        // It is the input of a contained reactor.
                        generateVariablesForSendingToContainedReactors(pyObjectDescriptor, pyObjects, effect.container,
                            effect.variable as Input, decl)
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): " + effect.variable.name + " is neither an input nor an output."
                        )
                    }

                }
            }
        }
    }

    /** Generate a reaction function definition for a reactor.
     *  This function has a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reaction The reaction.
     *  @param reactor The reactor.
     *  @param reactionIndex The position of the reaction within the reactor. 
     */
    override generateReaction(Reaction reaction, ReactorDecl decl, int reactionIndex) {

        val reactor = decl.toDefinition

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.name.contains(GEN_DELAY_CLASS_NAME) ||
            ((decl === this.mainDef?.reactorClass) && reactor.isFederated)) {
            super.generateReaction(reaction, decl, reactionIndex)
            return
        }

        // Contains "O" characters. The number of these characters depend on the number of inputs to the reaction
        val StringBuilder pyObjectDescriptor = new StringBuilder()

        // Contains the actual comma separated list of inputs to the reaction of type generic_port_instance_struct or generic_port_instance_with_token_struct.
        // Each input must be cast to (PyObject *)
        val StringBuilder pyObjects = new StringBuilder()

        // Create a unique function name for each reaction.
        val functionName = reactionFunctionName(decl, reactionIndex)

        // Generate the function name in Python
        val pythonFunctionName = pythonReactionFunctionName(reactionIndex);

        pr('void ' + functionName + '(void* instance_args) {')
        indent()

        // First, generate C initializations
        super.generateInitializationForReaction("", reaction, decl, reactionIndex)
        
        prSourceLineNumber(reaction.code)

        // Ensure that GIL is locked
        pr('''
            // Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.         
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();
        ''')
        
        // Generate Python-related initializations
        generatePythonInitializationForReaction(reaction, decl, pyObjectDescriptor, pyObjects)
        
        // Call the Python reaction
        pr('''
            
            DEBUG_PRINT("Calling reaction function «decl.name».«pythonFunctionName»");
            PyObject *rValue = PyObject_CallObject(
                self->_lf_py_reaction_function_«reactionIndex», 
                Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»)
            );
            if (rValue == NULL) {
                error_print("FATAL: Calling reaction «decl.name».«pythonFunctionName» failed.");
                if (PyErr_Occurred()) {
                    PyErr_PrintEx(0);
                    PyErr_Clear(); // this will reset the error indicator so we can run Python code again
                }
                /* Release the thread. No Python API allowed beyond this point. */
                PyGILState_Release(gstate);
                Py_FinalizeEx();
                exit(1);
            }
            
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
        ''')

        unindent()
        pr("}")

        // Now generate code for the deadline violation function, if there is one.
        if (reaction.deadline !== null) {
            // The following name has to match the choice in generateReactionInstances
            val deadlineFunctionName = decl.name.toLowerCase + '_deadline_function' + reactionIndex

            pr('void ' + deadlineFunctionName + '(void* instance_args) {')
            indent();

            super.generateInitializationForReaction("", reaction, decl, reactionIndex)

            pr('''
                // Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.         
                PyGILState_STATE gstate;
                gstate = PyGILState_Ensure();
                
                DEBUG_PRINT("Calling deadline function «decl.name».«deadlineFunctionName»");
                PyObject *rValue = PyObject_CallObject(
                    self->_lf_py_deadline_function_«reactionIndex», 
                    Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»)
                );
                if (rValue == NULL) {
                    error_print("FATAL: Calling reaction «decl.name».«deadlineFunctionName» failed.\n");
                    if (rValue == NULL) {
                        if (PyErr_Occurred()) {
                            PyErr_PrintEx(0);
                            PyErr_Clear(); // this will reset the error indicator so we can run Python code again
                        }
                    }
                    /* Release the thread. No Python API allowed beyond this point. */
                    PyGILState_Release(gstate);
                    Py_FinalizeEx();
                    exit(1);
                }
                
                /* Release the thread. No Python API allowed beyond this point. */
                PyGILState_Release(gstate);
            ''')

            unindent()
            pr("}")
        }
    }

    /**
     * Generate code for parameter variables of a reactor in the form "parameter.type parameter.name;"
     * 
     * FIXME: for now we assume all parameters are int. This is to circumvent the issue of parameterized
     * port widths for now.
     * 
     * @param reactor The reactor
     * @param builder The StringBuilder that the generated code is appended to
     * @return 
     */
    override generateParametersForReactor(StringBuilder builder, Reactor reactor) {
        for (parameter : reactor.allParameters) {
            prSourceLineNumber(builder, parameter)
            // Assume all parameters are integers
            pr(builder, '''int «parameter.name» ;''');
        }
    }

    /**
     * Generate code that initializes the state variables for a given instance.
     * Unlike parameters, state variables are uniformly initialized for all instances
     * of the same reactor. This task is left to Python code to allow for more liberal
     * state variable assignments.
     * @param instance The reactor class instance
     * @return Initialization code fore state variables of instance
     */
    override generateStateVariableInitializations(ReactorInstance instance) {
        // Do nothing
    }

    /**
     * Generate runtime initialization code in C for parameters of a given reactor instance.
     * All parameters are also initialized in Python code, but those parameters that are
     * used as width must be also initialized in C.
     * 
     * FIXME: Here, we use a hack: we attempt to convert the parameter initialization to an integer.
     * If it succeeds, we proceed with the C initialization. If it fails, we defer initialization
     * to Python.
     * 
     * Generate runtime initialization code for parameters of a given reactor instance
     * @param instance The reactor instance.
     */
    override void generateParameterInitialization(ReactorInstance instance) {
        // Mostly ignore the initialization in C
        // The actual initialization will be done in Python
        // Except if the parameter is a width (an integer)
        // Here, we attempt to convert the parameter value to 
        // integer. If it succeeds, we also initialize it in C.
        // If it fails, we defer the initialization to Python.
        var nameOfSelfStruct = CUtil.reactorRef(instance)
        for (parameter : instance.parameters) {
            val initializer = parameter.getInitializer
            try {
                // Attempt to convert it to integer
                val number = Integer.parseInt(initializer);
                pr(initializeTriggerObjects, '''
                    «nameOfSelfStruct»->«parameter.name» = «number»;
                ''')
            } catch (NumberFormatException ex) {
                // Ignore initialization in C for this parameter
            }
        }
    }

    /**
     * This function is overridden in the Python generator to do nothing.
     * The state variables are initialized in Python code directly.
     * @param reactor The reactor
     * @param builder The StringBuilder that the generated code is appended to
     * @return 
     */
    override generateStateVariablesForReactor(StringBuilder builder, Reactor reactor) {
        // Do nothing
    }

    /**
     * Generates C preambles defined by user for a given reactor
     * Since the Python generator expects preambles written in C,
     * this function is overridden and does nothing.
     * @param reactor The given reactor
     */
    override generateUserPreamblesForReactor(Reactor reactor) {
        // Do nothing
    }

    /**
     * Generate code that is executed while the reactor instance is being initialized.
     * This wraps the reaction functions in a Python function.
     * @param instance The reactor instance.
     * @param reactions The reactions of this instance.
     */
    override void generateReactorInstanceExtension(
        ReactorInstance instance,
        Iterable<ReactionInstance> reactions
    ) {
        var nameOfSelfStruct = CUtil.reactorRef(instance)
        var reactor = instance.definition.reactorClass.toDefinition

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.name.contains(GEN_DELAY_CLASS_NAME) ||
            ((instance.definition.reactorClass === this.mainDef?.reactorClass) && reactor.isFederated)) {
            return
        }

        // Initialize the name field to the unique name of the instance
        pr(initializeTriggerObjects, '''
            «nameOfSelfStruct»->_lf_name = "«instance.uniqueID»_lf";
        ''');

        for (reaction : reactions) {
            val pythonFunctionName = pythonReactionFunctionName(reaction.index)
            // Create a PyObject for each reaction
            pr(initializeTriggerObjects, '''
                «nameOfSelfStruct»->_lf_py_reaction_function_«reaction.index» = 
                    get_python_function("__main__", 
                        «nameOfSelfStruct»->_lf_name,
                        «CUtil.runtimeIndex(instance)»,
                        "«pythonFunctionName»");
            ''')

            if (reaction.definition.deadline !== null) {
                pr(initializeTriggerObjects, '''
                    «nameOfSelfStruct»->_lf_py_deadline_function_«reaction.index» = 
                        get_python_function("__main__", 
                            «nameOfSelfStruct»->_lf_name,
                            «CUtil.runtimeIndex(instance)»,
                            "deadline_function_«reaction.index»");
                ''')
            }
        }
    }

    /**
     * This function is provided to allow extensions of the CGenerator to append the structure of the self struct
     * @param selfStructBody The body of the self struct
     * @param decl The reactor declaration for the self struct
     * @param instance The current federate instance
     * @param constructorCode Code that is executed when the reactor is instantiated
     * @param destructorCode Code that is executed when the reactor instance is freed
     */
    override generateSelfStructExtension(StringBuilder selfStructBody, ReactorDecl decl, FederateInstance instance,
        StringBuilder constructorCode, StringBuilder destructorCode) {
        val reactor = decl.toDefinition
        // Add the name field
        pr(selfStructBody, '''char *_lf_name;
        ''');

        var reactionIndex = 0
        for (reaction : reactor.allReactions) {
            // Create a PyObject for each reaction
            pr(selfStructBody, '''PyObject* _lf_py_reaction_function_«reactionIndex»;''')

            if (reaction.deadline !== null) {
                pr(selfStructBody, '''PyObject* _lf_py_deadline_function_«reactionIndex»;''')
            }

            reactionIndex++
        }
    }

    /**
     * Generate code to convert C actions to Python action capsules
     * @see pythontarget.h
     * @param builder The string builder into which to write the code.
     * @param structs A map from reactor instantiations to a place to write
     *        struct fields.
     * @param port The port.
     * @param reactor The reactor.
     */
    def generateActionVariableToSendToPythonReaction(StringBuilder pyObjectDescriptor, StringBuilder pyObjects,
        Action action, ReactorDecl decl) {
        pyObjectDescriptor.append("O")
        // Values passed to an action are always stored in the token->value.
        // However, sometimes token might not be initialized. Therefore, this function has an internal check for NULL in case token is not initialized.
        pyObjects.append(''', convert_C_action_to_py(«action.name»)''')
    }

    /** Generate into the specified string builder the code to
     *  send local variables for ports to a Python reaction function
     *  from the "self" struct. The port may be an input of the
     *  reactor or an output of a contained reactor. The second
     *  argument provides, for each contained reactor, a place to
     *  write the declaration of the output of that reactor that
     *  is triggering reactions.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param port The port.
     *  @param reactor The reactor.
     */
    private def generatePortVariablesToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        VarRef port,
        ReactorDecl decl
    ) {
        if (port.variable instanceof Input) {
            generateInputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, port.variable as Input, decl)
        } else {
            pyObjectDescriptor.append("O")
            val output = port.variable as Output
            val reactorName = port.container.name
            // port is an output of a contained reactor.
            if (port.container.widthSpec !== null) {
                var String widthSpec = "-2"
                if (JavaAstUtils.isMultiport(port.variable as Port)) {
                    widthSpec = '''self->_lf_«reactorName»[i].«output.name»_width'''
                }
                // Output is in a bank.
                // Create a Python list
                generatePythonListForContainedBank(reactorName, output, widthSpec)
                pyObjects.append(''', «reactorName»_py_list''')
            } else {
                var String widthSpec = "-2"
                if (JavaAstUtils.isMultiport(port.variable as Port)) {
                    widthSpec = '''«port.container.name».«port.variable.name»_width'''
                }
                pyObjects.append(''', convert_C_port_to_py(«reactorName».«port.variable.name», «widthSpec»)''')
            }
        }
    }
    
    /**
     * Generate code that creates a Python list (i.e., []) for contained banks to be passed to Python reactions.
     * The Python reaction will then subsequently be able to address each individual bank member of the contained 
     * bank using an index or an iterator. Each list member will contain the given <code>port<code> 
     * (which could be a multiport with a width determined by <code>widthSpec<code>).
     * 
     * This is to accommodate reactions like <code>reaction() -> s.out<code> where s is a bank. In this example,
     * the generate Python function will have the signature <code>reaction_function_0(self, s_out)<code>, where
     * s_out is a list of out ports. This will later be turned into the proper <code>s.out<code> format using the
     * Python code generated in {@link #generatePythonPortVariableInReaction}.
     * 
     * @param reactorName The name of the bank of reactors (which is the name of the reactor class).
     * @param port The port that should be put in the Python list.
     * @param widthSpec A string that should be -2 for non-multiports and the width expression for multiports.
     */
    protected def void generatePythonListForContainedBank(String reactorName, Port port, String widthSpec) {
        pr('''
            PyObject* «reactorName»_py_list = PyList_New(«reactorName»_width);
            
            if(«reactorName»_py_list == NULL) {
                error_print("Could not create the list needed for «reactorName».");
                if (PyErr_Occurred()) {
                    PyErr_PrintEx(0);
                    PyErr_Clear(); // this will reset the error indicator so we can run Python code again
                }
                /* Release the thread. No Python API allowed beyond this point. */
                PyGILState_Release(gstate);
                Py_FinalizeEx();
                exit(1);
            }
            
            for (int i = 0; i < «reactorName»_width; i++) {
                if (PyList_SetItem(
                        «reactorName»_py_list,
                        i,
                        convert_C_port_to_py(
                            self->_lf_«reactorName»[i].«port.name», 
                            «widthSpec»
                        )
                    ) != 0) {
                    error_print("Could not add elements to the list for «reactorName».");
                    if (PyErr_Occurred()) {
                        PyErr_PrintEx(0);
                        PyErr_Clear(); // this will reset the error indicator so we can run Python code again
                    }
                    /* Release the thread. No Python API allowed beyond this point. */
                    PyGILState_Release(gstate);
                    Py_FinalizeEx();
                    exit(1);
                }
            }
            
        ''')
    }

    /** Generate into the specified string builder the code to
     *  send local variables for output ports to a Python reaction function
     *  from the "self" struct.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param output The output port.
     *  @param decl The reactor declaration.
     */
    private def generateOutputVariablesToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Output output,
        ReactorDecl decl
    ) {
        // Unfortunately, for the SET macros to work out-of-the-box for
        // multiports, we need an array of *pointers* to the output structs,
        // but what we have on the self struct is an array of output structs.
        // So we have to handle multiports specially here a construct that
        // array of pointers.
        // FIXME: The C Generator also has this awkwardness. It makes the code generators
        // unnecessarily difficult to maintain, and it may have performance consequences as well.
        // Maybe we should change the SET macros.
        if (!JavaAstUtils.isMultiport(output)) {
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«output.name», -2)''')
        } else {
            // Set the _width variable.                
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«output.name»,«output.name»_width) ''')
        }
    }

    /** Generate into the specified string builder the code to
     *  pass local variables for sending data to an input
     *  of a contained reaction (e.g. for a deadline violation).
     *  @param builder The string builder.
     *  @param definition AST node defining the reactor within which this occurs
     *  @param input Input of the contained reactor.
     */
    private def generateVariablesForSendingToContainedReactors(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Instantiation definition,
        Input input,
        ReactorDecl decl
    ) {
        pyObjectDescriptor.append("O")
        
        if (definition.widthSpec !== null) {
            var String widthSpec = "-2"
            if (JavaAstUtils.isMultiport(input)) {
                widthSpec = '''self->_lf_«definition.name»[i].«input.name»_width'''
            }
            // Contained reactor is a bank.
            // Create a Python list
            generatePythonListForContainedBank(definition.name, input, widthSpec);
            pyObjects.append(''', «definition.name»_py_list''')
        }
        else {
            var String widthSpec = "-2"
            if (JavaAstUtils.isMultiport(input)) {
                widthSpec = '''«definition.name».«input.name»_width'''
            }
            pyObjects.
                append(''', convert_C_port_to_py(«definition.name».«input.name», «widthSpec»)''')
        }
    }

    /** Generate into the specified string builder the code to
     *  send local variables for input ports to a Python reaction function
     *  from the "self" struct.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param input The input port.
     *  @param reactor The reactor.
     */
    private def generateInputVariablesToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Input input,
        ReactorDecl decl
    ) {
        // Create the local variable whose name matches the input name.
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value (hence, as done
        // below, we have to use writable_copy()). There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable && !JavaAstUtils.isMultiport(input)) {
            // Non-mutable, non-multiport, primitive type.
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name», «input.name»_width)''')
        } else if (input.isMutable && !JavaAstUtils.isMultiport(input)) {
            // Mutable, non-multiport, primitive type.
            // TODO: handle mutable
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name», «input.name»_width)''')
        } else if (!input.isMutable && JavaAstUtils.isMultiport(input)) {
            // Non-mutable, multiport, primitive.
            // TODO: support multiports
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name»,«input.name»_width) ''')
        } else {
            // Mutable, multiport, primitive type
            // TODO: support mutable multiports
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name»,«input.name»_width) ''')
        }
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * The file will go into src-gen/filename.Dockerfile.
     * If there is no main reactor, then no Dockerfile will be generated
     * (it wouldn't be very useful).
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    override writeDockerFile(File dockerComposeDir, String dockerFileName, String federateName) {
        var srcGenPath = fileConfig.getSrcGenPath
        val dockerFile = srcGenPath + File.separator + dockerFileName
        // If a dockerfile exists, remove it.
        var file = new File(dockerFile)
        if (file.exists) {
            file.delete
        }

        if (this.mainDef === null) {
            return
        }

        val OS = System.getProperty("os.name").toLowerCase();
        var dockerComposeCommand = (OS.indexOf("nux") >= 0) ? "docker-compose" : "docker compose"

        val contents = new StringBuilder()
        pr(contents, '''
            # Generated docker file for «topLevelName».lf in «srcGenPath».
            # For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution
            FROM python:slim
            WORKDIR /lingua-franca/«topLevelName»
            RUN set -ex && apt-get update && apt-get install -y python3-pip
            COPY . src-gen
            RUN cd src-gen && python3 setup.py install && cd ..
            ENTRYPOINT ["python3", "src-gen/«topLevelName».py"]
        ''')
        JavaGeneratorUtils.writeSourceCodeToFile(contents, dockerFile)
        println('''Dockerfile for «topLevelName» written to ''' + dockerFile)
        println('''
            #####################################
            To build the docker image, go to «dockerComposeDir» and run:
               
                «dockerComposeCommand» build «federateName»
            
            #####################################
        ''')
    }

    /**
     * Convert C types to formats used in Py_BuildValue and PyArg_PurseTuple.
     * This is unused but will be useful to enable inter-compatibility between 
     * C and Python reactors.
     * @param type C type
     */
    def pyBuildValueArgumentType(String type) {
        switch (type) {
            case "int": "i"
            case "string": "s"
            case "char": "b"
            case "short int": "h"
            case "long": "l"
            case "unsigned char": "B"
            case "unsigned short int": "H"
            case "unsigned int": "I"
            case "unsigned long": "k"
            case "long long": "L"
            case "interval_t": "L"
            case "unsigned long long": "K"
            case "double": "d"
            case "float": "f"
            case "Py_complex": "D"
            case "Py_complex*": "D"
            case "Py_Object": "O"
            case "Py_Object*": "O"
            default: "O"
        }
    }
}
