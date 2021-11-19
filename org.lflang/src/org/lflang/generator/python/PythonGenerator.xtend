/* Generator for the Python target. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator.python

import java.io.File
import java.util.ArrayList
import java.util.LinkedHashSet
import java.util.List
import java.util.regex.Pattern
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetProperty.CoordinationType
import org.lflang.federated.FedFileConfig
import org.lflang.federated.FederateInstance
import org.lflang.federated.PythonGeneratorExtension
import org.lflang.federated.launcher.FedPyLauncher
import org.lflang.federated.serialization.FedNativePythonSerialization
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.generator.ParameterInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance
import org.lflang.generator.c.CCompiler
import org.lflang.generator.c.CGenerator
import org.lflang.generator.c.CUtil
import org.lflang.lf.Action
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
 * A backend is also generated using the CGenrator that interacts with the C code library (see CGenerator.xtend).
 * The backend is responsible for passing arguments to the Python reactor functions.
 *
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
class PythonGenerator extends CGenerator {
	
	// Used to add statements that come before reactor classes and user code
    var pythonPreamble = new StringBuilder()

    // Used to add module requirements to setup.py (delimited with ,)
    var pythonRequiredModules = new StringBuilder()

    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        super(fileConfig, errorReporter)
        // set defaults
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags = newArrayList // -Wall -Wconversion"
        targetConfig.linkerFlags = ""
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
	val generic_port_type =  "generic_port_instance_struct"

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
	
	override getTargetUndefinedType() '''PyObject*'''
	
	/** Returns the Target enum for this generator */
    override getTarget() {
        return Target.Python
    }

	// Regular expression pattern for pointer types. The star at the end has to be visible.
    static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");
    
    ////////////////////////////////////////////
    //// Public methods
    override printInfo() {
        println("Generating code for: " + fileConfig.resource.getURI.toString)
        println('******** Mode: ' + fileConfig.compilerMode)
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
    
    ////////////////////////////////////////////
    //// Protected methods
    
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
        switch(v.toText) {
            case "false": returnValue = "False"
            case "true": returnValue = "True"
            default: returnValue = super.getTargetValue(v)
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
                list.add(i.parameter.targetReference)
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
     * Create a Python list for parameter initialization in target code.
     * 
     * @param p The parameter instance to create initializers for
     * @return Initialization code
     */
     protected def String getPythonInitializer(ParameterInstance p) {        
            if (p.init.size > 1) {
                // parameters are initialized as immutable tuples
                return p.init.join('(', ', ', ')', [it.pythonTargetValue])
            } else {
                return p.init.get(0).getPythonTargetValue
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
    def generatePythonReactionParametersAndInitializations(StringBuilder parameters, StringBuilder inits, ReactorDecl decl,
        Reaction reaction) {
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
                            if ((trigger.variable as Input).isMultiport) {
                                inits.
                                    append('''«trigger.variable.name» = [Make() for i in range(len(mutable_«trigger.variable.name»))]
                                    ''')
                                inits.append('''for i in range(len(mutable_«trigger.variable.name»)):
                                ''')
                                inits.
                                    append('''    «trigger.variable.name»[i].value = copy.deepcopy(mutable_«trigger.variable.name»[i].value)
                                    ''')
                            } else {
                                inits.append('''«trigger.variable.name» = Make
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
                        inits.append('''«trigger.container.name» = Make
                        ''')
                        inits.
                            append('''«trigger.container.name».«trigger.variable.name» = «trigger.container.name»_«trigger.variable.name»
                            ''')
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
                inits.append('''«src.container.name» = Make
                ''')
                inits.append('''«src.container.name».«src.variable.name» = «src.container.name»_«src.variable.name»
                ''')
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
                inits.append('''«effect.container.name» = Make
                ''')
                inits.
                    append('''«effect.container.name».«effect.variable.name» = «effect.container.name»_«effect.variable.name»
                    ''')
            } else {
                generatedParams.add(effect.variable.name)
                if (effect.variable instanceof Port) {
                    if (isMultiport(effect.variable as Port)) {
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
     * Handle initialization for state variable
     * @param state a state variable
     */
    def String getTargetInitializer(StateVar state) {
        if(!state.isInitialized)
        {
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
    def generatePythonReactorClass(ReactorInstance instance, StringBuilder pythonClasses, FederateInstance federate)
    {
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

        // Do not generate classes that don't have any reactions
        // Do not generate the main federated class, which is always implemented in C
        if (!instance.definition.reactorClass.toDefinition.allReactions.isEmpty && !decl.toDefinition.isFederated) {
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
                

                var reactionIndex = 0
                for (reaction : reactor.allReactions) {
                    val reactionParameters = new StringBuilder() // Will contain parameters for the function (e.g., Foo(x,y,z,...)
                    val inits = new StringBuilder() // Will contain initialization code for some parameters
                    generatePythonReactionParametersAndInitializations(reactionParameters, inits, reactor, reaction)
                    pythonClasses.append('''    def «pythonReactionFunctionName(reactionIndex)»(self «reactionParameters»):
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
            if (!param.inferredType.targetType.equals("PyObject*")) {
                // If type is given, use it
                temporary_code.
                    append('''        self._«param.name»:«param.inferredType.pythonType» = «param.pythonInitializer»
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
            if (!stateVar.inferredType.targetType.equals("PyObject*")) {
                // If type is given, use it
                temporary_code.
                    append('''        self.«stateVar.name»:«stateVar.inferredType.pythonType» = «stateVar.pythonInitializer»
                    ''')
            } else if (stateVar.isInitialized) {
                // If type is not given, pass along the initialization directly if it is present
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
            temporary_code.append('''    @property
            ''') 
            temporary_code.append('''    def «param.name»(self):
            ''')
            temporary_code.append('''        return self._«param.name»
            
            ''')
        }
        
        return temporary_code;
    }
    
    /**
     * Generate the function that is executed whenever the deadline of the reaction
     * with the given reaction index is missed
     * @param reaction The reaction to generate deadline miss code for
     * @param reactionIndex The agreed-upon index of the reaction in the reactor (should match the C generated code)
     * @param reactionParameters The parameters to the deadline violation function, which are the same as the reaction function
     */
    def generateDeadlineFunctionForReaction(Reaction reaction, int reactionIndex, String reactionParameters)'''
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
     * This generator inherits types from the CGenerator.
     * This function reverts them back to Python types
     * For example, the types double is converted to float,
     * the * for pointer types is removed, etc.
     * @param type The type
     * @return The Python equivalent of a C type
     */
    def getPythonType(InferredType type) {
        var result = super.getTargetType(type)
        
        switch(result){
            case "double": result = "float"
            case "string": result = "object"
            default: result = result
        }
        
        val matcher = pointerPatternVariable.matcher(result)
        if(matcher.find()) {
            return matcher.group(1)
        }
        
        return result
    }
    
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
        if(className.contains(GEN_DELAY_CLASS_NAME)) {
            return
        }

        // Do not instantiate reactor classes that don't have a reaction in Python
        // Do not instantiate the federated main reactor since it is generated in C
        if (!instance.definition.reactorClass.toDefinition.allReactions.isEmpty && !instance.definition.reactorClass.toDefinition.isFederated) {
            if (federate.contains(instance) && instance.width > 0 && !instance.definition.reactorClass.toDefinition.allReactions.isEmpty) {
                if (instance.isBank) {
                    // If this reactor is a bank, then generate
                    // a list of instances of reactors and return.         
                    pythonClassesInstantiation.
                        append('''
                        «instance.uniqueID»_lf = \
                            _«className»(bank_index = 0 /* bank_index is specially assigned by us*/»,\
                                «FOR param : instance.parameters SEPARATOR ", "»_«param.name»=«param.pythonInitializer»«ENDFOR»)
                        ''')
                        for (var i = 1; i < instance.width; i++) {
                            pythonClassesInstantiation.
                                append('''
                                    _«className»(bank_index = «i» /* bank_index is specially assigned by us*/,\
                                        «FOR param : instance.parameters SEPARATOR ", "»_«param.name»=«param.pythonInitializer»«ENDFOR»), \\\n
                                ''')
                        }
                        pythonClassesInstantiation.
                            append('''
                                ]
                            ''')
                    return
                } else {
                    // FIXME: Why does this add a bank_index if it's not a bank?
                    pythonClassesInstantiation.append('''
                        «instance.uniqueID»_lf = \
                            [_«className»(bank_index = 0 /* bank_index is specially assigned by us*/, \
                                «FOR param : instance.parameters SEPARATOR ", \\\n"»_«param.name»=«param.pythonInitializer»«ENDFOR»)]
                    ''')
                }
            }
        }

        for (child : instance.children) {
            generatePythonClassInstantiation(child, pythonClassesInstantiation, federate)
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
        
        // Instantiate generated classes
        this.main.generatePythonClassInstantiation(pythonClassesInstantiation, federate)

        '''«pythonClasses»
        
        ''' +
        '''# Instantiate classes
        ''' +
        '''«pythonClassesInstantiation»
        '''
    }
    
    /**
     * Generate the Python code constructed from reactor classes and user-written classes.
     * @return the code body 
     */
    def generatePythonCode(FederateInstance federate) '''
       from LinguaFranca«topLevelName» import *
       from LinguaFrancaBase.constants import * #Useful constants
       from LinguaFrancaBase.functions import * #Useful helper functions
       from LinguaFrancaBase.classes import * #Useful classes
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
    def generatePythonFiles(IFileSystemAccess2 fsa, FederateInstance federate)
    {
        var file = new File(fileConfig.getSrcGenPath.toFile,  topLevelName + ".py")
        if (file.exists) {
            file.delete
        }
        // Create the necessary directories
        if (!file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }
        writeSourceCodeToFile(generatePythonCode(federate).toString.bytes, file.absolutePath)
        
        val setupPath = fileConfig.getSrcGenPath.resolve("setup.py")
        // Handle Python setup
        System.out.println("Generating setup file to " + setupPath)
        file = setupPath.toFile
        if (file.exists) {
            // Append
            file.delete
        }
            
        // Create the setup file
        writeSourceCodeToFile(generatePythonSetupFile.toString.bytes, setupPath.toString)
             
        
    }
    
    /**
     * Execute the command that compiles and installs the current Python module
     */
    def pythonCompileCode(IGeneratorContext context) {
        // if we found the compile command, we will also find the install command
        val installCmd = commandFactory.createCommand(
            '''python3''',
            #["-m", "pip", "install", "--force-reinstall", "."],
            fileConfig.srcGenPath)
               
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
            errorReporter.reportError("Failed to install python extension.")
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

        pr('#define NUMBER_OF_FEDERATES ' + federates.size);

        // Handle target parameters.
        // First, if there are federates, then ensure that threading is enabled.
        if (targetConfig.threads === 0 && isFederated) {
            targetConfig.threads = 1
        }

        super.includeTargetLanguageSourceFiles()

        super.parseTargetParameters()
        
        // FIXME: Probably not the best place to do 
        // this.
        if (!targetConfig.protoFiles.isNullOrEmpty) {
            // Enable support for proto serialization
            enabledSerializers.add(SupportedSerializers.PROTO)
        }
    }
    
    /**
     * Add necessary code to the source and necessary build supports to
     * enable the requested serializations in 'enabledSerializations'
     */  
    override enableSupportForSerialization(CancelIndicator cancelIndicator) {
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
         val protoc = commandFactory.createCommand(
            "protoc",
            #['''--python_out=«this.fileConfig.getSrcGenPath»''', filename],
            fileConfig.srcPath)
         //val protoc = createCommand("protoc", #['''--python_out=src-gen/«topLevelName»''', topLevelName], codeGenConfig.outPath)
        if (protoc === null) {
            errorReporter.reportError("Processing .proto files requires libprotoc >= 3.6.1")
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
            action, sendingPort, receivingPort,
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
        if (CCompiler.isHostWindows) { 
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
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        
        // If there are federates, assign the number of threads in the CGenerator to 1        
        if(isFederated) {
            targetConfig.threads = 1;
        }
        
        // Prevent the CGenerator from compiling the C code.
        // The PythonGenerator will compiler it.
        val compileStatus = targetConfig.noCompile;
        targetConfig.noCompile = true;
        targetConfig.useCmake = false; // Force disable the CMake because 
                                       // it interferes with the Python target functionality
        
        super.doGenerate(resource, fsa, context)
        
        targetConfig.noCompile = compileStatus

        if (errorsOccurred) return;
        
        var baseFileName = topLevelName
        // Keep a separate file config for each federate
        val oldFileConfig = fileConfig;
        for (federate : federates) {
            if (isFederated) {
                topLevelName = baseFileName + '_' + federate.name
                fileConfig = new FedFileConfig(fileConfig, federate.name);
            }
            // Don't generate code if there is no main reactor
            if (this.main !== null) {
                generatePythonFiles(fsa, federate)
                if (targetConfig.noCompile !== true) {
                    // If there are no federates, compile and install the generated code
                    pythonCompileCode(context)
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
        val ref = generateVarRef(port);
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
        val outputName = generateVarRef(port)
        if (action.inferredType.isTokenType) {
            // Forward the entire token and prevent freeing.
            // Increment the ref_count because it will be decremented
            // by both the action handling code and the input handling code.
            '''
            «DISABLE_REACTION_INITIALIZATION_MARKER»
            self->_lf_«outputName».value = («action.inferredType.targetType»)self->_lf__«action.name».token->value;
            self->_lf_«outputName».token = (lf_token_t*)self->_lf__«action.name».token;
            ((lf_token_t*)self->_lf__«action.name».token)->ref_count++;
            self->«getStackPortMember('''_lf_«outputName»''', "is_present")» = true;
            '''
        } else {
            '''
            SET(«outputName», «action.name»->token->value);
            '''
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
        if(reactor.name.contains(GEN_DELAY_CLASS_NAME) || ((decl === this.mainDef?.reactorClass) && reactor.isFederated)) {
            return super.generateReaction(reaction, decl, reactionIndex)
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
        
        // Actions may appear twice, first as a trigger, then with the outputs.
        // But we need to declare it only once. Collect in this data structure
        // the actions that are declared as triggered so that if they appear
        // again with the outputs, they are not defined a second time.
        // That second redefinition would trigger a compile error.  
        var actionsAsTriggers = new LinkedHashSet<Action>();
        
        // Next, add the triggers (input and actions; timers are not needed).
        // TODO: handle triggers
        for (TriggerRef trigger : reaction.triggers ?: emptyList) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    generatePortVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, trigger, decl)
                } else if (trigger.variable instanceof Action) {
                    actionsAsTriggers.add(trigger.variable as Action)
                    generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects, trigger.variable as Action, decl)
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
            if(src.variable instanceof Port)
            {
                generatePortVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, src, decl)
            } else if (src.variable instanceof Action) {
                //TODO: handle actions
                actionsAsTriggers.add(src.variable as Action)
                generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects, src.variable as Action, decl)
            }
        }
        
        // Next, handle effects
        if (reaction.effects !== null) {
            for (effect : reaction.effects) {
                if(effect.variable instanceof Action)
                {
                    // It is an action, not an output.
                    // If it has already appeared as trigger, do not redefine it.
                    if (!actionsAsTriggers.contains(effect.variable)) {
                        generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects, effect.variable as Action, decl)
                    }
                } else {
                    if (effect.variable instanceof Output) {
                        generateOutputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, effect.variable as Output, decl)
                    } else if (effect.variable instanceof Input ) {
                        // It is the input of a contained reactor.
                        generateVariablesForSendingToContainedReactors(pyObjectDescriptor, pyObjects, effect.container, effect.variable as Input, decl)                
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): " + effect.variable.name + " is neither an input nor an output."
                        )
                    }
                
                }
            }
        }
        
        
        pr('void ' + functionName + '(void* instance_args) {')
        indent()
        
        // First, generate C initializations
        super.generateInitializationForReaction("", reaction, decl, reactionIndex)
        
        
        prSourceLineNumber(reaction.code)
        
        pr('''
            // Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.         
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();
            
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
            pr(builder,'''int «parameter.name» ;''');
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
        var nameOfSelfStruct = CUtil.selfRef(instance)
        for (parameter : instance.parameters) {
            val initializer =  parameter.getInitializer
            try {
                // Attempt to convert it to integer
                val number = Integer.parseInt(initializer);
                pr(initializeTriggerObjects, '''
                    «nameOfSelfStruct»->«parameter.name» = «number»;
                ''')
            } catch (NumberFormatException ex){
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
        ReactorInstance instance, Iterable<ReactionInstance> reactions
    ) {
        var nameOfSelfStruct = CUtil.selfRef(instance)
        var reactor = instance.definition.reactorClass.toDefinition
        
         // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.name.contains(GEN_DELAY_CLASS_NAME) || 
            ((instance.definition.reactorClass === this.mainDef?.reactorClass) 
                && reactor.isFederated)
        ) {
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
                    get_python_function("«topLevelName»", 
                        «nameOfSelfStruct»->_lf_name,
                        «IF (instance.isBank)» i_«instance.depth» «ELSE» «0» «ENDIF»,
                        "«pythonFunctionName»");
                ''')
        
            if (reaction.definition.deadline !== null) {
                pr(initializeTriggerObjects, '''
                «nameOfSelfStruct»->_lf_py_deadline_function_«reaction.index» = 
                    get_python_function("«topLevelName»", 
                        «nameOfSelfStruct»->_lf_name,
                        «IF (instance.isBank)» i_«instance.depth» «ELSE» «0» «ENDIF»,
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
    override generateSelfStructExtension(StringBuilder selfStructBody, ReactorDecl decl, FederateInstance instance, StringBuilder constructorCode, StringBuilder destructorCode) {
        val reactor = decl.toDefinition
        // Add the name field
        pr(selfStructBody, '''char *_lf_name;
        ''');
        
        var reactionIndex = 0
        for (reaction : reactor.allReactions)
        {
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
    def generateActionVariableToSendToPythonReaction(StringBuilder pyObjectDescriptor, StringBuilder pyObjects, Action action, ReactorDecl decl) {
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
    )
    {
        if(port.variable instanceof Input)
        {
            generateInputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, port.variable as Input, decl)
        }
        else
        {
            if(!(port.variable as Port).isMultiport)
            {
                pyObjectDescriptor.append("O")
                pyObjects.append(''', convert_C_port_to_py(«port.container.name».«port.variable.name», -2)''')
            }
            else
            {                
                pyObjectDescriptor.append("O")
                pyObjects.append(''', convert_C_port_to_py(«port.container.name».«port.variable.name», «port.container.name».«port.variable.name»_width) ''')
            }
        }
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
    )
    {
            // Unfortunately, for the SET macros to work out-of-the-box for
            // multiports, we need an array of *pointers* to the output structs,
            // but what we have on the self struct is an array of output structs.
            // So we have to handle multiports specially here a construct that
            // array of pointers.
            if (!output.isMultiport) {
                pyObjectDescriptor.append("O")
                pyObjects.append(''', convert_C_port_to_py(«output.name», -2)''')
            } else if (output.isMultiport) {
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
    )
    {
        if(input.isMultiport)
        {            
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«definition.name».«input.name», «definition.name».«input.name»_width)''')   
        }
        else
        {
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«definition.name».«input.name», -2)''')        
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
    )
    {        
        // Create the local variable whose name matches the input name.
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value (hence, as done
        // below, we have to use writable_copy()). There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable && !input.isMultiport) {
            // Non-mutable, non-multiport, primitive type.
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name», «input.name»_width)''')
        } else if (input.isMutable && !input.isMultiport) {
            // Mutable, non-multiport, primitive type.
            // TODO: handle mutable
            pyObjectDescriptor.append("O")
            pyObjects.append(''', convert_C_port_to_py(«input.name», «input.name»_width)''')
        } else if (!input.isMutable && input.isMultiport) {
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
     * @param The root filename (without any extension).
     */
    override writeDockerFile(String filename) {
        var srcGenPath = fileConfig.getSrcGenPath
        val dockerFile = srcGenPath + File.separator + filename + '.Dockerfile'
        // If a dockerfile exists, remove it.
        var file = new File(dockerFile)
        if (file.exists) {
            file.delete
        }

        if (this.mainDef === null) {
            return
        }

        val contents = new StringBuilder()
        pr(contents, '''
            # Generated docker file for «topLevelName».lf in «srcGenPath».
            # For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution
            FROM python:alpine
            WORKDIR /lingua-franca/«topLevelName»
            COPY . src-gen
            RUN set -ex && apk add --no-cache gcc musl-dev \
             && cd src-gen && python3 setup.py install && cd .. \
             && apk del gcc musl-dev
            ENTRYPOINT ["python3", "src-gen/«filename».py"]
        ''')
        writeSourceCodeToFile(contents.toString.getBytes, dockerFile)
        println("Dockerfile written to " + dockerFile)
        println('''
            #####################################
            To build the docker image, use:
               
                docker build -t «topLevelName.toLowerCase()» -f «dockerFile» «srcGenPath»
            
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
