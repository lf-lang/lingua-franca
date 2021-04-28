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

package org.lflang.generator

import java.io.File
import java.io.FileOutputStream
import java.util.ArrayList
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.List
import java.util.regex.Pattern
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.InferredType
import org.lflang.Target
import org.lflang.lf.Action
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

    new() {
        super()
        // set defaults
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags = newArrayList // -Wall -Wconversion"
        targetConfig.linkerFlags = ""
    }
    	
    /** 
    * Template struct for ports with primitive types and
    * statically allocated arrays in Lingua Franca.
    * This template is defined as
    *     template <class T>
    *     struct template_input_output_port_struct {
    *         T value;
    *         bool is_present;
    *         int num_destinations;
    *     };
    *
    * @see xtext/org.lflang.linguafranca/src/lib/CCpp/ccpptarget.h
    */
	val generic_port_type =  "generic_port_instance_struct"

    /** 
    * Special template struct for ports with dynamically allocated
    * array types (a.k.a. token types) in Lingua Franca.
    * This template is defined as
    *     template <class T>
    *     struct template_input_output_port_struct {
    *         T value;
    *         bool is_present;
    *         int num_destinations;
    *         lf_token_t* token;
    *         int length;
    *     };
    *
    * @see xtext/org.lflang.linguafranca/src/lib/CCpp/ccpptarget.h
    */
	val generic_port_type_with_token = "generic_port_instance_with_token_struct"
	
	override getTargetUndefinedType() '''PyObject*'''
	
	/** Returns the Target enum for this generator */
    override getTarget() {
        return Target.Python
    }

	// Regular expression pattern for pointer types. The star at the end has to be visible.
    static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");
    
   ////////////////////////////////////////////
    //// Public methods
    
    
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
        switch(v.toText) {
            case "false": return "False"
            case "true": return "True"
            default: return super.getTargetValue(v)
        }
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

        var list = new LinkedList<String>();

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
                // parameters are initialized as mutable lists
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
        if (instance !== this.main && !reactorBelongsToFederate(instance, federate)) {
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
            if (reactorBelongsToFederate(instance, federate) && !instantiatedClasses.contains(className)) {

                pythonClasses.append('''
                    class _«className»:
                ''');

                // Generate preamble code
                pythonClasses.append('''
                    
                        «generatePythonPreamblesForReactor(decl.toDefinition)»
                ''')

                val reactor = decl.toDefinition

                // Handle parameters first
                for (param : decl.toDefinition.allParameters) {
                    if (!param.inferredType.targetType.equals("PyObject*")) {
                        // If type is given, use it
                        pythonClasses.
                            append('''    «param.name»:«param.inferredType.pythonType» = «param.pythonInitializer»
                            ''')
                    } else {
                        // If type is not given, just pass along the initialization
                        pythonClasses.append('''    «param.name» = «param.pythonInitializer»
                        ''')

                    }
                }

                // Next, handle state variables
                for (stateVar : reactor.allStateVars) {
                    if (!stateVar.inferredType.targetType.equals("PyObject*")) {
                        // If type is given, use it
                        pythonClasses.
                            append('''    «stateVar.name»:«stateVar.inferredType.pythonType» = «stateVar.pythonInitializer»
                            ''')
                    } else if (stateVar.isInitialized) {
                        // If type is not given, pass along the initialization directly if it is present
                        pythonClasses.append('''    «stateVar.name» = «stateVar.pythonInitializer»
                        ''')
                    } else {
                        // If neither the type nor the initialization is given, use None
                        pythonClasses.append('''    «stateVar.name» = None
                        ''')                        
                    }
                }

                // Handle runtime initializations
                pythonClasses.append('''    
                    «'    '»def __init__(self, **kwargs):
                        «'    '»self.__dict__.update(kwargs)
                ''')

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
        if (instance !== this.main && !reactorBelongsToFederate(instance, federate)) {
            return
        }
        
        val className = instance.definition.reactorClass.name
        
        
        // Do not instantiate delay reactors in Python
        if(className.contains(GEN_DELAY_CLASS_NAME))
        {
            return
        }

        // Do not instantiate reactor classes that don't have a reaction in Python
        // Do not instantiate the federated main reactor since it is generated in C
        if (!instance.definition.reactorClass.toDefinition.allReactions.isEmpty && !instance.definition.reactorClass.toDefinition.isFederated) {
            if (reactorBelongsToFederate(instance, federate) && instance.bankMembers !== null) {
                // If this reactor is a placeholder for a bank of reactors, then generate
                // a list of instances of reactors and return.         
                pythonClassesInstantiation.
                    append('''«instance.uniqueID»_lf = [«FOR member : instance.bankMembers SEPARATOR ", "»_«className»(bank_index = «member.bankIndex/* bank_index is specially assigned by us*/», «FOR param : member.parameters SEPARATOR ", "»«param.name»=«param.pythonInitializer»«ENDFOR»)«ENDFOR»]
                    ''')
                return
            } else if (instance.bankIndex === -1 && !instance.definition.reactorClass.toDefinition.allReactions.isEmpty) {
                pythonClassesInstantiation.append('''«instance.uniqueID»_lf = [_«className»(bank_index = 0«/* bank_index is specially assigned by us*/», «FOR param : instance.parameters SEPARATOR ", "»«param.name»=«param.pythonInitializer»«ENDFOR»)]
                ''')
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
       
       # The main function
       def main():
           start()
       
       # As is customary in Python programs, the main() function
       # should only be executed if the main module is active.
       if __name__=="__main__":
           main()
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
        if (!file.getParentFile().exists())
            file.getParentFile().mkdirs();
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
    def pythonCompileCode() {
        val compileCmd = createCommand('''python3''', #["setup.py", "build"], fileConfig.outPath)
        val installCmd = createCommand('''python3''',
            #["-m", "pip", "install", "--ignore-installed", "--force-reinstall", "--no-binary", ":all:", "--user", "."], fileConfig.outPath)

        compileCmd.directory(fileConfig.getSrcGenPath.toFile)
        installCmd.directory(fileConfig.getSrcGenPath.toFile)

        // Set compile time environment variables
        val compileEnv = compileCmd.environment
        compileEnv.put("CC", targetConfig.compiler) // Use gcc as the compiler
        compileEnv.put("LDFLAGS", targetConfig.linkerFlags) // The linker complains about including pythontarget.h twice (once in the generated code and once in pythontarget.c)
        // To avoid this, we force the linker to allow multiple definitions. Duplicate names would still be caught by the 
        // compiler.
        val installEnv = compileCmd.environment
        installEnv.put("CC", targetConfig.compiler) // Use gcc as the compiler
        installEnv.put("LDFLAGS", targetConfig.linkerFlags) // The linker complains about including pythontarget.h twice (once in the generated code and once in pythontarget.c)
        // To avoid this, we force the linker to allow multiple definitions. Duplicate names would still be caught by the 
        // compiler.
        if (executeCommand(installCmd) == 0) {
            println("Successfully installed python extension.")
        } else {
            reportError("Failed to install python extension.")
        }
    }
    
    /**
     * Generate code that ensures only one thread can execute at a time as per Python specifications
     * @param state 0=beginning, 1=end
     */
    def pyThreadMutexLockCode(int state, Reactor reactor) {
        if(targetConfig.threads > 0)
        {
            switch(state){
                case 0: return '''lf_mutex_lock(&py_«reactor.name»_reaction_mutex);'''
                case 1: return '''lf_mutex_unlock(&py_«reactor.name»_reaction_mutex);'''
                default: return ''''''
            }
        }
        else
        {
            return ''''''
        }
    }
    
    /**
     * Do nothing. The Python generator handles compiling differently.
     */
    override runCCompiler(String file, boolean doNotLinkIfNoMain) {
        // Note that this function is deliberately left empty to prevent the CGenerator from
        // compiling this code. The Python generator will create a setup.py and compile generated
        // C code appropriately.
        return true
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

        includeTargetLanguageHeaders()

        pr('#define NUMBER_OF_FEDERATES ' + federates.size);

        // Handle target parameters.
        // First, if there are federates, then ensure that threading is enabled.
        if (targetConfig.threads === 0 && isFederated) {
            targetConfig.threads = 1
        }

        super.includeTargetLanguageSourceFiles()

        super.parseTargetParameters()
        
        // If the program is threaded, create a mutex for each reactor
        // that guards the execution of its reactions.
        // This is necessary because Python is not thread-safe
        // and running multiple instances of the same function can cause
        // a segmentation fault.
        if (targetConfig.threads > 0) {
            for (r : this.reactors ?: emptyList) {
                pr('''
                    lf_mutex_t py_«r.toDefinition.name»_reaction_mutex;
                ''')
                pr(super.initializeTriggerObjects, '''
                    // Initialize reaction mutex for «r.toDefinition.name»
                    lf_mutex_init(&py_«r.toDefinition.name»_reaction_mutex);
                ''')
            }
            // Add mutex for the main reactor
            if (this.mainDef !== null) {
                pr('''
                    lf_mutex_t py_«this.mainDef.name»_reaction_mutex;
                ''')                
                pr(super.initializeTriggerObjects, '''
                    // Initialize reaction mutex for «this.mainDef.name»
                    lf_mutex_init(&py_«this.mainDef.name»_reaction_mutex);
                ''')
            }
        }
        // Handle .proto files.
        for (name : targetConfig.protoFiles) {
            this.processProtoFile(name)
            val dotIndex = name.lastIndexOf('.')
            var rootFilename = name
            if (dotIndex > 0) {
                rootFilename = name.substring(0, dotIndex)
            }
            pythonPreamble.append('''import «rootFilename»_pb2 as «rootFilename»
            ''')
        }
    }
    
    /**
     * Process a given .proto file.
     * 
     * Run, if possible, the proto-c protocol buffer code generator to produce
     * the required .h and .c files.
     * @param filename Name of the file to process.
     */
    override processProtoFile(String filename) {
         val protoc = createCommand("protoc", #['''--python_out=«this.fileConfig.getSrcGenPath»''', filename], fileConfig.srcPath)
         //val protoc = createCommand("protoc", #['''--python_out=src-gen/«topLevelName»''', topLevelName], codeGenConfig.outPath)
        if (protoc === null) {
            return
        }
        val returnCode = protoc.executeCommand()
        if (returnCode == 0) {
            pythonRequiredModules.append(''', 'google-api-python-client' ''')
        } else {
            reportError("protoc returns error code " + returnCode)
        }
    }
    
    /**
     * Generate the aliases for inputs, outputs, and struct type definitions for 
     * actions of the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     */
    override generateAuxiliaryStructs(
        ReactorDecl decl, FederateInstance federate
    ) {
        val reactor = decl.toDefinition
        // First, handle inputs.
        for (input : reactor.allInputs) {
            if (input.inferredType.isTokenType) {
                pr(input, code, '''
                    typedef «generic_port_type_with_token» «variableStructType(input, decl)»;
                ''')
            }
            else
            {
                pr(input, code, '''
                   typedef «generic_port_type» «variableStructType(input, decl)»;
                ''')                
            }
            
        }
        // Next, handle outputs.
        for (output : reactor.allOutputs) {
            if (output.inferredType.isTokenType) {
                 pr(output, code, '''
                    typedef «generic_port_type_with_token» «variableStructType(output, decl)»;
                 ''')
            }
            else
            {
                pr(output, code, '''
                    typedef «generic_port_type» «variableStructType(output, decl)»;
                ''')
            }
        }
        // Finally, handle actions.
        // The very first item on this struct needs to be
        // a trigger_t* because the struct will be cast to (trigger_t*)
        // by the schedule() functions to get to the trigger.
        for (action : reactor.allActions) {
            pr(action, code, '''
                typedef struct {
                    trigger_t* trigger;
                    «action.valueDeclaration»
                    bool is_present;
                    bool has_value;
                    lf_token_t* token;
                } «variableStructType(action, reactor)»;
            ''')
        }
    }
    
       /**
     * For the specified action, return a declaration for action struct to
     * contain the value of the action. An action of
     * type int[10], for example, will result in this:
     * ```
     *     int* value;
     * ```
     * This will return an empty string for an action with no type.
     * @param action The action.
     * @return A string providing the value field of the action struct.
     */
    override valueDeclaration(Action action) {
        if (action.type === null) {
            return ''
        }
        return "PyObject* value;"
    }
    
    /** Add necessary include files specific to the target language.
     *  Note. The core files always need to be (and will be) copied 
     *  uniformly across all target languages.
     */
    override includeTargetLanguageHeaders() {
        pr('''#define MODULE_NAME LinguaFranca«topLevelName»''')
        pr('''#define __GARBAGE_COLLECTED''')    	
        pr('#include "pythontarget.c"')
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

        super.doGenerate(resource, fsa, context)

        if (generatorErrorsOccurred) return;

        var baseFileName = topLevelName
        for (federate : federates) {
            if (isFederated) {
                topLevelName = baseFileName + '_' + federate.name
            }
            // Don't generate code if there is no main reactor
            if (this.main !== null) {
                generatePythonFiles(fsa, federate)
                // The C generator produces all the .c files in a single central folder.
                // However, we need to create a setup.py for each federate and run
                // "pip install ." individually to compile and install each module
                // Here, we move the necessary C files into each federate's folder
                if (!federate.isSingleton) {
//                    val srcDir = directory + File.separator + "src-gen" + File.separator + baseFileName
//                    val dstDir = directory + File.separator + "src-gen" + File.separator + filename
                    var filesToCopy = newArrayList('''«topLevelName».c''', "pythontarget.c", "pythontarget.h",
                        "ctarget.h", "core")
                    
                    copyFilesFromClassPath(fileConfig.srcPath.toString, fileConfig.getSrcGenPath.toString, filesToCopy);
                    
                    // Do not compile the Python code here. They will be compiled on remote machines
                }
                else {
                    if (targetConfig.noCompile !== true) {
                        // If there are no federates, compile and install the generated code
                        pythonCompileCode
                    }
                }
            }

        }
        // Restore filename
        topLevelName = baseFileName
    }
            
            
    
    /**
     * Copy Python specific target code to the src-gen directory
     */        
    override copyUserFiles() {
        super.copyUserFiles()
        // Copy the required target language files into the target file system.
        // This will also overwrite previous versions.
        var targetFiles = newArrayList("pythontarget.h", "pythontarget.c");
        for (file : targetFiles) {
            copyFileFromClassPath(
                "/" + "lib" + "/" + "Python" + "/" + file,
                fileConfig.getSrcGenPath.resolve(file).toString
            )
        }
        
        // Copy the C target header.
        // This will also overwrite previous versions.
        var cTargetFiles = newArrayList("ctarget.h");
        for (file : cTargetFiles) {
            copyFileFromClassPath(
                "/" + "lib" + "/" + "C" + "/" + file,
                fileConfig.getSrcGenPath.resolve(file).toString
            )
        }
    }
    
        
    /** FIXME: This function is copied from the CGenerator to enable federated
     *  execution. Ideally, the CGenerator.createLauncher() function should be refactored
     *  into a more flexible format that allows for various target source code extensions.
     * 
     *  Create the launcher shell scripts. This will create one or two file
     *  in the output path (bin directory). The first has name equal to
     *  the filename of the source file without the ".lf" extension.
     *  This will be a shell script that launches the
     *  RTI and the federates.  If, in addition, either the RTI or any
     *  federate is mapped to a particular machine (anything other than
     *  the default "localhost" or "0.0.0.0"), then this will generate
     *  a shell script in the bin directory with name filename_distribute.sh
     *  that copies the relevant source files to the remote host and compiles
     *  them so that they are ready to execute using the launcher.
     * 
     *  A precondition for this to work is that the user invoking this
     *  code generator can log into the remote host without supplying
     *  a password. Specifically, you have to have installed your
     *  public key (typically found in ~/.ssh/id_rsa.pub) in
     *  ~/.ssh/authorized_keys on the remote host. In addition, the
     *  remote host must be running an ssh service.
     *  On an Arch Linux system using systemd, for example, this means
     *  running:
     * 
     *      sudo systemctl <start|enable> ssh.service
     * 
     *  Enable means to always start the service at startup, whereas
     *  start means to just start it this once.
     *  On MacOS, open System Preferences from the Apple menu and 
     *  click on the "Sharing" preference panel. Select the checkbox
     *  next to "Remote Login" to enable it.
     * 
     *  @param coreFiles The files from the core directory that must be
     *   copied to the remote machines.
     */
    override createLauncher(ArrayList<String> coreFiles) {
        // NOTE: It might be good to use screen when invoking the RTI
        // or federates remotely so you can detach and the process keeps running.
        // However, I was unable to get it working properly.
        // What this means is that the shell that invokes the launcher
        // needs to remain live for the duration of the federation.
        // If that shell is killed, the federation will die.
        // Hence, it is reasonable to launch the federation on a
        // machine that participates in the federation, for example,
        // on the machine that runs the RTI.  The command I tried
        // to get screen to work looks like this:
        // ssh -t «target» cd «path»; screen -S «filename»_«federate.name» -L bin/«filename»_«federate.name» 2>&1
        var outPath = fileConfig.getSrcGenPath

        val shCode = new StringBuilder()
        val distCode = new StringBuilder()
        pr(shCode, '''
            #!/bin/bash
            # Launcher for federated «topLevelName».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
            # Set a trap to kill all background jobs on error.
            trap 'echo "#### Killing federates."; kill $(jobs -p)' ERR
            # Launch the federates:
        ''')
        val distHeader = '''
            #!/bin/bash
            # Distributor for federated «topLevelName».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
        '''
        val host = federationRTIProperties.get('host')
        var target = host

        var path = federationRTIProperties.get('dir')
        if(path === null) path = 'LinguaFrancaRemote'

        var user = federationRTIProperties.get('user')
        if (user !== null) {
            target = user + '@' + host
        }
        for (federate : federates) {
            if (federate.host !== null && federate.host != 'localhost' && federate.host != '0.0.0.0') {
                if(distCode.length === 0) pr(distCode, distHeader)
                pr(distCode, '''
                    echo "Making directory «path» and subdirectories src-gen and path on host «federate.host»"
                    ssh «federate.host» mkdir -p «path»/log «path»/src-gen/«topLevelName»/core
                    echo "Copying necessary files to host «federate.host»"
                    scp -r  src-gen/«topLevelName» «federate.host»:«path»/src-gen/
                    echo "Compiling on host «federate.host» using: pip install ."
                    ssh «federate.host» 'cd «path»/src-gen/«topLevelName»; pip install .'
                ''')
                pr(shCode, '''
                    echo "#### Launching the federate «federate.name» on host «federate.host»"
                    ssh «federate.host» '\
                        cd «path»; python3 src-gen/«topLevelName»/«topLevelName»_«federate.name».py >& log/«topLevelName»_«federate.name».out; \
                        echo "****** Output from federate «federate.name» on host «federate.host»:"; \
                        cat log/«topLevelName»_«federate.name».out; \
                        echo "****** End of output from federate «federate.name» on host «federate.host»"' &
                ''')                
            } else {
                pr(shCode, '''
                    echo "#### Launching the federate «federate.name»."
                    pushd «outPath» > /dev/
                    echo "Compiling and installing the LinguaFranca«topLevelName» module"
                    pip install .
                    popd > /dev/null
                    python3 «outPath»«File.separator»«topLevelName»_«federate.name».py &
                ''')                
            }
        }
        // Launch the RTI in the foreground.
        if (host == 'localhost' || host == '0.0.0.0') {
            pr(shCode, '''
                echo "#### Launching the runtime infrastructure (RTI)."
                «outPath»«File.separator»«topLevelName»_RTI
            ''')
        } else {
            // Copy the source code onto the remote machine and compile it there.
            if (distCode.length === 0) pr(distCode, distHeader)
            // The mkdir -p flag below creates intermediate directories if needed.
            pr(distCode, '''
                cd «path»
                echo "Making directory «path» and subdirectories src-gen and path on host «target»"
                ssh «target» mkdir -p «path»/log «path»/src-gen/«topLevelName»/core
                pushd src-gen/«topLevelName»/core > /dev/null
                echo "Copying LF core files to host «target»"
                scp rti.c rti.h util.h util.c reactor.h pqueue.h «target»:«path»/src-gen/«topLevelName»/core
                popd > /dev/null
                pushd src-gen/«topLevelName» > /dev/null
                echo "Copying source files to host «target»"
                scp «topLevelName»_RTI.c ctarget.h «target»:«path»/src-gen/«topLevelName»
                popd > /dev/null
                echo "Compiling on host «target» using: «targetConfig.compiler» -O2 «path»/src-gen/«topLevelName»/«topLevelName»_RTI.c -o «path»/bin/«topLevelName»_RTI -pthread"
                ssh «target» '«targetConfig.compiler» -O2 «path»/src-gen/«topLevelName»/«topLevelName»_RTI.c -o «path»/bin/«topLevelName»_RTI -pthread'
            ''')

            // Launch the RTI on the remote machine using ssh and screen.
            // The -t argument to ssh creates a virtual terminal, which is needed by screen.
            // The -S gives the session a name.
            // The -L option turns on logging. Unfortunately, the -Logfile giving the log file name
            // is not standardized in screen. Logs go to screenlog.0 (or screenlog.n).
            // FIXME: Remote errors are not reported back via ssh from screen.
            // How to get them back to the local machine?
            // Perhaps use -c and generate a screen command file to control the logfile name,
            // but screen apparently doesn't write anything to the log file!
            //
            // The cryptic 2>&1 reroutes stderr to stdout so that both are returned.
            // The sleep at the end prevents screen from exiting before outgoing messages from
            // the federate have had time to go out to the RTI through the socket.
            pr(shCode, '''
                echo "#### Launching the runtime infrastructure (RTI) on remote host «host»."
                ssh «target» 'cd «path»; \
                    «outPath»/«topLevelName»_RTI >& log/«topLevelName»_RTI.out; \
                    echo "------ output from «topLevelName»_RTI on host «target»:"; \
                    cat log/«topLevelName»_RTI.out; \
                    echo "------ end of output from «topLevelName»_RTI on host «target»"'
            ''')
        }

        // Write the launcher file.
        // Delete file previously produced, if any.
        var file = new File(outPath + File.separator + topLevelName)
        if (file.exists) {
            file.delete
        }
                
        var fOut = new FileOutputStream(file)
        fOut.write(shCode.toString().getBytes())
        fOut.close()
        if (!file.setExecutable(true, false)) {
            reportWarning(null, "Unable to make launcher script executable.")
        }
        
        // Write the distributor file.
        // Delete the file even if it does not get generated.
        file = new File(outPath + File.separator + topLevelName + '_distribute.sh')
        if (file.exists) {
            file.delete
        }
        if (distCode.length > 0) {
            fOut = new FileOutputStream(file)
            fOut.write(distCode.toString().getBytes())
            fOut.close()
            if (!file.setExecutable(true, false)) {
                reportWarning(null, "Unable to make distributor script executable.")
            }
        }
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
            // FIXME: Setting ref_counts of the token directly causes memory leak
            '''
            // Create a token
            lf_token_t* t = create_token(sizeof(PyObject*));
            t->value = self->__«ref»->value;
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
            self->__«outputName».value = («action.inferredType.targetType»)self->___«action.name».token->value;
            self->__«outputName».token = (lf_token_t*)self->___«action.name».token;
            ((lf_token_t*)self->___«action.name».token)->ref_count++;
            self->«getStackPortMember('''__«outputName»''', "is_present")» = true;
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
                        reportError(
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
        // Unfortunately, threads cannot run concurrently in Python.
        // Therefore, we need to make sure reactions cannot execute concurrently by
        // holding the mutex lock.
        if(targetConfig.threads > 0) {
            pr(pyThreadMutexLockCode(0, reactor))
        }
        
        pr('''PyObject *rValue = PyObject_CallObject(self->__py_reaction_function_«reactionIndex», Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»));
        ''')
        pr('''
            if (rValue == NULL) {
                fprintf(stderr, "Failed to call reaction «pythonFunctionName».\n");
            }
        ''')
        
        if(targetConfig.threads > 0) {
            pr(pyThreadMutexLockCode(1, reactor))
        }
        
        unindent()
        pr("}")
        
        // Now generate code for the deadline violation function, if there is one.
        if (reaction.deadline !== null) {
            // The following name has to match the choice in generateReactionInstances
            val deadlineFunctionName = decl.name.toLowerCase + '_deadline_function' + reactionIndex

            pr('void ' + deadlineFunctionName + '(void* instance_args) {')
            indent();
            
            
            super.generateInitializationForReaction("", reaction, decl, reactionIndex)
            // Unfortunately, threads cannot run concurrently in Python.
            // Therefore, we need to make sure reactions cannot execute concurrently by
            // holding the mutex lock.
            if (targetConfig.threads > 0) {
                pr(pyThreadMutexLockCode(0, reactor))
            }
            
            pr('''PyObject *rValue = PyObject_CallObject(self->__py_deadline_function_«reactionIndex», Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»));
            ''')
            pr('''
                if (rValue == NULL) {
                    fprintf(stderr, "Failed to call reaction «deadlineFunctionName».\n");
                }
            ''')

            if (targetConfig.threads > 0) {
                pr(pyThreadMutexLockCode(1, reactor))
            }
            //pr(reactionInitialization.toString)
            // Code verbatim from 'deadline'
            //prSourceLineNumber(reaction.deadline.code)
            //pr(reaction.deadline.code.toText)
            // TODO: Handle deadlines
            unindent()
            pr("}")
        }
    }
        
    
    /**
     * Generate code for parameters variables of a reactor in the form "parameter.type parameter.name;"
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
            // Check for targetBankIndex
            // FIXME: for now throw a reserved error
            if (parameter.name.equals(targetBankIndex)) {
                reportError('''«targetBankIndex» is reserved.''')
            }

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
     * Generate runtime initialization code for parameters of a given reactor instance
     * All parameters are initialized in Python code
     * 
     * FIXME: To allow for parameterized port widths, we assume that all parameters are int
     * in C and try to assign the value. We don't need to do this for list types as they
     * cannot be used to delineate port widths.
     * 
     * @param builder The StringBuilder used to append the initialization code to
     * @param instance The reactor instance
     * @return initialization code
     */
    override generateParameterInitialization(StringBuilder builder, ReactorInstance instance) {
       var nameOfSelfStruct = selfStructName(instance)
        for (parameter : instance.parameters) {            
            if (parameter.init.size > 1) {
                // Ignore the initialization in C for arrays
                // The actual initialization will be done in Python
            } else {
                pr(builder, '''
                    «nameOfSelfStruct»->«parameter.name» = «parameter.getInitializer»; 
                ''')
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
     * Generate code that is executed while the reactor instance is being initialized
     * @param initializationCode The StringBuilder appended to __initialize_trigger_objects()
     * @param instance The reactor instance
     * @param federate The federate instance
     */
    override generateReactorInstanceExtension(StringBuilder initializationCode, ReactorInstance instance, FederateInstance federate) {
        var nameOfSelfStruct = selfStructName(instance)
        var reactor = instance.definition.reactorClass.toDefinition
        
         // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.name.contains(GEN_DELAY_CLASS_NAME) || ((instance.definition.reactorClass === this.mainDef?.reactorClass) && reactor.isFederated))
        {
            return
        }
        
        // Initialize the name field to the unique name of the instance
        pr(initializationCode, '''«nameOfSelfStruct»->__lf_name = "«instance.uniqueID»_lf";
        ''');
        
        for (reaction : instance.reactions)
        {
            val pythonFunctionName = pythonReactionFunctionName(reaction.reactionIndex)
            // Create a PyObject for each reaction
            pr(initializationCode, '''«nameOfSelfStruct»->__py_reaction_function_«reaction.reactionIndex» = get_python_function("«topLevelName»", «nameOfSelfStruct»->__lf_name,«IF (instance.bankIndex > -1)» «instance.bankIndex» «ELSE» «0» «ENDIF»,"«pythonFunctionName»");''')
        
            if (reaction.definition.deadline !== null) {
                pr(initializationCode, '''«nameOfSelfStruct»->__py_deadline_function_«reaction.reactionIndex» = get_python_function("«topLevelName»", «nameOfSelfStruct»->__lf_name,«IF (instance.bankIndex > -1)» «instance.bankIndex» «ELSE» «0» «ENDIF»,"deadline_function_«reaction.reactionIndex»");''')
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
        pr(selfStructBody, '''char *__lf_name;
        ''');
        
        var reactionIndex = 0
        for (reaction : reactor.allReactions)
        {
            // Create a PyObject for each reaction
            pr(selfStructBody, '''PyObject *__py_reaction_function_«reactionIndex»;''')
            
            if (reaction.deadline !== null) {                
                pr(selfStructBody, '''PyObject *__py_deadline_function_«reactionIndex»;''')
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
                pyObjects.append(''', convert_C_port_to_py(&«output.name»,«output.name»_width) ''')
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
