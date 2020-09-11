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

package org.icyphy.generator

import java.io.File
import java.io.FileOutputStream
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext

import static extension org.icyphy.ASTUtils.*
import org.icyphy.linguaFranca.ReactorDecl
import org.icyphy.linguaFranca.Reaction
import java.util.HashMap
import org.icyphy.linguaFranca.Instantiation
import java.util.HashSet
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Type
import java.util.ArrayList
import org.icyphy.ASTUtils
import java.util.LinkedHashSet
import org.icyphy.linguaFranca.Value
import java.util.LinkedList
import java.util.List
import java.util.regex.Pattern

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
	
	// Set of acceptable import targets includes only C.
    val acceptableTargetSet = newLinkedHashSet('Python')
	
	new () {
        super()
        // set defaults
        this.targetCompiler = "python3"
        this.targetCompilerFlags = "-m pip install ."// -Wall -Wconversion"
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
    * @see xtext/org.icyphy.linguafranca/src/lib/CCpp/ccpptarget.h
    */
	val generic_port_type =  "generic_port_instance_struct*"

    /** 
    * Special template struct for ports with dynamically allocated
    * array types (a.k.a. token types) in Lingua Franca.
    * This template is defined as
    *     template <class T>
    *     struct template_input_output_port_struct {
    *         T value;
    *         bool is_present;
    *         int num_destinations;
    *         token_t* token;
    *         int length;
    *     };
    *
    * @see xtext/org.icyphy.linguafranca/src/lib/CCpp/ccpptarget.h
    */
	val generic_port_type_with_token = "generic_instance_with_token_struct*"
	
	
	// Regular expression pattern for pointer types. The star at the end has to be visible.
    static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");
    
   ////////////////////////////////////////////
    //// Public methods
    
    
    ////////////////////////////////////////////
    //// Protected methods
    
     /**
     * Override to convert true/false to True/False 
     * @param v A value
     * @return A value string in the target language
     */
    private def getPythonTargetValue(Value v) {
        if(v.toText == "false")
        {
            return "False"
        }
        else if(v.toText == "true")
        {
            return "True"
        }
        return super.getTargetValue(v)
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
     * Generate parameters for a reaction function
     * @param decl Reactor declaration
     * @param reaction The reaction to be used to generate parameters for
     */
    def generatePythonReactionParameters(ReactorDecl decl, Reaction reaction)
    {
        var StringBuilder parameters = new StringBuilder();
        val reactor = decl.toDefinition
        var generatedParams = new LinkedHashSet<String>()
                
        // Handle triggers
        for (TriggerRef trigger : reaction.triggers ?:emptyList)
        {
            if (trigger instanceof VarRef)
            {
                if (trigger.variable instanceof Port)
                {  
                    if(trigger.variable instanceof Input)
                    {
                        generatedParams.add(trigger.variable.name)
                        generatedParams.add('''«trigger.variable.name»_width''')
                    } else {
                        // FIXME: not using proper "."
                        generatedParams.add('''«trigger.container.name»_«trigger.variable.name»''')
                    }
                        
                }
                else if (trigger.variable instanceof Action)
                {
                    generatedParams.add(trigger.variable.name)
                }
            }
        }
        
        // Handle non-triggering inputs
        if (reaction.triggers === null || reaction.triggers.size === 0)
        {
             for (input : reactor.inputs ?:emptyList) {
                generatedParams.add(input.name)
                generatedParams.add('''«input.name»_width''')
            }
        }
        for (src : reaction.sources ?:emptyList)
        {
                generatedParams.add(src.variable.name)
        }
        
        // Handle effects
        for (effect : reaction.effects ?:emptyList)
        {
            if(effect.variable instanceof Input)
            {
                generatedParams.add('''«effect.container.name»_«effect.variable.name»''')
            }
            else{
                generatedParams.add(effect.variable.name)
                if (effect.variable instanceof Port)
                {
                    if(isMultiport(effect.variable as Port))
                    {
                        // Handle multiports           
                        generatedParams.add('''«effect.variable.name»_width''')
                    }
                }           
            }
        }
        
        // Handle parameters        
        for (param : reactor.allParameters)
        {
           generatedParams.add(param.name)
        }
        
        // Fill out the StrinBuilder parameters
        for (s : generatedParams)
        {
            parameters.append(''', «s»''')
        }
        
        return parameters
        
      }
    
    /**
     * Handle initialization for state variable
     * @param state a state variable
     */
    def String getTargetInitializer(StateVar state) {
        '''«FOR init : state.pythonInitializerList SEPARATOR ", "»«init»«ENDFOR»'''
    }
    
        
    /**
     * Generate a Python class corresponding to decl
     * @param decl The reactor's declaration
     */
    def generatePythonReactorClass(ReactorDecl decl) '''
        «var className = ""»
        «IF decl instanceof Reactor»
            «{className = decl.name; ""}»
        «ELSE»
            «{className = decl.toDefinition.name; ""}»    
        «ENDIF»
        
        class _«className»:
        
        «val reactor = decl.toDefinition»
        «FOR stateVar : reactor.allStateVars»
            «'    '»«stateVar.name»:«stateVar.targetType» = «stateVar.targetInitializer»
        «ENDFOR»
        
        «var reactionIndex = 0»
        «FOR reaction : reactor.allReactions»
                def «pythonReactionFunctionName(reactionIndex)»(self «generatePythonReactionParameters(reactor, reaction)»):
                    «reaction.pythonInitializaitons»
                    «reaction.code.toText»
                    return 0
            «{reactionIndex = reactionIndex+1; ""}»
        «ENDFOR»
    '''
    
    /**
     * Generate initialization code put at the beginning of the reaction before user code
     * @param reaction The reaction to generate initialization code for
     */
    def getPythonInitializaitons(Reaction reaction) {
        var StringBuilder inits = new StringBuilder();        
        // Handle triggers
        for (TriggerRef trigger : reaction.triggers ?:emptyList)
        {
            if (trigger instanceof VarRef)
            {
                if (trigger.variable instanceof Port)
                {  
                    if(trigger.variable instanceof Input)
                    {
                        
                    } else {
                        // Handle contained reactors' ports
                        inits.append('''«trigger.container.name» = Make''')
                        inits.append('''«trigger.container.name».«trigger.variable.name» = «trigger.container.name»_«trigger.variable.name»''')
                    }
                        
                }
                else if (trigger.variable instanceof Action)
                {
                    // TODO: handle actions
                }
            }
        }
        
        // Handle effects
        for (effect : reaction.effects ?:emptyList)
        {
            if(effect.variable instanceof Input)
            {
                inits.append('''«effect.container.name» = Make
                ''')
                inits.append('''«effect.container.name».«effect.variable.name» = «effect.container.name»_«effect.variable.name»''')  
            }
            else{
                // Do nothing          
            }
        }
        
        return inits
        
    }
    
    /**
     * Helper function for class instantiation in Python
     * @param instance The reactor instance to be instantiated
     * @param pythonClassesInstantiation The class instantiations are appended to this string builder
     * @param federate The federate instance for the reactor instance
     */
    def generatePythonClassInstantiation(ReactorInstance instance, StringBuilder pythonClassesInstantiation, FederateInstance federate)
    {
        var instantiatedClasses = new ArrayList<String>()
        generatePythonClassInstantiation(instance, pythonClassesInstantiation, federate, instantiatedClasses)
    }
    
    /**
     * Instantiate classes in Python.
     * Instances are always instantiated as a list of className = [_className, _className, ...] depending on the size of the bank.
     * If there is no bank or the size is 1, the instance would be generated as className = [_className]
     * @param instance The reactor instance to be instantiated
     * @param pythonClassesInstantiation The class instantiations are appended to this string builder
     * @param federate The federate instance for the reactor instance
     * @param instantiatedClasses A list of visited instances to avoid generating duplicates
     */
    def void generatePythonClassInstantiation(ReactorInstance instance, StringBuilder pythonClassesInstantiation,
        FederateInstance federate, ArrayList<String> instantiatedClasses) {
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

        // Invalid use of the function
        if (instantiatedClasses === null) {
            return
        }

        if (!instance.definition.reactorClass.toDefinition.allReactions.isEmpty) {
            if (reactorBelongsToFederate(instance, federate) && instance.bankMembers !== null &&
                !instantiatedClasses.contains(className)) {
                // If this reactor is a placeholder for a bank of reactors, then generate
                // a list of instances of reactors and return.         
                pythonClassesInstantiation.
                    append('''«className»s = [_«className»() for i in range(«instance.bankMembers.size»)]
                    ''')
                instantiatedClasses.add(className)
                return
            } else if (!instantiatedClasses.contains(className) &&
                !instance.definition.reactorClass.toDefinition.allReactions.isEmpty) {
                pythonClassesInstantiation.append('''«className»s = [_«className»()]
                ''')
                instantiatedClasses.add(className)
            }

        }

        for (child : instance.children) {
            generatePythonClassInstantiation(child, pythonClassesInstantiation, federate, instantiatedClasses)
        }
    }
    
    /**
     * Generate all Python classes if they have a reaction
     * @param federate The federate instance used to generate classes
     */
    def generatePythonReactorClasses(FederateInstance federate) {
        
        var StringBuilder pythonClasses = new StringBuilder()
        var StringBuilder pythonClassesInstantiation = new StringBuilder()
        
        // Generate the main reactor class if there are reactions
        if (!this.mainDef.reactorClass.toDefinition.allReactions.isEmpty) {
            pythonClasses.append("# The main reactor class")
            if (this.mainDef !== null) {
                pythonClasses.append(mainDef.reactorClass.generatePythonReactorClass)
            }
        }

        pythonClasses.append("\n")
        pythonClasses.append("# Generated Python classes")
        // Generate other reactor classes if they have reactions
        for (reactor : reactors)
        {
            // Generate the reactor classes in Python only if they have a reaction
            // Skip delay reactors
            if(!reactor.toDefinition.allReactions.isEmpty && !reactor.name.contains(GEN_DELAY_CLASS_NAME))
            {
                pythonClasses.append(reactor.generatePythonReactorClass())
            }
        }
        
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
       from LinguaFranca«filename» import *
       from LinguaFrancaBase.constants import * #Useful constants
       from LinguaFrancaBase.functions import * #Useful helper functions
       from LinguaFrancaBase.classes import * #Useful classes
       import sys
              
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
     */
    def generatePythonSetupFile() '''
    from setuptools import setup, Extension
    
    linguafranca«filename»module = Extension("LinguaFranca«filename»", ["«filename».c"])
    
    setup(name="LinguaFranca«filename»", version="1.0",
            ext_modules = [linguafranca«filename»module],
            install_requires=['LinguaFrancaBase'],)
    '''
    
    /**
     * Generate the necessary Python files
     * @param fsa The file system access (used to write the result).
     * 
     */
    def generatePythonFiles(IFileSystemAccess2 fsa, FederateInstance federate)
    {
        var srcGenPath = getSrcGenPath()
        
        var file = new File(srcGenPath + File.separator + filename + ".py")
        if (file.exists) {
            file.delete
        }
        // Create the necessary directories
        if (!file.getParentFile().exists())
            file.getParentFile().mkdirs();
        writeSourceCodeToFile(generatePythonCode(federate).toString.bytes, srcGenPath + File.separator + filename + ".py")
        
        // Handle Python setup
        file = new File(srcGenPath + File.separator + "setup.py")
        if (file.exists) {
            // Append
            file.delete
        }
            
        // Create the setup file
        writeSourceCodeToFile(generatePythonSetupFile.toString.bytes, srcGenPath + File.separator + "setup.py")        
        
    }
    
    /**
     * Execute the command that compiles and installs the current Python module
     */
    def pythonCompileCode()
    {
        val compileCmd = createCommand("python3", #["setup.py" , "build"])
        val installCmd = createCommand("python3", #["-m", "pip", "install", "."])
        
        compileCmd.directory(new File(getSrcGenPath))
        installCmd.directory(new File(getSrcGenPath))
        
        if(executeCommand(compileCmd) == 0) {
            println("Successfully compiled python extension.")
            if(executeCommand(installCmd) == 0)
            {
                println("Successfully installed python extension.")
            }
            else
            {
                reportError("Failed to install python extension.")
            }
        }        
        else
        {
            reportError("Failed to compile python extension.")
        }
    }
    
    /**
     * Generate code that ensures only one thread can execute at a time as per Python specifications
     * @param state 0=beginning, 1=end
     */
    def pyThreadMutexLockCode(int state) {
        if(targetThreads !== 0)
        {
            switch(state){
                case 0: return '''pthread_mutex_lock(&mutex);'''
                case 1: return '''pthread_mutex_unlock(&mutex);'''
                default: throw new Exception("Unknown state")
            }
        }
    }
    
    /**
     * Returns the desired source gen. path
     */
    override getSrcGenPath() {
          directory + File.separator + "src-gen" + File.separator + filename
    }
     
    /**
     * Returns the desired output path
     */
    override getBinGenPath() {
          directory + File.separator + "src-gen" + File.separator + filename
    }
    
    /**
     * Python always uses heap memory for ports 
     */
    override getStackPortMember(String portName, String member){
         portName.getHeapPortMember(member)
     }
     
     /**
     * Return the operator used to retrieve struct members
     */
    override getStackStructOperator() '''
    ->
    '''
    
    /**
     * Invoke pip on the generated code.
     */
    override compileCode() {
        // If there is more than one federate, compile each one.
        //var fileToCompile = "" // base file name.
        /*for (federate : federates) {
            // Empty string means no federates were defined, so we only
            // compile one file.
            if (!federate.isSingleton) {
                fileToCompile = filename + '_' + federate.name
            }*/
        //executeCommand(pythonCompileCommand, directory + File.separator + "src-gen")
        //}
        // Also compile the RTI files if there is more than one federate.
        /*if (federates.length > 1) {
            compileRTI()
        }*/
        // TODO: add support for compiling federates
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
                    typedef «generic_port_type_with_token» «variableStructType(input, reactor)»;
                ''')
            }
            else
            {
                pr(input, code, '''
                   typedef «generic_port_type» «variableStructType(input, reactor)»;
                ''')                
            }
            
        }
        // Next, handle outputs.
        for (output : reactor.allOutputs) {
            if (output.inferredType.isTokenType) {
                 pr(output, code, '''
                    typedef «generic_port_type_with_token» «variableStructType(output, reactor)»;
                 ''')
            }
            else
            {
                pr(output, code, '''
                    typedef «generic_port_type» «variableStructType(output, reactor)»;
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
                    token_t* token;
                } «variableStructType(action, reactor)»;
            ''')
        }
    }
    
    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set contains only "C".
     */
    override acceptableTargets() {
        acceptableTargetSet
    }
    
    /** Add necessary include files specific to the target language.
     *  Note. The core files always need to be (and will be) copied 
     *  uniformly across all target languages.
     */
    override includeTargetLanguageHeaders()
    {
        pr('''#define MODULE_NAME LinguaFranca«filename»''')    	
        pr('#include "pythontarget.h"')
    }
    
//    /** Add necessary source files specific to the target language.  */
//    override includeTargetLanguageSourceFiles()
//    {
//        if (targetThreads > 0) {
//            // Set this as the default in the generated code,
//            // but only if it has not been overridden on the command line.
//            pr(startTimers, '''
//                if (number_of_threads == 0) {
//                   number_of_threads = «targetThreads»;
//                }
//            ''')
//        }
//        if (federates.length > 1) {
//            pr("#include \"core/federate.c\"")
//        }
//    }

    /** Generate C code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
                // Always use the non-threaded version
            	super.doGenerate(resource, fsa, context)
                generatePythonFiles(fsa, null)
                pythonCompileCode
            }
            
    
    /**
     * Copy Python specific target code to the src-gen directory
     */        
    override copyTargetFiles()
    {    	
        var srcGenPath = getSrcGenPath()
    	// Copy the required target language files into the target file system.
        // This will also overwrite previous versions.
        var targetFiles = newArrayList("pythontarget.h");
        for (file : targetFiles) {
            copyFileFromClassPath(
                "/" + "lib" + "/" + "Python" + "/" + file,
                srcGenPath + File.separator + file
            )
        }
        
        // Copy the C target header.
        // This will also overwrite previous versions.
        var cTargetFiles = newArrayList("ctarget.h");
        for (file : cTargetFiles) {
            copyFileFromClassPath(
                "/" + "lib" + "/" + "C" + "/" + file,
                srcGenPath + File.separator + file
            )
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
        
        if(reactor.name.contains(GEN_DELAY_CLASS_NAME))
        {
            return super.generateReaction(reaction, decl, reactionIndex)
        }
        
        // Contains "O" characters. The number of these characters depend on the number of inputs to the reaction
        val StringBuilder pyObjectDescriptor = new StringBuilder()

        // Define the "self" struct.
        var structType = selfStructType(decl)
        
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
        
        // Indicates if the reactor is in a bank
        var isBank = false
               
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
        
        // Finally, handle parameters        
        for (param : reactor.allParameters )
        {
            // FIXME: array sizes cannot change at runtime
            generateParametersToSendToPythonReaction(pyObjectDescriptor, pyObjects, param, decl)
            if(param.name == "instance"
                && getTargetType(param.inferredType) == "int"
            )
            {
                // The reactor is in a bank
                isBank = true
            }
        }

        pr('void ' + functionName + '(void* instance_args) {')
        indent()

        pr(structType + "* self = (" + structType + "*)instance_args;")
        // Code verbatim from 'reaction'
        prSourceLineNumber(reaction.code)
        if(isBank) {
            // The reaction is in a reactor that belongs to a bank of reactors
            pr(pyThreadMutexLockCode(0))
            pr('''invoke_python_function("__main__", "«reactor.name»s", self->instance ,"«pythonFunctionName»", Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»));''')
            pr(pyThreadMutexLockCode(1))
        }
        else {
            pr('''invoke_python_function("__main__", "«reactor.name»s", 0 ,"«pythonFunctionName»", Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»));''')
        }
        unindent()
        pr("}")
        
        // Now generate code for the deadline violation function, if there is one.
        if (reaction.deadline !== null) {
            // The following name has to match the choice in generateReactionInstances
            val deadlineFunctionName = decl.name.toLowerCase + '_deadline_function' + reactionIndex

            pr('void ' + deadlineFunctionName + '(void* instance_args) {')
            indent();
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
            «DISABLE_REACTION_INITIALIZATION_MARKER»
            // Create a token
            token_t* t = create_token(sizeof(PyObject*));
            t->value = (*self->__«ref»)->value;
            t->length = 1; // Length is 1
            t->ref_count += (*self->__«ref»)->num_destinations;
            
            // Pass the token along
            schedule_token(&self->__«action.name», 0, t);
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
            self->__«outputName».token = (token_t*)self->___«action.name».token;
            ((token_t*)self->___«action.name».token)->ref_count++;
            self->«getStackPortMember('''__«outputName»''', "is_present")» = true;
            '''
        } else {
            '''
            «DISABLE_REACTION_INITIALIZATION_MARKER»
            SET(self->__«outputName», (self->___«action.name».token)->value);
            '''
        }
    }
    
    
    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    override generateConstructor(
        ReactorDecl decl, FederateInstance federate, StringBuilder constructorCode
    ) {
        val structType = selfStructType(decl)
        val StringBuilder portsAndTriggers = new StringBuilder()
        
        val reactor = decl.toDefinition
  

                
        // Initialize actions in Python
        for (action : reactor.allActions) {
            // TODO
        }
        
        // Next handle inputs.
        for (input : reactor.allInputs) {
           if (input.isMultiport) {
               // TODO
           }
           else
           {
           }
        }
        
        // Next handle outputs.
        for (output : reactor.allOutputs) {
            if (output.isMultiport) {

            } else {
                pr(output, portsAndTriggers, '''
                    self->__«output.name» =  PyObject_GC_New(generic_port_instance_struct, &port_instance_t);
                ''')
            }
        }
        
        // Handle outputs of contained reactors
        for (reaction : reactor.allReactions)
        {
            for (effect : reaction.effects ?:emptyList)
            {
                if(effect.variable instanceof Input)
                {
                    pr(effect.variable , portsAndTriggers, '''
                        self->__«effect.container.name».«effect.variable.name» =  PyObject_GC_New(generic_port_instance_struct, &port_instance_t);
                    ''')
                }
                else {
                    // Do nothing
                }
            }
        }
        
        pr('''
            «structType»* new_«reactor.name»() {
                «structType»* self = («structType»*)calloc(1, sizeof(«structType»));
                «constructorCode.toString»
                «portsAndTriggers.toString»
                return self;
            }''')

    }
    
    
    /**
     * A function used to generate initalization code for an output multiport
     * @param builder The generated code is put into builder
     * @param output The output port to be initialized
     * @name
     */
    override initializeOutputMultiport(StringBuilder builder, Output output, String nameOfSelfStruct, ReactorInstance instance) {
        val reactor = instance.definition.reactorClass
        pr(builder, '''
            «nameOfSelfStruct»->__«output.name»__width = «multiportWidthSpecInC(output, null, instance)»;
            // Allocate memory for multiport output.
            «nameOfSelfStruct»->__«output.name» = («variableStructType(output, reactor)»*)malloc(sizeof(PyObject *) * «nameOfSelfStruct»->__«output.name»__width);
            
            for ( int __i=0 ; __i<«nameOfSelfStruct»->__«output.name»__width ; __i++) {
                «nameOfSelfStruct»->__«output.name»[__i] = PyObject_GC_New(generic_port_instance_struct, &port_instance_t);
            }
        ''')
    }
    
    
    /**
     * Generate instantiation and initialization code for an output multiport of a reaction.
     * The instantiations and the initializations are put into two separate StringBuilders in case delayed initialization is desirable
     * @param instantiation The StringBuilder used to put code that allocates overall memory for a multiport
     * @param initialization The StringBuilderused to put code that initializes members of a multiport
     * @param effect The output effect of a given reaction
     * @param instance The reaction instance itself
     * @param reactionIdx The index of the reaction in the Reactor
     * @param startIdx The index used to figure out the starting position of the output_produced array
     */
    override initializeReactionEffectMultiport(StringBuilder instantiation, StringBuilder initialization, VarRef effect, ReactorInstance instance, int reationIdx, String startIdx)
    {
        val port = effect.variable as Port
        val reactorClass = instance.definition.reactorClass
        val nameOfSelfStruct = selfStructName(instance)
        // If the width is given as a numeric constant, then add that constant
        // to the output count. Otherwise, assume it is a reference to one or more parameters.
        val widthSpec = multiportWidthSpecInC(port, effect.container, instance)
        // Allocate memory where the data will produced by the reaction will be stored
        // and made available to the input of the contained reactor.
        // This is done differently for ports like "c.in" than "out".
        // This has to go at the end of the initialize_trigger_objects() function
        // because the self struct of contained reactors has not yet been defined.
        // FIXME: The following mallocs are not freed by the destructor!
        if (effect.container === null) {
            // This has form "out".
            val portStructType = variableStructType(port, reactorClass)
            pr(instantiation, '''
                «nameOfSelfStruct»->__«port.name»__width = «widthSpec»;
                // Allocate memory for to store output of reaction feeding a multiport input of a contained reactor.
                «nameOfSelfStruct»->__«port.name» = («portStructType»*)malloc(sizeof(PyObject *) * «nameOfSelfStruct»->__«port.name»__width);
                                    
                for ( int __i=0 ; __i<«nameOfSelfStruct»->__«port.name»__width ; __i++) {
                    «nameOfSelfStruct»->__«port.name»[__i] = PyObject_GC_New(generic_port_instance_struct, &port_instance_t);
                }
            ''')
            pr(initialization, '''
                for (int i = 0; i < «widthSpec»; i++) {
                    «nameOfSelfStruct»->___reaction_«reationIdx».output_produced[«startIdx» + i]
                            = &«nameOfSelfStruct»->«getStackPortMember('''__«ASTUtils.toText(effect)»[i]''', "is_present")»;
                }
            ''')
        } else {
            // This has form "c.in".
            val containerName = effect.container.name
            val portStructType = variableStructType(port, effect.container.reactorClass)
            pr(instantiation, '''
                «nameOfSelfStruct»->__«containerName».«port.name»__width = «widthSpec»;
                // Allocate memory for to store output of reaction feeding a multiport input of a contained reactor.
                «nameOfSelfStruct»->__«containerName».«port.name» = («portStructType»**)malloc(sizeof(PyObject *) 
                        * «nameOfSelfStruct»->__«containerName».«port.name»__width);
                for (int i = 0; i < «nameOfSelfStruct»->__«containerName».«port.name»__width; i++) {
                    «nameOfSelfStruct»->__«containerName».«port.name»[i] = PyObject_GC_New(generic_port_instance_struct, &port_instance_t);
                }
            ''')
            pr(initialization, '''
                for (int i = 0; i < «widthSpec»; i++) {
                    «nameOfSelfStruct»->___reaction_«reationIdx».output_produced[«startIdx» + i]
                            = &«nameOfSelfStruct»->__«ASTUtils.toText(effect)»[i]->is_present;
                }
            ''')
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
        if(action.type === null)
        {
            pyObjects.append(''', convert_C_action_to_py((void *)&self->__«action.name», PyLong_FromLong(0), self->__«action.name».is_present)''')            
        }
        else
        {
            pyObjects.append(''', convert_C_action_to_py((void *)&self->__«action.name», Py_BuildValue("«action.inferredType.targetType.pyBuildValueArgumentType»", self->__«action.name».value), self->__«action.name».is_present)''')
        }
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
            pyObjectDescriptor.append("O")
            pyObjects.append(''', (PyObject *)self->__«port.container.name».«port.variable.name»''')
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
         if (output.type === null) {
            reportError(output,
                "Output is required to have a type: " + output.name)
        } else {
            val outputStructType = variableStructType(output, decl)
            // Unfortunately, for the SET macros to work out-of-the-box for
            // multiports, we need an array of *pointers* to the output structs,
            // but what we have on the self struct is an array of output structs.
            // So we have to handle multiports specially here a construct that
            // array of pointers.
            if (!output.isMultiport) {
                pyObjectDescriptor.append("O")
                pyObjects.append(''', (PyObject *)self->__«output.name»''')
            } else {
                // Set the _width variable.                
                pyObjectDescriptor.append("O")
                pyObjects.append(''', make_output_port_list((generic_port_instance_struct **)self->__«output.name»,self->__«output.name»__width) ''')
                
                
                pyObjectDescriptor.append("i")
                pyObjects.append(''', self->__«output.name»__width''')
            }
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
        // TODO: handle multiports
        pyObjectDescriptor.append("O")
        pyObjects.append(''', (PyObject *)self->__«definition.name».«input.name»''')
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
            pyObjects.append(''', (PyObject *)*self->__«input.name»''')
        } else if (input.isMutable && !input.isMultiport) {
            // Mutable, non-multiport, primitive type.
            // TODO: handle mutable
            pyObjectDescriptor.append("O")
            pyObjects.append(''', (PyObject *)*self->__«input.name»''')
        } else if (!input.isMutable && input.isMultiport) {
            // Non-mutable, multiport, primitive.
            // TODO: support multiports
            pyObjectDescriptor.append("O")            
            pyObjects.append(''', make_input_port_tuple((generic_port_instance_struct ***)self->__«input.name»,self->__«input.name»__width) ''')
        } else {
            // Mutable, multiport, primitive type
            // TODO: support mutable multiports
            pyObjectDescriptor.append("O")            
            pyObjects.append(''', make_input_port_list((generic_port_instance_struct ***)self->__«input.name»,self->__«input.name»__width) ''')
        }
        // Set the _width variable for all cases. This will be -1
        // for a variable-width multiport, which is not currently supported.
        // It will be -2 if it is not multiport.
        pyObjectDescriptor.append("i")
        pyObjects.append(''', self->__«input.name»__width''')
    }
    
    private def generateParametersToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Parameter param,
        ReactorDecl decl
    )
    {
        val targetType = param.inferredType.targetType
        // If the parameter is an array, we need to convert it to a Python tuple
        // FIXME: Size cannot expand
        val matcher = pointerPatternVariable.matcher(targetType)
        if(matcher.find()){       
            pyObjectDescriptor.append("O")
            pyObjects.append(''', \
                    «'      '»«param.pyConvertParameterArrayToList(matcher.group(1))»''')
        }
        else
        {
            pyObjectDescriptor.append(param.targetType.pyBuildValueArgumentType)
            pyObjects.append(''', self->«param.name»''')
        }
    }
    
    /** 
     * Convert a parameter array to a Python tuple
     */
    private def pyConvertParameterArrayToList(Parameter param, String type) {
        var StringBuilder pyObjectDescriptor = new StringBuilder()
        var StringBuilder pyObjects = new StringBuilder()
        
        for(var i = 0; i < param.init.size ; i++)
        {
            pyObjectDescriptor.append(type.pyBuildValueArgumentType)
            pyObjects.append(''', self->«param.name»[«i»]''')
        }
        
        
        return '''Py_BuildValue("(«pyObjectDescriptor»)" «pyObjects»)'''
        
        
    }
    
    private def pyBuildValueArgumentType(String type)
    {
        switch(type)
        {
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
