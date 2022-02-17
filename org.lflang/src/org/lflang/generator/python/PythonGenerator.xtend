/* Generator for the Python target. */

/*************
 * Copyright (c) 2022, The University of California at Berkeley.
 * Copyright (c) 2022, The University of Texas at Dallas.

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
import java.util.List
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.JavaAstUtils
import org.lflang.Target
import org.lflang.TargetConfig.Mode
import org.lflang.TargetProperty.CoordinationType
import org.lflang.federated.FedFileConfig
import org.lflang.federated.FederateInstance
import org.lflang.federated.launcher.FedPyLauncher
import org.lflang.federated.serialization.FedNativePythonSerialization
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.generator.CodeBuilder
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorResult
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.JavaGeneratorUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance
import org.lflang.generator.SubContext
import org.lflang.generator.c.CGenerator
import org.lflang.generator.c.CUtil
import org.lflang.generator.python.PythonDockerGenerator
import org.lflang.generator.python.PyUtil
import org.lflang.generator.python.PythonReactionGenerator;
import org.lflang.generator.python.PythonReactorGenerator;
import org.lflang.generator.python.PythonParameterGenerator;
import org.lflang.generator.python.PythonNetworkGenerator;
import org.lflang.lf.Action
import org.lflang.lf.Delay
import org.lflang.lf.Model
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.ReactorDecl
import org.lflang.lf.StateVar
import org.lflang.lf.VarRef
import static org.lflang.generator.python.PythonInfoGenerator.*
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

    override getTargetTypes() {
        return types;
    }

    // //////////////////////////////////////////
    // // Protected methods
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
                list.add(PyUtil.getPythonTargetValue(i))
            }
        }
        return list
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
     * Generate all Python classes if they have a reaction
     * @param federate The federate instance used to generate classes
     */
    def generatePythonReactorClasses(FederateInstance federate) {

        var CodeBuilder pythonClasses = new CodeBuilder()
        var CodeBuilder pythonClassesInstantiation = new CodeBuilder()

        // Generate reactor classes in Python
        pythonClasses.pr(PythonReactorGenerator.generatePythonClass(main, federate, main, types))

        // Create empty lists to hold reactor instances
        pythonClassesInstantiation.pr(PythonReactorGenerator.generateListsToHoldClassInstances(main, federate))

        // Instantiate generated classes
        pythonClassesInstantiation.pr(PythonReactorGenerator.generatePythonClassInstantiations(main, federate, main))

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
     * Generate the necessary Python files.
     * @param federate The federate instance
     */
    def generatePythonFiles(FederateInstance federate) {
        var file = new File(fileConfig.getSrcGenPath.toFile, topLevelName + ".py")
        if (file.exists) {
            file.delete
        }
        // Create the necessary directories
        if (!file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }
        val codeMaps = #{file.toPath -> CodeMap.fromGeneratedCode(generatePythonCode(federate).toString)}
        JavaGeneratorUtils.writeToFile(codeMaps.get(file.toPath).generatedCode, file.toPath)
        
        val setupPath = fileConfig.getSrcGenPath.resolve("setup.py")
        // Handle Python setup
        System.out.println("Generating setup file to " + setupPath)
        file = setupPath.toFile
        if (file.exists) {
            // Append
            file.delete
        }

        // Create the setup file
        JavaGeneratorUtils.writeToFile(generatePythonSetupFile, setupPath)
             
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

        code.pr(CGenerator.defineLogLevel(this))
        
        if (isFederated) {
            // FIXME: Instead of checking
            // #ifdef FEDERATED, we could
            // use #if (NUMBER_OF_FEDERATES > 1)
            // To me, the former is more accurate.
            code.pr('''
                #define FEDERATED
            ''')
            if (targetConfig.coordination === CoordinationType.CENTRALIZED) {
                // The coordination is centralized.
                code.pr('''
                    #define FEDERATED_CENTRALIZED
                ''')
            } else if (targetConfig.coordination === CoordinationType.DECENTRALIZED) {
                // The coordination is decentralized
                code.pr('''
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

        code.pr("#include \"core/mixed_radix.h\"");

        code.pr('#define NUMBER_OF_FEDERATES ' + federates.size);

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
                    code.pr(pickler.generatePreambleForSupport.toString);
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
        return PythonNetworkGenerator.generateNetworkReceiverBody(
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
            serializer
        );
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
        return PythonNetworkGenerator.generateNetworkSenderBody(
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
            targetConfig.coordination
        );
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
                if (CUtil.isTokenType(input.inferredType, types)) {
                    code.pr(input, '''
                        typedef «generic_port_type_with_token» «variableStructType(input, decl)»;
                    ''')
                } else {
                    code.pr(input, '''
                        typedef «generic_port_type» «variableStructType(input, decl)»;
                    ''')
                }

            }

        }
        // Next, handle outputs.
        for (output : reactor.allOutputs) {
            if (federate === null || federate.contains(output as Port)) {
                if (CUtil.isTokenType(output.inferredType, types)) {
                    code.pr(output, '''
                        typedef «generic_port_type_with_token» «variableStructType(output, decl)»;
                    ''')
                } else {
                    code.pr(output, '''
                        typedef «generic_port_type» «variableStructType(output, decl)»;
                    ''')
                }

            }
        }
        // Finally, handle actions.
        for (action : reactor.allActions) {
            if (federate === null || federate.contains(action)) {
                code.pr(action, '''
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
        code.pr('''#define _LF_GARBAGE_COLLECTED''') 
        if (targetConfig.tracing !== null) {
            var filename = "";
            if (targetConfig.tracing.traceFileName !== null) {
                filename = targetConfig.tracing.traceFileName;
            }
            code.pr('#define LINGUA_FRANCA_TRACE ' + filename)
        }
                       
        code.pr('#include "pythontarget.c"')
        if (targetConfig.tracing !== null) {
            code.pr('#include "core/trace.c"')            
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
     *  @param context Context relating to invocation of the code generator.
     */
    override void doGenerate(Resource resource, LFGeneratorContext context) {
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
        super.doGenerate(resource, new SubContext(
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
                val codeMapsForFederate = generatePythonFiles(federate)
                codeMaps.putAll(codeMapsForFederate)
                PyUtil.copyTargetFiles(fileConfig);
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
                    println(generateSetupInfo(fileConfig))
                }

                if (!isFederated) {
                    println(generateRunInfo(fileConfig, topLevelName))
                }
            }
            fileConfig = oldFileConfig;
        }
        if (isFederated) {
            println(generateFedRunInfo(fileConfig))
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
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    override generateDelayBody(Action action, VarRef port) {
        return PythonReactionGenerator.generateCDelayBody(action, port, CUtil.isTokenType(action.inferredType, types))
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
        if (CUtil.isTokenType(action.inferredType, types)) {
            super.generateForwardBody(action, port)
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
        var reactor = decl.toDefinition;

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.getName().contains(GEN_DELAY_CLASS_NAME) ||
            ((mainDef !== null && decl == mainDef.getReactorClass() || mainDef == decl) && reactor.isFederated())) {
            super.generateReaction(reaction, decl, reactionIndex);
            return;
        }
        code.pr(PythonReactionGenerator.generateInitializers(reaction, decl, reactionIndex, mainDef, errorReporter, types, isFederatedAndDecentralized));
    }

    /**
     * Generate code for parameter variables of a reactor in the form "parameter.type parameter.name;"
     * 
     * FIXME: for now we assume all parameters are int. This is to circumvent the issue of parameterized
     * port widths for now.
     * 
     * @param reactor The reactor.
     * @param builder The place that the generated code is written to.
     * @return 
     */
    override generateParametersForReactor(CodeBuilder builder, Reactor reactor) {
        PythonParameterGenerator.generateCDeclarations(builder, reactor);
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
        initializeTriggerObjects.pr(PythonParameterGenerator.generateCInitializers(instance));
    }

    /**
     * This function is overridden in the Python generator to do nothing.
     * The state variables are initialized in Python code directly.
     * @param reactor The reactor.
     * @param builder The place that the generated code is written to.
     * @return 
     */
    override generateStateVariablesForReactor(CodeBuilder builder, Reactor reactor) {        
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
        initializeTriggerObjects.pr(PythonReactionGenerator.generateCPythonReactionLinkers(instance, reactions, mainDef, topLevelName));
    }

    /**
     * This function is provided to allow extensions of the CGenerator to append the structure of the self struct
     * @param selfStructBody The body of the self struct
     * @param decl The reactor declaration for the self struct
     * @param instance The current federate instance
     * @param constructorCode Code that is executed when the reactor is instantiated
     */
    override generateSelfStructExtension(
        CodeBuilder selfStructBody, 
        ReactorDecl decl, 
        FederateInstance instance, 
        CodeBuilder constructorCode
    ) {
        val reactor = decl.toDefinition
        // Add the name field
        selfStructBody.pr('''char *_lf_name;
        ''');

        var reactionIndex = 0
        for (reaction : reactor.allReactions) {
            // Create a PyObject for each reaction
            selfStructBody.pr('''PyObject* _lf_py_reaction_function_«reactionIndex»;''')
            
            if (reaction.deadline !== null) {                
                selfStructBody.pr('''PyObject* _lf_py_deadline_function_«reactionIndex»;''')
            }
            reactionIndex++
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

        val contents = new CodeBuilder()
        contents.pr(PythonDockerGenerator.generateDockerFileContent(topLevelName, srcGenPath))
        contents.writeToFile(dockerFile)
        println(getDockerBuildCommand(dockerFile, dockerComposeDir, federateName))
    }
}
