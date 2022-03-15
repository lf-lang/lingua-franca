/* Generator for C target. */

/*************
Copyright (c) 2019-2021, The University of California at Berkeley.

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

package org.lflang.generator.c;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.regex.Pattern;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.FedFileConfig;
import org.lflang.federated.FederateInstance;
import org.lflang.federated.launcher.FedCLauncher;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.DockerComposeGenerator;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.SubContext;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.c.CActionGenerator;
import org.lflang.generator.c.CTimerGenerator;
import org.lflang.generator.c.CStateGenerator;
import org.lflang.generator.c.CTracingGenerator;
import org.lflang.generator.c.CPortGenerator;
import org.lflang.generator.c.CModesGenerator;
import org.lflang.generator.c.CMainGenerator;
import org.lflang.generator.c.CFederateGenerator;
import org.lflang.generator.c.CNetworkGenerator;
import org.lflang.generator.c.CTriggerObjectsGenerator;
import org.lflang.generator.c.InteractingContainedReactors;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Delay;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.util.FileUtil;
import static extension org.lflang.ASTUtils.*;
import static extension org.lflang.ASTUtils.*;

/** 
 * Generator for C target. This class generates C code defining each reactor
 * class given in the input .lf file and imported .lf files. The generated code
 * has the following components:
 * 
 * * A typedef for inputs, outputs, and actions of each reactor class. These
 *   define the types of the variables that reactions use to access inputs and
 *   action values and to set output values.
 * 
 * * A typedef for a "self" struct for each reactor class. One instance of this
 *   struct will be created for each reactor instance. See below for details.
 * 
 * * A function definition for each reaction in each reactor class. These
 *   functions take an instance of the self struct as an argument.
 * 
 * * A constructor function for each reactor class. This is used to create
 *   a new instance of the reactor.
 * 
 * After these, the main generated function is `_lf_initialize_trigger_objects()`.
 * This function creates the instances of reactors (using their constructors)
 * and makes connections between them.
 * 
 * A few other smaller functions are also generated.
 * 
 * ## Self Struct
 * 
 * The "self" struct has fields for each of the following:
 * 
 * * parameter: the field name and type match the parameter.
 * * state: the field name and type match the state.
 * * action: the field name prepends the action name with "_lf_".
 *   A second field for the action is also created to house the trigger_t object.
 *   That second field prepends the action name with "_lf__".
 * * output: the field name prepends the output name with "_lf_".
 * * input:  the field name prepends the output name with "_lf_".
 *   A second field for the input is also created to house the trigger_t object.
 *   That second field prepends the input name with "_lf__".
 *
 * If, in addition, the reactor contains other reactors and reacts to their outputs,
 * then there will be a struct within the self struct for each such contained reactor.
 * The name of that self struct will be the name of the contained reactor prepended with "_lf_".
 * That inside struct will contain pointers the outputs of the contained reactors
 * that are read together with pointers to booleans indicating whether those outputs are present.
 * 
 * If, in addition, the reactor has a reaction to shutdown, then there will be a pointer to
 * trigger_t object (see reactor.h) for the shutdown event and an action struct named
 * _lf_shutdown on the self struct.
 * 
 * ## Reaction Functions
 * 
 * For each reaction in a reactor class, this generator will produce a C function
 * that expects a pointer to an instance of the "self" struct as an argument.
 * This function will contain verbatim the C code specified in the reaction, but
 * before that C code, the generator inserts a few lines of code that extract from the
 * self struct the variables that that code has declared it will use. For example, if
 * the reaction declares that it is triggered by or uses an input named "x" of type
 * int, the function will contain a line like this:
 * ```
 *     r_x_t* x = self->_lf_x;
 * ```
 * where `r` is the full name of the reactor class and the struct type `r_x_t`
 * will be defined like this:
 * ```
 *     typedef struct {
 *         int value;
 *         bool is_present;
 *         int num_destinations;
 *     } r_x_t;
 * ```
 * The above assumes the type of `x` is `int`.
 * If the programmer fails to declare that it uses x, then the absence of the
 * above code will trigger a compile error when the verbatim code attempts to read `x`.
 *
 * ## Constructor
 * 
 * For each reactor class, this generator will create a constructor function named
 * `new_r`, where `r` is the reactor class name. This function will malloc and return
 * a pointer to an instance of the "self" struct.  This struct initially represents
 * an unconnected reactor. To establish connections between reactors, additional
 * information needs to be inserted (see below). The self struct is made visible
 * to the body of a reaction as a variable named "self".  The self struct contains the
 * following:
 * 
 * * Parameters: For each parameter `p` of the reactor, there will be a field `p`
 *   with the type and value of the parameter. So C code in the body of a reaction
 *   can access parameter values as `self->p`.
 * 
 * * State variables: For each state variable `s` of the reactor, there will be a field `s`
 *   with the type and value of the state variable. So C code in the body of a reaction
 *   can access state variables as as `self->s`.
 * 
 * The self struct also contains various fields that the user is not intended to
 * use. The names of these fields begin with at least two underscores. They are:
 * 
 * * Outputs: For each output named `out`, there will be a field `_lf_out` that is
 *   a struct containing a value field whose type matches that of the output.
 *   The output value is stored here. That struct also has a field `is_present`
 *   that is a boolean indicating whether the output has been set.
 *   This field is reset to false at the start of every time
 *   step. There is also a field `num_destinations` whose value matches the
 *   number of downstream reactors that use this variable. This field must be
 *   set when connections are made or changed. It is used to initialize
 *   reference counts for dynamically allocated message payloads.
 *   The reference count is decremented in each destination reactor at the
 *   conclusion of each time step, and when it drops to zero, the memory
 *   is freed.
 * 
 * * Inputs: For each input named `in` of type T, there is a field named `_lf_in`
 *   that is a pointer struct with a value field of type T. The struct pointed
 *   to also has an `is_present` field of type bool that indicates whether the
 *   input is present.
 * 
 * * Outputs of contained reactors: If a reactor reacts to outputs of a
 *   contained reactor `r`, then the self struct will contain a nested struct
 *   named `_lf_r` that has fields pointing to those outputs. For example,
 *   if `r` has an output `out` of type T, then there will be field in `_lf_r`
 *   named `out` that points to a struct containing a value field
 *   of type T and a field named `is_present` of type bool.
 * 
 * * Inputs of contained reactors: If a reactor sends to inputs of a
 *   contained reactor `r`, then the self struct will contain a nested struct
 *   named `_lf_r` that has fields for storing the values provided to those
 *   inputs. For example, if R has an input `in` of type T, then there will
 *   be field in _lf_R named `in` that is a struct with a value field
 *   of type T and a field named `is_present` of type bool.
 * 
 * * Actions: If the reactor has an action a (logical or physical), then there
 *   will be a field in the self struct named `_lf_a` and another named `_lf__a`.
 *   The type of the first is specific to the action and contains a `value`
 *   field with the type and value of the action (if it has a value). That
 *   struct also has a `has_value` field, an `is_present` field, and a
 *   `token` field (which is NULL if the action carries no value).
 *   The `_lf__a` field is of type trigger_t.
 *   That struct contains various things, including an array of reactions
 *   sensitive to this trigger and a lf_token_t struct containing the value of
 *   the action, if it has a value.  See reactor.h in the C library for
 *   details.
 * 
 * * Reactions: Each reaction will have several fields in the self struct.
 *   Each of these has a name that begins with `_lf__reaction_i`, where i is
 *   the number of the reaction, starting with 0. The fields are:
 *   * _lf__reaction_i: The struct that is put onto the reaction queue to
 *     execute the reaction (see reactor.h in the C library).
 * 
 *  * Timers: For each timer t, there is are two fields in the self struct:
 *    * _lf__t: The trigger_t struct for this timer (see reactor.h).
 *    * _lf__t_reactions: An array of reactions (pointers to the
 *      reaction_t structs on this self struct) sensitive to this timer.
 *
 * * Triggers: For each Timer, Action, Input, and Output of a contained
 *   reactor that triggers reactions, there will be a trigger_t struct
 *   on the self struct with name `_lf__t`, where t is the name of the trigger.
 * 
 * ## Connections Between Reactors
 * 
 * Establishing connections between reactors involves two steps.
 * First, each destination (e.g. an input port) must have pointers to
 * the source (the output port). As explained above, for an input named
 * `in`, the field `_lf_in->value` is a pointer to the output data being read.
 * In addition, `_lf_in->is_present` is a pointer to the corresponding
 * `out->is_present` field of the output reactor's self struct.
 *  
 * In addition, the `reaction_i` struct on the self struct has a `triggers`
 * field that records all the trigger_t structs for ports and actions
 * that are triggered by the i-th reaction. The triggers field is
 * an array of arrays of pointers to trigger_t structs.
 * The length of the outer array is the number of output channels
 * (single ports plus multiport widths) that the reaction effects
 * plus the number of input port channels of contained
 * reactors that it effects. Each inner array has a length equal to the
 * number of final destinations of that output channel or input channel.
 * The reaction_i struct has an array triggered_sizes that indicates
 * the sizes of these inner arrays. The num_outputs field of the
 * reaction_i struct gives the length of the triggered_sizes and
 * (outer) triggers arrays. The num_outputs field is equal to the
 * total number of single ports and multiport channels that the reaction
 * writes to.
 * 
 * ## Runtime Tables
 * 
 * This generator creates an populates the following tables used at run time.
 * These tables may have to be resized and adjusted when mutations occur.
 * 
 * * _lf_is_present_fields: An array of pointers to booleans indicating whether an
 *   event is present. The _lf_start_time_step() function in reactor_common.c uses
 *   this to mark every event absent at the start of a time step. The size of this
 *   table is contained in the variable _lf_is_present_fields_size.
 *    * This table is accompanied by another list, _lf_is_present_fields_abbreviated,
 *      which only contains the is_present fields that have been set to true in the
 *      current tag. This list can allow a performance improvement if most ports are
 *      seldom present because only fields that have been set to true need to be
 *      reset to false.
 *
 * * _lf_tokens_with_ref_count: An array of pointers to structs that point to lf_token_t
 *   objects, which carry non-primitive data types between reactors. This is used
 *   by the _lf_start_time_step() function to decrement reference counts, if necessary,
 *   at the conclusion of a time step. Then the reference count reaches zero, the
 *   memory allocated for the lf_token_t object will be freed.  The size of this
 *   array is stored in the _lf_tokens_with_ref_count_size variable.
 * 
 * * _lf_shutdown_triggers: An array of pointers to trigger_t structs for shutdown
 *   reactions. The length of this table is in the _lf_shutdown_triggers_size
 *   variable.
 * 
 * * _lf_timer_triggers: An array of pointers to trigger_t structs for timers that
 *   need to be started when the program runs. The length of this table is in the
 *   _lf_timer_triggers_size variable.
 * 
 * * _lf_action_table: For a federated execution, each federate will have this table
 *   that maps port IDs to the corresponding trigger_t struct.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Mehrdad Niknami <mniknami@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 * @author {Matt Weber <matt.weber@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
class CGenerator extends GeneratorBase {
    protected new(FileConfig fileConfig, ErrorReporter errorReporter, boolean CCppMode, CTypes types) {
        super(fileConfig, errorReporter)
        this.CCppMode = CCppMode;
        this.types = types
    }

    new(FileConfig fileConfig, ErrorReporter errorReporter, boolean CCppMode) {
        this(fileConfig, errorReporter, CCppMode, new CTypes(errorReporter))
    }

    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        this(fileConfig, errorReporter, false)
    }

    ////////////////////////////////////////////
    //// Public methods
    /**
     * Set C-specific default target configurations if needed.
     */
    def setCSpecificDefaults(LFGeneratorContext context) {
        if (!targetConfig.useCmake && targetConfig.compiler.isNullOrEmpty) {
            if (this.CCppMode) {
                targetConfig.compiler = "g++"
                targetConfig.compilerFlags.addAll("-O2", "-Wno-write-strings")
            } else {
                targetConfig.compiler = "gcc"
                targetConfig.compilerFlags.addAll("-O2") // "-Wall -Wconversion"
            }
        }
        if (isFederated) {
            // Add compile definitions for federated execution
            targetConfig.compileDefinitions.put("FEDERATED", "");
            if (targetConfig.coordination === CoordinationType.CENTRALIZED) {
                // The coordination is centralized.
                targetConfig.compileDefinitions.put("FEDERATED_CENTRALIZED", "");                
            } else if (targetConfig.coordination === CoordinationType.DECENTRALIZED) {
                // The coordination is decentralized
                targetConfig.compileDefinitions.put("FEDERATED_DECENTRALIZED", "");  
            }        
        }
    }
    
    /**
     * Look for physical actions in all resources.
     * If found, set threads to be at least one to allow asynchronous schedule calls.
     */
    def accommodatePhysicalActionsIfPresent() {
        // If there are any physical actions, ensure the threaded engine is used and that
        // keepalive is set to true, unless the user has explicitly set it to false.
        for (resource : GeneratorUtils.getResources(reactors)) {
            for (action : resource.allContents.toIterable.filter(Action)) {
                if (action.origin == ActionOrigin.PHYSICAL) {
                    // If the unthreaded runtime is requested, use the threaded runtime instead
                    // because it is the only one currently capable of handling asynchronous events.
                    if (!targetConfig.threading) {
                        targetConfig.threading = true
                        errorReporter.reportWarning(
                            action,
                            '''Using the threaded C runtime to allow for asynchronous handling of«
                            » physical action «action.name».'''
                        );
                        return;
                    }
                }
            }
        }
    }
    
    /**
     * Return true if the host operating system is compatible and
     * otherwise report an error and return false.
     */
    protected def boolean isOSCompatible() {
        if (GeneratorUtils.isHostWindows) {
            if (isFederated) { 
                errorReporter.reportError(
                    "Federated LF programs with a C target are currently not supported on Windows. " + 
                    "Exiting code generation."
                )
                // Return to avoid compiler errors
                return false
            }
            if (CCppMode) {
                errorReporter.reportError(
                    "LF programs with a CCpp target are currently not supported on Windows. " + 
                    "Exiting code generation."
                )
                // FIXME: The incompatibility between our C runtime code and the
                //  Visual Studio compiler is extensive. 
                return false;             
            }
            if (targetConfig.useCmake == false) {
                errorReporter.reportError(
                    "Only CMake is supported as the build system on Windows. "+
                    "Use `cmake: true` in the target properties. Exiting code generation."
                )
                return false;
            }
        }
        return true;
    }

    /**
     * Generate C code from the Lingua Franca model contained by the
     * specified resource. This is the main entry point for code
     * generation.
     * @param resource The resource containing the source code.
     * @param context The context in which the generator is
     *     invoked, including whether it is cancelled and
     *     whether it is a standalone context
     */
    override void doGenerate(Resource resource, LFGeneratorContext context) {
        super.doGenerate(resource, context)
        if (!GeneratorUtils.canGenerate(errorsOccurred, mainDef, errorReporter, context)) return;
        if (!isOSCompatible()) return; // Incompatible OS and configuration

        // Perform set up that does not generate code
        setUpParameters(context)

        // Create the output directories if they don't yet exist.
        var dir = fileConfig.getSrcGenPath.toFile
        if (!dir.exists()) dir.mkdirs()
        dir = fileConfig.binPath.toFile
        if (!dir.exists()) dir.mkdirs()

        // Docker related paths
        var dockerComposeDir = fileConfig.getSrcGenPath().toFile();
        var dockerComposeServices = new StringBuilder();

        // Perform distinct code generation into distinct files for each federate.
        val baseFilename = topLevelName
        
        // Copy the code generated so far.
        var commonCode = new CodeBuilder(code);
        
        // Keep a separate file config for each federate
        val oldFileConfig = fileConfig;
        val numOfCompileThreads = Math.min(6,
                Math.min(
                    Math.max(federates.size, 1), 
                    Runtime.getRuntime().availableProcessors()
                )
            )
        val compileThreadPool = Executors.newFixedThreadPool(numOfCompileThreads);
        System.out.println("******** Using "+numOfCompileThreads+" threads to compile the program.");
        var federateCount = 0;
        val LFGeneratorContext generatingContext = new SubContext(
            context, IntegratedBuilder.VALIDATED_PERCENT_PROGRESS, IntegratedBuilder.GENERATED_PERCENT_PROGRESS
        )
        for (federate : federates) {
            currentFederate = federate;
            federateCount++;
            startTimeStepIsPresentCount = 0
            startTimeStepTokens = 0
            
            // If federated, append the federate name to the file name.
            // Only generate one output if there is no federation.
            if (isFederated) {
                topLevelName = baseFilename + '_' + federate.name // FIXME: don't (temporarily) reassign a class variable for this
                fileConfig = new FedFileConfig(fileConfig, federate.name);
                
                // Reset the cmake-includes and files, to be repopulated for each federate individually.
                // This is done to enable support for separately
                // adding cmake-includes/files for different federates to prevent linking and mixing
                // all federates' supporting libraries/files together.
                targetConfig.cmakeIncludes.clear();
                targetConfig.cmakeIncludesWithoutPath.clear();
                targetConfig.fileNames.clear();
                targetConfig.filesNamesWithoutPath.clear();
                
                // Re-apply the cmake-include target property of the main .lf file.
                val target = GeneratorUtils.findTarget(mainDef.reactorClass.eResource)
                if (target.config !== null) {
                    // Update the cmake-include
                    TargetProperty.updateOne(
                        this.targetConfig, 
                        TargetProperty.CMAKE_INCLUDE,
                        target.config.pairs ?: emptyList,
                        errorReporter
                    )
                    // Update the files
                    TargetProperty.updateOne(
                        this.targetConfig, 
                        TargetProperty.FILES,
                        target.config.pairs ?: emptyList,
                        errorReporter
                    )
                }
                
                // Need to copy user files again since the source structure changes
                // for federated programs.
                copyUserFiles(this.targetConfig, this.fileConfig);
                
                // Clear out previously generated code.
                code = new CodeBuilder(commonCode)
                initializeTriggerObjects = new CodeBuilder()
                        
                // Enable clock synchronization if the federate
                // is not local and clock-sync is enabled
                initializeClockSynchronization()
                

                startTimeStep = new CodeBuilder()
            }
            
            // Copy the core lib
            FileUtil.copyFilesFromClassPath(
                "/lib/c/reactor-c/core", 
                fileConfig.getSrcGenPath.resolve("core"),
                CCoreFilesUtils.getCoreFiles(
                    isFederated,
                    targetConfig.threading,
                    targetConfig.schedulerType
                )
            )
            // Copy the header files
            copyTargetHeaderFile()

            generateDirectives()
            generateTopLevelPreambles();
            code.pr(CMainGenerator.generateCode());
            // Generate code for each reactor.
            generateReactorDefinitions();
        
            // Derive target filename from the .lf filename.
            val cFilename = CCompiler.getTargetFileName(topLevelName, this.CCppMode);

            var file = fileConfig.getSrcGenPath().resolve(cFilename).toFile
            // Delete source previously produced by the LF compiler.
            if (file.exists) {
                file.delete
            }

            // Delete binary previously produced by the C compiler.
            file = fileConfig.binPath.resolve(topLevelName).toFile
            if (file.exists) {
                file.delete
            }

            // Generate main instance, if there is one.
            // Note that any main reactors in imported files are ignored.
            // Skip generation if there are cycles.      
            if (this.main !== null) {
                initializeTriggerObjects.pr('''
                    int _lf_startup_reactions_count = 0;
                    int _lf_shutdown_reactions_count = 0;
                    int _lf_timer_triggers_count = 0;
                    int _lf_tokens_with_ref_count_count = 0;
                ''');

                // Create an array of arrays to store all self structs.
                // This is needed because connections cannot be established until
                // all reactor instances have self structs because ports that
                // receive data reference the self structs of the originating
                // reactors, which are arbitarily far away in the program graph.
                generateSelfStructs(main);

                generateReactorInstance(this.main)
                // Generate function to set default command-line options.
                // A literal array needs to be given outside any function definition,
                // so start with that.
                if (runCommand.length > 0) {
                    code.pr('''
                        char* _lf_default_argv[] = { "«runCommand.join('", "')»" };
                        void _lf_set_default_command_line_options() {
                            default_argc = «runCommand.length»;
                            default_argv = _lf_default_argv;
                        }
                    ''');
                } else {
                    code.pr("void _lf_set_default_command_line_options() {}");
                }
                
                // If there are timers, create a table of timers to be initialized.
                code.pr(CTimerGenerator.generateDeclarations(timerCount));
                
                // If there are shutdown reactions, create a table of triggers.
                code.pr(CReactionGenerator.generateShutdownTriggersTable(shutdownReactionCount));
                
                // If there are modes, create a table of mode state to be checked for transitions.
                code.pr(CModesGenerator.generateModeStatesTable(
                    hasModalReactors,
                    modalReactorCount,
                    modalStateResetCount
                ));
                
                // Generate function to return a pointer to the action trigger_t
                // that handles incoming network messages destined to the specified
                // port. This will only be used if there are federates.
                if (federate.networkMessageActions.size > 0) {
                    // Create a static array of trigger_t pointers.
                    // networkMessageActions is a list of Actions, but we
                    // need a list of trigger struct names for ActionInstances.
                    // There should be exactly one ActionInstance in the
                    // main reactor for each Action.
                    val triggers = new LinkedList<String>()
                    for (action : federate.networkMessageActions) {
                        // Find the corresponding ActionInstance.
                        val actionInstance = main.lookupActionInstance(action)
                        triggers.add(CUtil.triggerRef(actionInstance, null))
                    }
                    var actionTableCount = 0
                    for (trigger : triggers) {
                        initializeTriggerObjects.pr('''
                            _lf_action_table[«actionTableCount++»] = &«trigger»;
                        ''')
                    }
                    code.pr('''
                        trigger_t* _lf_action_table[«federate.networkMessageActions.size»];
                        trigger_t* _lf_action_for_port(int port_id) {
                            if (port_id < «federate.networkMessageActions.size») {
                                return _lf_action_table[port_id];
                            } else {
                                return NULL;
                            }
                        }
                    ''')
                } else {
                    code.pr('''
                        trigger_t* _lf_action_for_port(int port_id) {
                            return NULL;
                        }
                    ''')
                }
                
                // Generate function to initialize the trigger objects for all reactors.
                code.pr(CTriggerObjectsGenerator.generateInitializeTriggerObjects(
                    federate,
                    main,
                    targetConfig,
                    initializeTriggerObjects,
                    startTimeStep,
                    types,
                    topLevelName,
                    federationRTIProperties,
                    startTimeStepTokens,
                    startTimeStepIsPresentCount,
                    startupReactionCount,
                    isFederated,
                    isFederatedAndDecentralized,
                    clockSyncIsOn
                )); 

                // Generate function to trigger startup reactions for all reactors.
                code.pr(CReactionGenerator.generateLfTriggerStartupReactions(startupReactionCount));

                // Generate function to schedule timers for all reactors.
                code.pr(CTimerGenerator.generateLfInitializeTimer(timerCount));

                // Generate a function that will either do nothing
                // (if there is only one federate or the coordination 
                // is set to decentralized) or, if there are
                // downstream federates, will notify the RTI
                // that the specified logical time is complete.
                code.pr('''
                    void logical_tag_complete(tag_t tag_to_send) {
                        «IF isFederatedAndCentralized»
                            _lf_logical_tag_complete(tag_to_send);
                        «ENDIF»
                    }
                ''')
                
                if (isFederated) {
                    code.pr(CFederateGenerator.generateFederateNeighborStructure(federate).toString());
                }
                                
                // Generate function to schedule shutdown reactions if any
                // reactors have reactions to shutdown.
                code.pr(CReactionGenerator.generateLfTriggerShutdownReactions(
                    shutdownReactionCount
                ));
                
                // Generate an empty termination function for non-federated
                // execution. For federated execution, an implementation is
                // provided in federate.c.  That implementation will resign
                // from the federation and close any open sockets.
                if (!isFederated) {
                    code.pr("void terminate_execution() {}");
                }
                
                code.pr(CModesGenerator.generateLfHandleModeChanges(
                    hasModalReactors,
                    modalStateResetCount
                ));
            }
            val targetFile = fileConfig.getSrcGenPath() + File.separator + cFilename
            code.writeToFile(targetFile)
            
            
            if (targetConfig.useCmake) {
                // If cmake is requested, generated the CMakeLists.txt
                val cmakeGenerator = new CCmakeGenerator(targetConfig, fileConfig)
                val cmakeFile = fileConfig.getSrcGenPath() + File.separator + "CMakeLists.txt"
                val cmakeCode = cmakeGenerator.generateCMakeCode(
                        #[cFilename], 
                        topLevelName, 
                        errorReporter,
                        CCppMode,
                        mainDef !== null,
                        cMakeExtras
                )
                cmakeCode.writeToFile(cmakeFile)
            }
            
            // Create docker file.
            if (targetConfig.dockerOptions !== null) {
                var dockerFileName = topLevelName + '.Dockerfile'
                if (isFederated) {
                    writeDockerFile(dockerComposeDir, dockerFileName, federate.name)
                    DockerComposeGenerator.appendFederateToDockerComposeServices(dockerComposeServices, federate.name, federate.name, dockerFileName)
                } else {
                    writeDockerFile(dockerComposeDir, dockerFileName, topLevelName.toLowerCase())
                    DockerComposeGenerator.appendFederateToDockerComposeServices(dockerComposeServices, topLevelName.toLowerCase(), ".", dockerFileName)
                }
            }

            // If this code generator is directly compiling the code, compile it now so that we
            // clean it up after, removing the #line directives after errors have been reported.
            if (
                !targetConfig.noCompile
                && targetConfig.buildCommands.nullOrEmpty
                && !federate.isRemote
                // This code is unreachable in LSP_FAST mode, so that check is omitted.
                && context.getMode() != LFGeneratorContext.Mode.LSP_MEDIUM
            ) {
                // FIXME: Currently, a lack of main is treated as a request to not produce
                // a binary and produce a .o file instead. There should be a way to control
                // this. 
                // Create an anonymous Runnable class and add it to the compileThreadPool
                // so that compilation can happen in parallel.
                val cleanCode = code.removeLines("#line");
                
                val execName = topLevelName
                val threadFileConfig = fileConfig;
                val generator = this; // FIXME: currently only passed to report errors with line numbers in the Eclipse IDE
                val CppMode = CCppMode;
                generatingContext.reportProgress(
                    String.format("Generated code for %d/%d executables. Compiling...", federateCount, federates.size()),
                    100 * federateCount / federates.size()
                );
                compileThreadPool.execute(new Runnable() {
                    override void run() {
                        // Create the compiler to be used later
                        var cCompiler = new CCompiler(targetConfig, threadFileConfig,
                            errorReporter, CppMode);
                        if (targetConfig.useCmake) {
                            // Use CMake if requested.
                            cCompiler = new CCmakeCompiler(targetConfig, threadFileConfig,
                                errorReporter, CppMode);
                        }
                        if (!cCompiler.runCCompiler(execName, main === null, generator, context)) {
                            // If compilation failed, remove any bin files that may have been created.
                            CUtil.deleteBinFiles(threadFileConfig)
                            // If finish has already been called, it is illegal and makes no sense. However,
                            //  if finish has already been called, then this must be a federated execution.
                            if (!isFederated) context.unsuccessfulFinish();
                        } else if (!isFederated) context.finish(
                            GeneratorResult.Status.COMPILED, execName, fileConfig, null
                        );
                        cleanCode.writeToFile(targetFile)
                    }
                });
            }
            fileConfig = oldFileConfig;
        }

        if (targetConfig.dockerOptions !== null) {
            if (isFederated) {
                DockerComposeGenerator.appendRtiToDockerComposeServices(
                    dockerComposeServices,  
                    "lflang/rti:rti", 
                    federationRTIProperties.get("host").toString,
                    federates.size
                );
            }
            DockerComposeGenerator.writeFederatesDockerComposeFile(dockerComposeDir, dockerComposeServices, "lf");
        }
        
        // Initiate an orderly shutdown in which previously submitted tasks are 
        // executed, but no new tasks will be accepted.
        compileThreadPool.shutdown();
        
        // Wait for all compile threads to finish (NOTE: Can block forever)
        compileThreadPool.awaitTermination(Long.MAX_VALUE, TimeUnit.NANOSECONDS);
        
        // Restore the base filename.
        topLevelName = baseFilename

        if (isFederated) {
            createFederatedLauncher();
        }

        // If a build directive has been given, invoke it now.
        // Note that the code does not get cleaned in this case.
        if (!targetConfig.noCompile) {
            if (!targetConfig.buildCommands.nullOrEmpty) {
                CUtil.runBuildCommand(
                    fileConfig,
                    targetConfig,
                    commandFactory,
                    errorReporter,
                    [it | reportCommandErrors(it)],
                    context.mode
                )
                context.finish(
                    GeneratorResult.Status.COMPILED, fileConfig.name, fileConfig, null
                );
            } else if (isFederated) {
                context.finish(
                    GeneratorResult.Status.COMPILED, fileConfig.name, fileConfig, null
                );
            }
            println("Compiled binary is in " + fileConfig.binPath);
        } else {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(null));
        }
        
        // In case we are in Eclipse, make sure the generated code is visible.
        GeneratorUtils.refreshProject(resource, context.mode)
    }
    
    override checkModalReactorSupport(boolean _) {
        // Modal reactors are currently only supported for non federated applications
        super.checkModalReactorSupport(!isFederated);
    }
    
    override protected String getConflictingConnectionsInModalReactorsBody(String source, String dest) {
        return String.join("\n",
            "// Generated forwarding reaction for connections with the same destination",
            "// but located in mutually exclusive modes.",
            "SET("+dest+", "+source+"->value);"
        );
    }
    
    /**
     * Add files needed for the proper function of the runtime scheduler to
     * {@code coreFiles} and {@link TargetConfig#compileAdditionalSources}.
     */
    def pickScheduler() {
        // Don't use a scheduler that does not prioritize reactions based on deadlines
        // if the program contains a deadline (handler). Use the GEDF_NP scheduler instead.
        if (!targetConfig.schedulerType.prioritizesDeadline) {
            // Check if a deadline is assigned to any reaction
            if (reactors.filter[reactor |
                // Filter reactors that contain at least one reaction 
                // that has a deadline handler.
                return reactor.allReactions.filter[ reaction |
                    return reaction.deadline !== null
                ].size > 0;
            ].size > 0) {
                if (!targetConfig.setByUser.contains(TargetProperty.SCHEDULER)) {
                    targetConfig.schedulerType = TargetProperty.SchedulerOption.GEDF_NP;
                }
            }        
        }
        targetConfig.compileAdditionalSources.add(
             "core" + File.separator + "threaded" + File.separator + 
             "scheduler_" + targetConfig.schedulerType.toString() + ".c"
        );
        System.out.println("******** Using the "+targetConfig.schedulerType.toString()+" runtime scheduler.");
        targetConfig.compileAdditionalSources.add(
            "core" + File.separator + "utils" + File.separator + "semaphore.c"
        );
    }
    
    /**
     * Look at the 'reactor' eResource.
     * If it is an imported .lf file, incorporate it into the current 
     * program in the following manner:
     * - Merge its target property with `targetConfig`
     * - If there are any preambles, add them to the preambles of the reactor.
     */
    def inspectReactorEResource(ReactorDecl reactor) {
        // If the reactor is imported, look at the
        // target definition of the .lf file in which the reactor is imported from and
        // append any cmake-include.
        // Check if the reactor definition is imported
        if (reactor.eResource !== mainDef.reactorClass.eResource) {
            // Find the LFResource corresponding to this eResource
            val lfResource = resources.filter[ 
                r | return r.EResource === reactor.eResource;
            ].get(0);
            
            // Copy the user files and cmake-includes to the src-gen path of the main .lf file
            if (lfResource !== null) {
                copyUserFiles(lfResource.targetConfig, lfResource.fileConfig);
            }
            
            // Extract the contents of the imported file for the preambles
            val contents = reactor.toDefinition.eResource.contents;
            val model = contents.get(0) as Model
            // Add the preambles from the imported .lf file
            reactor.toDefinition.preambles.addAll(model.preambles)
        }
    }
    
    /**
     * Copy all files listed in the target property `files` and `cmake-include` 
     * into the src-gen folder of the main .lf file
     * 
     * @param targetConfig The targetConfig to read the `files` and `cmake-include` from.
     * @param fileConfig The fileConfig used to make the copy and resolve paths.
     */
    override copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {
        super.copyUserFiles(targetConfig, fileConfig)
        // Make sure the target directory exists.
        val targetDir = this.fileConfig.getSrcGenPath
        Files.createDirectories(targetDir)

        for (filename : targetConfig.fileNames) {
            val relativeFileName = CUtil.copyFileOrResource(
                    filename,
                    fileConfig.srcFile.parent,
                    targetDir);
            if (relativeFileName.isNullOrEmpty) {
                errorReporter.reportError(
                    "Failed to find file " + filename + " specified in the" +
                    " files target property."
                )
            } else {
                this.targetConfig.filesNamesWithoutPath.add(
                    relativeFileName
                );
            }
        }

        for (filename : targetConfig.cmakeIncludes) {
            val relativeCMakeIncludeFileName = 
                CUtil.copyFileOrResource(
                    filename,
                    fileConfig.srcFile.parent,
                    targetDir);
            // Check if the file exists
            if (relativeCMakeIncludeFileName.isNullOrEmpty) {
                errorReporter.reportError( 
                    "Failed to find cmake-include file " + filename
                )
            } else {
                this.targetConfig.cmakeIncludesWithoutPath.add(
                    relativeCMakeIncludeFileName
                );
            }
        }
    }
    
    /**
     * Generate code for defining all reactors that belong to the federate, 
     * including all the child reactors down the hierarchy. Duplicate
     * Duplicates are avoided.
     * 
     * Imported reactors' original .lf file is 
     * incorporated in the following manner:
     * - If there are any cmake-include files, add them to the current list
     *  of cmake-include files.
     * - If there are any preambles, add them to the preambles of the reactor.
     */
    private def void generateReactorDefinitions() {
        val generatedReactorDecls = newLinkedHashSet
        if (this.main !== null) {
            generateReactorChildren(this.main, generatedReactorDecls);
        }

        if (this.mainDef !== null) {
            generateReactorClass(this.mainDef.reactorClass)
        }

        if (mainDef === null) {
            // Generate code for each reactor that was not instantiated in main or its children.
            for (r : reactors) {
                // Get the declarations for reactors that are instantiated somewhere.
                // A declaration is either a reactor definition or an import statement.;
                val declarations = this.instantiationGraph.getDeclarations(r);
                // If the reactor has no instantiations and there is no main reactor, then
                // generate code for it anyway (at a minimum, this means that the compiler is invoked
                // so that reaction bodies are checked).
                if (declarations.isEmpty()) {
                    generateReactorClass(r)
                }
            }
        }
    }
    
    /**
     * Generate code for the children of 'reactor' that belong to 'federate'.
     * Duplicates are avoided. 
     * 
     * Imported reactors' original .lf file is 
     * incorporated in the following manner:
     * - If there are any cmake-include files, add them to the current list
     *  of cmake-include files.
     * - If there are any preambles, add them to the preambles of the reactor.
     * 
     * @param reactor Used to extract children from
     */
    private def void generateReactorChildren(
        ReactorInstance reactor,
        LinkedHashSet<ReactorDecl> generatedReactorDecls
    ) {
        for (r : reactor.children) {
            if (currentFederate.contains(r) && 
                  r.reactorDeclaration !== null &&
                  !generatedReactorDecls.contains(r.reactorDeclaration)) {
                generatedReactorDecls.add(r.reactorDeclaration);
                generateReactorChildren(r, generatedReactorDecls);
                inspectReactorEResource(r.reactorDeclaration);
                generateReactorClass(r.reactorDeclaration);
            }
        }
    }
    
    /**
     * Choose which platform files to compile with according to the OS.
     * If there is no main reactor, then compilation will produce a .o file requiring further linking.
     * Also, if useCmake is set to true, we don't need to add platform files. The CMakeLists.txt file
     * will detect and use the appropriate platform file based on the platform that cmake is invoked on.
     */
    def pickCompilePlatform() {
        val OS = System.getProperty("os.name").toLowerCase();
        // FIXME: allow for cross-compiling
        if ((OS.indexOf("mac") >= 0) || (OS.indexOf("darwin") >= 0)) {
            if (mainDef !== null && !targetConfig.useCmake) {
                targetConfig.compileAdditionalSources.add(
                     "core" + File.separator + "platform" + File.separator + "lf_macos_support.c"
                );
            }
        } else if (OS.indexOf("win") >= 0) {
            if (mainDef !== null && !targetConfig.useCmake) {
                targetConfig.compileAdditionalSources.add(
                    "core" + File.separator + "platform" + File.separator + "lf_windows_support.c"
                )
            }
        } else if (OS.indexOf("nux") >= 0) {
            if (mainDef !== null && !targetConfig.useCmake) {
                targetConfig.compileAdditionalSources.add(
                    "core" + File.separator + "platform" + File.separator + "lf_linux_support.c"
                )
            }
        } else {
            errorReporter.reportError("Platform " + OS + " is not supported")
        }
    }
    
    /**
     * Create a launcher script that executes all the federates and the RTI.
     * 
     * @param coreFiles The files from the core directory that must be
     *  copied to the remote machines.
     */
    def createFederatedLauncher() {
        val launcher = new FedCLauncher(
            targetConfig,
            fileConfig,
            errorReporter
        );
        launcher.createLauncher(
            federates,
            federationRTIProperties
        );
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
        if (mainDef === null) {
            return
        }
        var srcGenPath = fileConfig.getSrcGenPath
        val dockerFile = srcGenPath + File.separator + dockerFileName
        // If a dockerfile exists, remove it.
        var file = new File(dockerFile)
        if (file.exists) {
            file.delete
        }
        val contents = new CodeBuilder()
        val compileCommand = targetConfig.buildCommands.nullOrEmpty ? 
                                 CDockerGenerator.generateDefaultCompileCommand() : 
                                 targetConfig.buildCommands.join(' ')
        contents.pr(CDockerGenerator.generateDockerFileContent(
            topLevelName, 
            targetConfig.dockerOptions.from, 
            CCppMode ? "g++" : "gcc",
            compileCommand, 
            srcGenPath)
        );
        contents.writeToFile(dockerFile)
        println(getDockerBuildCommand(dockerFile, dockerComposeDir, federateName))
    }

    def clockSyncIsOn() {
        return targetConfig.clockSync != ClockSyncMode.OFF
            && (!federationRTIProperties.get('host').toString.equals(currentFederate.host)
            || targetConfig.clockSyncOptions.localFederatesOn);
    }

    /**
     * Initialize clock synchronization (if enabled) and its related options for a given federate.
     * 
     * Clock synchronization can be enabled using the clock-sync target property.
     * @see https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization
     */
    protected def initializeClockSynchronization() {
        // Check if clock synchronization should be enabled for this federate in the first place
        if (clockSyncIsOn()) {
            System.out.println("Initial clock synchronization is enabled for federate "
                + currentFederate.id
            );
            if (targetConfig.clockSync == ClockSyncMode.ON) {
                if (targetConfig.clockSyncOptions.collectStats) {
                    System.out.println("Will collect clock sync statistics for federate " + currentFederate.id)
                    // Add libm to the compiler flags
                    // FIXME: This is a linker flag not compile flag but we don't have a way to add linker flags
                    // FIXME: This is probably going to fail on MacOS (especially using clang)
                    // because libm functions are builtin
                    targetConfig.compilerFlags.add("-lm")
                }
                System.out.println("Runtime clock synchronization is enabled for federate "
                    + currentFederate.id
                );
            }
        }
    }
    
    /**
     * Copy target-specific header file to the src-gen directory.
     */
    def copyTargetHeaderFile() {
        FileUtil.copyFileFromClassPath("/lib/c/reactor-c/include/ctarget.h", fileConfig.getSrcGenPath.resolve("ctarget.h"))
        FileUtil.copyFileFromClassPath("/lib/c/reactor-c/lib/ctarget.c", fileConfig.getSrcGenPath.resolve("ctarget.c"))
    }

    ////////////////////////////////////////////
    //// Code generators.
    /** 
     * Generate a reactor class definition for the specified federate.
     * A class definition has four parts:
     * 
     * * Preamble code, if any, specified in the Lingua Franca file.
     * * A "self" struct type definition (see the class documentation above).
     * * A function for each reaction.
     * * A constructor for creating an instance.
     *  for deleting an instance.
     * 
     * If the reactor is the main reactor, then
     * the generated code may be customized. Specifically,
     * if the main reactor has reactions, these reactions
     * will not be generated if they are triggered by or send
     * data to contained reactors that are not in the federate.
     * @param reactor The parsed reactor data structure.
     */
    private def generateReactorClass(ReactorDecl reactor) {
        // FIXME: Currently we're not reusing definitions for declarations that point to the same definition.
        
        val defn = reactor.toDefinition
        
        if (reactor instanceof Reactor) {
            code.pr("// =============== START reactor class " + reactor.name)
        } else {
            code.pr("// =============== START reactor class " + defn.name + " as " + reactor.name)
        }
        
        // Preamble code contains state declarations with static initializers.
        generateUserPreamblesForReactor(defn)
            
        // Some of the following methods create lines of code that need to
        // go into the constructor.  Collect those lines of code here:
        val constructorCode = new CodeBuilder()
        generateAuxiliaryStructs(reactor)
        generateSelfStruct(reactor, constructorCode)
        generateReactions(reactor, currentFederate)
        generateConstructor(reactor, currentFederate, constructorCode)

        code.pr("// =============== END reactor class " + reactor.name)
        code.pr("")
    }
    
    /**
     * Generates preambles defined by user for a given reactor
     * @param reactor The given reactor
     */
    def generateUserPreamblesForReactor(Reactor reactor) {
        for (p : reactor.preambles ?: emptyList) {
            code.pr("// *********** From the preamble, verbatim:")
            code.prSourceLineNumber(p.code)
            code.pr(p.code.toText)
            code.pr("\n// *********** End of preamble.")
        }
    }
    
    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    protected def generateConstructor(
        ReactorDecl reactor, FederateInstance federate, CodeBuilder constructorCode
    ) {
        val structType = CUtil.selfType(reactor)
        code.pr('''
            «structType»* new_«reactor.name»() {
                «structType»* self = («structType»*)_lf_new_reactor(sizeof(«structType»));
                «constructorCode.toString»
                return self;
            }
        ''')
    }

    /**
     * Generate the struct type definitions for inputs, outputs, and
     * actions of the specified reactor.
     * @param reactor The parsed reactor data structure.
     */
    protected def generateAuxiliaryStructs(ReactorDecl decl) {
        val reactor = decl.toDefinition
        // In the case where there are incoming
        // p2p logical connections in decentralized
        // federated execution, there will be an
        // intended_tag field added to accommodate
        // the case where a reaction triggered by a
        // port or action is late due to network 
        // latency, etc..
        var federatedExtension = new CodeBuilder();    
        if (isFederatedAndDecentralized) {
            federatedExtension.pr('''«types.getTargetTagType» intended_tag;''');
        }
        if (isFederated) {
            federatedExtension.pr('''«types.getTargetTimeType» physical_time_of_arrival;''');
        }
        // First, handle inputs.
        for (input : reactor.allInputs) {
            code.pr(CPortGenerator.generateAuxiliaryStruct(
                decl,
                input,
                target,
                errorReporter,
                types,
                federatedExtension
            ))
        }
        // Next, handle outputs.
        for (output : reactor.allOutputs) {
            code.pr(CPortGenerator.generateAuxiliaryStruct(
                decl,
                output,
                target,
                errorReporter,
                types,
                federatedExtension
            ))
        }
        // Finally, handle actions.
        // The very first item on this struct needs to be
        // a trigger_t* because the struct will be cast to (trigger_t*)
        // by the schedule() functions to get to the trigger.
        for (action : reactor.allActions) {
            if (currentFederate.contains(action)) {
                code.pr(action, '''
                    typedef struct {
                        trigger_t* trigger;
                        «action.valueDeclaration»
                        bool is_present;
                        bool has_value;
                        lf_token_t* token;
                        «federatedExtension.toString»
                    } «variableStructType(action, decl)»;
                ''')
            }
            
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
    protected def valueDeclaration(Action action) {
        if (action.type === null && target.requiresTypes === true) {
            return ''
        }
        // Do not convert to lf_token_t* using lfTypeToTokenType because there
        // will be a separate field pointing to the token.
        return types.getTargetType(action) + " value;"
    }

    /**
     * Generate the self struct type definition for the specified reactor
     * in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param constructorCode Place to put lines of code that need to
     *  go into the constructor.
     */
    private def generateSelfStruct(ReactorDecl decl, CodeBuilder constructorCode) {
        val reactor = decl.toDefinition
        val selfType = CUtil.selfType(decl)
        
        // Construct the typedef for the "self" struct.
        // Create a type name for the self struct.
        var body = new CodeBuilder()
        
        // Extensions can add functionality to the CGenerator
        generateSelfStructExtension(body, decl, constructorCode)
        
        // Next handle parameters.
        body.pr(CParameterGenerator.generateDeclarations(reactor, types))
        
        // Next handle states.
        body.pr(CStateGenerator.generateDeclarations(reactor, types))
        
        // Next handle actions.
        CActionGenerator.generateDeclarations(reactor, decl, currentFederate, body, constructorCode)
        
        // Next handle inputs and outputs.
        CPortGenerator.generateDeclarations(reactor, decl, body, constructorCode)
        
        // If there are contained reactors that either receive inputs
        // from reactions of this reactor or produce outputs that trigger
        // reactions of this reactor, then we need to create a struct
        // inside the self struct for each contained reactor. That
        // struct has a place to hold the data produced by this reactor's
        // reactions and a place to put pointers to data produced by
        // the contained reactors.
        generateInteractingContainedReactors(reactor, body, constructorCode);

        // Next, generate the fields needed for each reaction.
        CReactionGenerator.generateReactionAndTriggerStructs(
            currentFederate,
            body, 
            decl, 
            constructorCode,
            types,
            isFederated,
            isFederatedAndDecentralized
        );

        // Next, generate fields for modes
        CModesGenerator.generateDeclarations(reactor, body, constructorCode);

        // The first field has to always be a pointer to the list of
        // of allocated memory that must be freed when the reactor is freed.
        // This means that the struct can be safely cast to self_base_t.
        code.pr('''
            typedef struct {
                struct self_base_t base;
                «body.toString»
            } «selfType»;
        ''')
    }
    
    /**
     * Generate structs and associated code for contained reactors that
     * send or receive data to or from the container's reactions.
     * 
     * If there are contained reactors that either receive inputs
     * from reactions of this reactor or produce outputs that trigger
     * reactions of this reactor, then we need to create a struct
     * inside the self struct of the container for each contained reactor.
     * That struct has a place to hold the data produced by the container reactor's
     * reactions and a place to put pointers to data produced by
     * the contained reactors.
     * 
     * @param reactor The reactor.
     * @param body The place to put the struct definition for the contained reactors.
     * @param constructorCode The place to put matching code that goes in the container's constructor.
     */
    private def generateInteractingContainedReactors(
        Reactor reactor,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        // The contents of the struct will be collected first so that
        // we avoid duplicate entries and then the struct will be constructed.
        val contained = new InteractingContainedReactors(reactor, currentFederate);
        // Next generate the relevant code.
        for (containedReactor : contained.containedReactors) {
            // First define an _width variable in case it is a bank.
            var array = "";
            var width = -2;
            // If the instantiation is a bank, find the maximum bank width
            // to define an array.
            if (containedReactor.widthSpec !== null) {
                width = CReactionGenerator.maxContainedReactorBankWidth(containedReactor, null, 0, mainDef);
                array = "[" + width + "]";
            }
            // NOTE: The following needs to be done for each instance
            // so that the width can be parameter, not in the constructor.
            // Here, we conservatively use a width that is the largest of all isntances.
            constructorCode.pr('''
                // Set the _width variable for all cases. This will be -2
                // if the reactor is not a bank of reactors.
                self->_lf_«containedReactor.name»_width = «width»;
            ''')

            // Generate one struct for each contained reactor that interacts.
            body.pr('''struct {''')
            body.indent()
            for (port : contained.portsOfInstance(containedReactor)) {
                if (port instanceof Input) {
                    // If the variable is a multiport, then the place to store the data has
                    // to be malloc'd at initialization.
                    if (!ASTUtils.isMultiport(port)) {
                        // Not a multiport.
                        body.pr(port, '''
                            «variableStructType(port, containedReactor.reactorClass)» «port.name»;
                        ''')
                    } else {
                        // Is a multiport.
                        // Memory will be malloc'd in initialization.
                        body.pr(port, '''
                            «variableStructType(port, containedReactor.reactorClass)»** «port.name»;
                            int «port.name»_width;
                        ''')
                    }
                } else {
                    // Must be an output port.
                    // Outputs of contained reactors are pointers to the source of data on the
                    // self struct of the container.
                    if (!ASTUtils.isMultiport(port)) {
                        // Not a multiport.
                        body.pr(port, '''
                            «variableStructType(port, containedReactor.reactorClass)»* «port.name»;
                        ''')
                    } else {
                        // Is a multiport.
                        // Here, we will use an array of pointers.
                        // Memory will be malloc'd in initialization.
                        body.pr(port, '''
                            «variableStructType(port, containedReactor.reactorClass)»** «port.name»;
                            int «port.name»_width;
                        ''')
                    }
                    body.pr(port, '''
                        trigger_t «port.name»_trigger;
                    ''')
                    var reactorIndex = ''
                    if (containedReactor.widthSpec !== null) {
                        reactorIndex = '[reactor_index]'
                        constructorCode.pr('''
                            for (int reactor_index = 0; reactor_index < self->_lf_«containedReactor.name»_width; reactor_index++) {
                        ''')
                        constructorCode.indent()
                    }
                    val portOnSelf = '''self->_lf_«containedReactor.name»«reactorIndex».«port.name»'''
                    
                    if (isFederatedAndDecentralized) {
                        constructorCode.pr(port, '''
                            «portOnSelf»_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
                        ''')
                    }
                    val triggered = contained.reactionsTriggered(containedReactor, port)
                    if (triggered.size > 0) {
                        body.pr(port, '''
                            reaction_t* «port.name»_reactions[«triggered.size»];
                        ''')
                        var triggeredCount = 0
                        for (index : triggered) {
                            constructorCode.pr(port, '''
                                «portOnSelf»_reactions[«triggeredCount++»] = &self->_lf__reaction_«index»;
                            ''')
                        }
                        constructorCode.pr(port, '''
                            «portOnSelf»_trigger.reactions = «portOnSelf»_reactions;
                        ''')
                    } else {
                        // Since the self struct is created using calloc, there is no need to set
                        // self->_lf_«containedReactor.name».«port.name»_trigger.reactions = NULL
                    }
                    // Since the self struct is created using calloc, there is no need to set
                    // self->_lf_«containedReactor.name».«port.name»_trigger.token = NULL;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.is_present = false;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.is_timer = false;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.is_physical = false;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.drop = false;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.element_size = 0;
                    // self->_lf_«containedReactor.name».«port.name»_trigger.intended_tag = (0, 0);
                    constructorCode.pr(port, '''
                        «portOnSelf»_trigger.last = NULL;
                        «portOnSelf»_trigger.number_of_reactions = «triggered.size»;
                    ''')
                    
                    if (isFederated) {
                        // Set the physical_time_of_arrival
                        constructorCode.pr(port, '''
                            «portOnSelf»_trigger.physical_time_of_arrival = NEVER;
                        ''')
                    }
                    if (containedReactor.widthSpec !== null) {
                        constructorCode.unindent()
                        constructorCode.pr("}")
                    }
                }
            }
            body.unindent()
            body.pr('''
                } _lf_«containedReactor.name»«array»;
                int _lf_«containedReactor.name»_width;
            ''');
            
        }
    }
    
    /**
     * This function is provided to allow extensions of the CGenerator to append the structure of the self struct
     * @param body The body of the self struct
     * @param decl The reactor declaration for the self struct
     * @param constructorCode Code that is executed when the reactor is instantiated
     */
    def void generateSelfStructExtension(
        CodeBuilder body,
        ReactorDecl decl,
        CodeBuilder constructorCode
    ) {
        // Do nothing
    }
    
    /** Generate reaction functions definition for a reactor.
     *  These functions have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reactor The reactor.
     *  @param federate The federate, or null if this is not
     *   federated or not the main reactor and reactions should be
     *   unconditionally generated.
     */
    def generateReactions(ReactorDecl decl, FederateInstance federate) {
        var reactionIndex = 0;
        val reactor = decl.toDefinition
        for (reaction : reactor.allReactions) {
            if (federate === null || federate.contains(reaction)) {
                generateReaction(reaction, decl, reactionIndex)
            }
            // Increment reaction index even if the reaction is not in the federate
            // so that across federates, the reaction indices are consistent.
            reactionIndex++
        }
    }
    
    /** Generate a reaction function definition for a reactor.
     *  This function will have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reaction The reaction.
     *  @param reactor The reactor.
     *  @param reactionIndex The position of the reaction within the reactor. 
     */
    def generateReaction(Reaction reaction, ReactorDecl decl, int reactionIndex) {
        code.pr(CReactionGenerator.generateReaction(
            reaction,
            decl,
            reactionIndex,
            mainDef,
            errorReporter,
            types,
            isFederatedAndDecentralized,
            target.requiresTypes
        ))
    }
    
    /**
     * Record startup and shutdown reactions.
     * @param instance A reactor instance.
     */
    private def void recordStartupAndShutdown(ReactorInstance instance) {
        // For each reaction instance, allocate the arrays that will be used to
        // trigger downstream reactions.
        for (reaction : instance.reactions) {
            if (currentFederate.contains(reaction.getDefinition())) {
                val reactor = reaction.parent;
                
                val temp = new CodeBuilder();
                var foundOne = false;
                
                val reactionRef = CUtil.reactionRef(reaction)
                            
                // Next handle triggers of the reaction that come from a multiport output
                // of a contained reactor.  Also, handle startup and shutdown triggers.
                for (trigger : reaction.triggers) {
                    if (trigger.isStartup) {
                        temp.pr('''
                            _lf_startup_reactions[_lf_startup_reactions_count++] = &«reactionRef»;
                        ''')
                        startupReactionCount += currentFederate.numRuntimeInstances(reactor);
                        foundOne = true;
                    } else if (trigger.isShutdown) {
                        temp.pr('''
                            _lf_shutdown_reactions[_lf_shutdown_reactions_count++] = &«reactionRef»;
                        ''')
                        foundOne = true;
                        shutdownReactionCount += currentFederate.numRuntimeInstances(reactor);
    
                        if (targetConfig.tracing !== null) {
                            val description = CUtil.getShortenedName(reactor)
                            val reactorRef = CUtil.reactorRef(reactor)
                            temp.pr('''
                                _lf_register_trace_event(«reactorRef», &(«reactorRef»->_lf__shutdown),
                                        trace_trigger, "«description».shutdown");
                            ''')
                        }
                    }
                }
                if (foundOne) initializeTriggerObjects.pr(temp.toString);
            }
        }
    }

   

    /** 
     * Generate code to set up the tables used in _lf_start_time_step to decrement reference
     * counts and mark outputs absent between time steps. This function puts the code
     * into startTimeStep.
     */
    private def generateStartTimeStep(ReactorInstance instance) {
        // First, set up to decrement reference counts for each token type
        // input of a contained reactor that is present.
        for (child : instance.children) {
            if (currentFederate.contains(child) && child.inputs.size > 0) {
                
                // Avoid generating code if not needed.
                var foundOne = false;
                val temp = new CodeBuilder();
                
                temp.startScopedBlock(child, currentFederate, isFederated, true);

                for (input : child.inputs) {
                    if (CUtil.isTokenType((input.definition as Input).inferredType, types)) {
                        foundOne = true;
                        val portRef = CUtil.portRefName(input);
                        if (input.isMultiport()) {
                            temp.pr('''
                                for (int i = 0; i < «input.width»; i++) {
                                    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token
                                            = &«portRef»[i]->token;
                                    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status
                                            = (port_status_t*)&«portRef»[i]->is_present;
                                    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;
                                }
                            ''')
                        } else {
                            temp.pr('''
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token
                                        = &«portRef»->token;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status
                                        = (port_status_t*)&«portRef»->is_present;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;
                            ''')
                        }
                        startTimeStepTokens += currentFederate.numRuntimeInstances(input.parent) * input.width;
                    }
                }
                temp.endScopedBlock();

                if (foundOne) {
                    startTimeStep.pr(temp.toString());
                }
            }
        }
        // Avoid generating dead code if nothing is relevant.
        var foundOne = false;
        var temp = new CodeBuilder();
        var containerSelfStructName = CUtil.reactorRef(instance)

        // Handle inputs that get sent data from a reaction rather than from
        // another contained reactor and reactions that are triggered by an
        // output of a contained reactor.
        // Note that there may be more than one reaction reacting to the same
        // port so we have to avoid listing the port more than once.
        val portsSeen = new LinkedHashSet<PortInstance>();
        for (reaction : instance.reactions) {
            if (currentFederate.contains(reaction.definition)) {
                for (port : reaction.effects.filter(PortInstance)) {
                    if (port.definition instanceof Input && !portsSeen.contains(port)) {
                        portsSeen.add(port)
                        // This reaction is sending to an input. Must be
                        // the input of a contained reactor in the federate.
                        // NOTE: If instance == main and the federate is within a bank,
                        // this assumes that the reaction writes only to the bank member in the federate.
                        if (currentFederate.contains(port.parent)) {
                            foundOne = true;

                            temp.pr('''
                                // Add port «port.getFullName» to array of is_present fields.
                            ''')
                            
                            if (port.parent != instance) {
                                // The port belongs to contained reactor, so we also have
                                // iterate over the instance bank members.
                                temp.startScopedBlock();
                                temp.pr("int count = 0;");
                                temp.startScopedBlock(instance, currentFederate, isFederated, true);
                                temp.startScopedBankChannelIteration(port, currentFederate, null, isFederated);
                            } else {
                                temp.startScopedBankChannelIteration(port, currentFederate, "count", isFederated);
                            }
                            val portRef = CUtil.portRefNested(port);
                            val con = (port.isMultiport)? "->" : ".";
                            
                            temp.pr('''
                                _lf_is_present_fields[«startTimeStepIsPresentCount» + count] = &«portRef»«con»is_present;
                            ''')
                            if (isFederatedAndDecentralized) {
                                // Intended_tag is only applicable to ports in federated execution.
                                temp.pr('''
                                    _lf_intended_tag_fields[«startTimeStepIsPresentCount» + count] = &«portRef»«con»intended_tag;
                                ''')
                            }

                            startTimeStepIsPresentCount += port.width * currentFederate.numRuntimeInstances(port.parent);

                            if (port.parent != instance) {
                                temp.pr("count++;");
                                temp.endScopedBlock();
                                temp.endScopedBlock();
                                temp.endScopedBankChannelIteration(port, null);
                            } else {
                                temp.endScopedBankChannelIteration(port, "count");
                            }
                       }
                    }
                }
                // Find outputs of contained reactors that have token types and therefore
                // need to have their reference counts decremented.
                for (port : reaction.sources.filter(PortInstance)) {
                    if (port.isOutput && !portsSeen.contains(port)) {
                        portsSeen.add(port)
                        // This reaction is receiving data from the port.
                        if (CUtil.isTokenType((port.definition as Output).inferredType, types)) {
                            foundOne = true;
                            
                            temp.pr('''
                                // Add port «port.getFullName» to array _lf_tokens_with_ref_count.
                            ''')
                            
                            // Potentially have to iterate over bank members of the instance
                            // (parent of the reaction), bank members of the contained reactor (if a bank),
                            // and channels of the multiport (if multiport).
                            temp.startScopedBlock(instance, currentFederate, isFederated, true);
                            temp.startScopedBankChannelIteration(port, currentFederate, "count", isFederated);
                            
                            val portRef = CUtil.portRef(port, true, true, null, null, null);
                            
                            temp.pr('''
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token = &«portRef»->token;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status = (port_status_t*)&«portRef»->is_present;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;
                            ''')
                            startTimeStepTokens += port.width * currentFederate.numRuntimeInstances(port.parent);

                            temp.endScopedBankChannelIteration(port, "count");
                            temp.endScopedBlock();
                        }
                    }
                }
            }
        }
        if (foundOne) startTimeStep.pr(temp.toString());
        temp = new CodeBuilder();
        foundOne = false;
        
        for (action : instance.actions) {
            if (currentFederate === null || currentFederate.contains(action.definition)) {
                foundOne = true;
                temp.startScopedBlock(instance, currentFederate, isFederated, true);
                
                temp.pr('''
                    // Add action «action.getFullName» to array of is_present fields.
                    _lf_is_present_fields[«startTimeStepIsPresentCount»] 
                            = &«containerSelfStructName»->_lf_«action.name».is_present;
                ''')
                if (isFederatedAndDecentralized) {
                    // Intended_tag is only applicable to actions in federated execution with decentralized coordination.
                    temp.pr('''
                        // Add action «action.getFullName» to array of intended_tag fields.
                        _lf_intended_tag_fields[«startTimeStepIsPresentCount»] 
                                = &«containerSelfStructName»->_lf_«action.name».intended_tag;
                    ''')
                }
                startTimeStepIsPresentCount += currentFederate.numRuntimeInstances(action.parent);
                temp.endScopedBlock();
            }
        }
        if (foundOne) startTimeStep.pr(temp.toString());
        temp = new CodeBuilder();
        foundOne = false;
        
        // Next, set up the table to mark each output of each contained reactor absent.
        for (child : instance.children) {
            if (currentFederate.contains(child) && child.outputs.size > 0) {
                
                temp.startScopedBlock();
                temp.pr("int count = 0;");
                temp.startScopedBlock(child, currentFederate, isFederated, true);
        
                var channelCount = 0;
                for (output : child.outputs) {
                    if (!output.dependsOnReactions.isEmpty){
                        foundOne = true;
                        
                        temp.pr('''
                            // Add port «output.getFullName» to array of is_present fields.
                        ''')
                        temp.startChannelIteration(output);
                        
                        temp.pr('''
                            _lf_is_present_fields[«startTimeStepIsPresentCount» + count] = &«CUtil.portRef(output)».is_present;
                        ''')
                        
                        if (isFederatedAndDecentralized) {
                            // Intended_tag is only applicable to ports in federated execution with decentralized coordination.
                            temp.pr('''
                                // Add port «output.getFullName» to array of intended_tag fields.
                                _lf_intended_tag_fields[«startTimeStepIsPresentCount» + count] = &«CUtil.portRef(output)».intended_tag;
                            ''')
                        }
                        
                        temp.pr("count++;");
                        channelCount += output.width;
                        temp.endChannelIteration(output);
                    }
                }
                startTimeStepIsPresentCount += channelCount * currentFederate.numRuntimeInstances(child);
                temp.endScopedBlock();
                temp.endScopedBlock();
            }
        }
        if (foundOne) startTimeStep.pr(temp.toString());
    }

    /**
     * For each timer in the given reactor, generate initialization code for the offset
     * and period fields.
     * 
     * This method will also populate the global _lf_timer_triggers array, which is
     * used to start all timers at the start of execution.
     * 
     * @param instance A reactor instance.
     */
    private def generateTimerInitializations(ReactorInstance instance) {
        for (timer : instance.timers) {
            if (currentFederate.contains(timer.getDefinition())) {
                if (!timer.isStartup) {
                    initializeTriggerObjects.pr(CTimerGenerator.generateInitializer(timer))
                    timerCount += currentFederate.numRuntimeInstances(timer.parent);
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
     def processProtoFile(String filename, CancelIndicator cancelIndicator) {
        val protoc = commandFactory.createCommand(
            "protoc-c",
            #['''--c_out=«this.fileConfig.getSrcGenPath»''', filename],
            fileConfig.srcPath)
        if (protoc === null) {
            errorReporter.reportError("Processing .proto files requires proto-c >= 1.3.3.")
            return
        }
        val returnCode = protoc.run(cancelIndicator)
        if (returnCode == 0) {
            val nameSansProto = filename.substring(0, filename.length - 6)
            targetConfig.compileAdditionalSources.add(
                this.fileConfig.getSrcGenPath.resolve(nameSansProto + ".pb-c.c").toString
            )
            
            targetConfig.compileLibraries.add('-l')
            targetConfig.compileLibraries.add('protobuf-c')
            targetConfig.compilerFlags.add('-lprotobuf-c');  
        } else {
            errorReporter.reportError("protoc-c returns error code " + returnCode)
        }
    }
     
    /** 
     * Construct a unique type for the struct of the specified
     * typed variable (port or action) of the specified reactor class.
     * This is required to be the same as the type name returned by
     * {@link variableStructType(TriggerInstance<?>)}.
     * @param variable The variable.
     * @param reactor The reactor class.
     * @return The name of the self struct.
     */
    static def variableStructType(Variable variable, ReactorDecl reactor) {
        '''«reactor.name.toLowerCase»_«variable.name»_t'''
    }

    /** 
     * Construct a unique type for the struct of the specified
     * instance (port or action).
     * This is required to be the same as the type name returned by
     * {@link variableStructType(Variable, ReactorDecl)}.
     * @param portOrAction The port or action instance.
     * @return The name of the self struct.
     */
    static def variableStructType(TriggerInstance<?> portOrAction) {
        '''«portOrAction.parent.reactorDeclaration.name.toLowerCase»_«portOrAction.name»_t'''
    }

    /**
     * Generates C code to retrieve port->member
     * This function is used for clarity and is called whenever struct is allocated on heap memory.
     * @param portName The name of the port in string
     * @param member The member's name (e.g., is_present)
     * @return Generated code
     */
    def getHeapPortMember(String portName, String member) '''
        «portName»->«member»
    '''
    
    /**
     * Return the operator used to retrieve struct members
     */
    def getStackStructOperator() '''
    .
    '''
    
    /**
     * If tracing is turned on, then generate code that records
     * the full name of the specified reactor instance in the
     * trace table. If tracing is not turned on, do nothing.
     * @param instance The reactor instance.
     */
    private def void generateTraceTableEntries(ReactorInstance instance) {
        if (targetConfig.tracing !== null) {
            initializeTriggerObjects.pr(
                CTracingGenerator.generateTraceTableEntries(instance, currentFederate)
            );
        }
    }
    
    /** 
     * Generate code to instantiate the specified reactor instance and
     * initialize it.
     * @param instance A reactor instance.
     * @param federate A federate instance to conditionally generate code by
     *  contained reactors or null if there are no federates.
     */
    def void generateReactorInstance(ReactorInstance instance) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        initializeTriggerObjects.pr(
                '// ***** Start initializing ' + fullName + ' of class ' + reactorClass.name)
        // Generate the instance self struct containing parameters, state variables,
        // and outputs (the "self" struct).
        initializeTriggerObjects.pr('''
            «CUtil.reactorRefName(instance)»[«CUtil.runtimeIndex(instance)»] = new_«reactorClass.name»();
        ''')
        // Generate code to initialize the "self" struct in the
        // _lf_initialize_trigger_objects function.
        generateTraceTableEntries(instance)
        generateReactorInstanceExtension(instance)
        generateParameterInitialization(instance)
        initializeOutputMultiports(instance)
        initializeInputMultiports(instance)
        recordStartupAndShutdown(instance);

        // Next, initialize the "self" struct with state variables.
        // These values may be expressions that refer to the parameter values defined above.        
        generateStateVariableInitializations(instance);

        // Generate trigger objects for the instance.
        generateTimerInitializations(instance);
        generateActionInitializations(instance);
        generateInitializeActionToken(instance);
        generateSetDeadline(instance);
        generateModeStructure(instance);

        // Recursively generate code for the children.
        for (child : instance.children) {
            if (currentFederate.contains(child)) {
                // If this reactor is a placeholder for a bank of reactors, then generate
                // an array of instances of reactors and create an enclosing for loop.
                // Need to do this for each of the builders into which the code writes.
                startTimeStep.startScopedBlock(child, currentFederate, isFederated, true);
                initializeTriggerObjects.startScopedBlock(child, currentFederate, isFederated, true);
                generateReactorInstance(child);
                initializeTriggerObjects.endScopedBlock();
                startTimeStep.endScopedBlock();
            }
        }
        
        // If this program is federated with centralized coordination and this reactor
        // instance is a federate, then check
        // for outputs that depend on physical actions so that null messages can be
        // sent to the RTI.
        if (isFederatedAndCentralized && instance.parent === main) {
            val outputDelayMap = currentFederate.findOutputsConnectedToPhysicalActions(instance)
            var minDelay = TimeValue.MAX_VALUE;
            var outputFound = null as Output;
            for (output : outputDelayMap.keySet) {
                val outputDelay = outputDelayMap.get(output)
                if (outputDelay.isEarlierThan(minDelay)) {
                    minDelay = outputDelay
                    outputFound = output
                }
            }
            if (minDelay != TimeValue.MAX_VALUE) {
                // Unless silenced, issue a warning.
                if (targetConfig.coordinationOptions.advance_message_interval === null) {
                    errorReporter.reportWarning(outputFound, '''
                            Found a path from a physical action to output for reactor "«instance.name»". 
                            The amount of delay is «minDelay.toString()».
                            With centralized coordination, this can result in a large number of messages to the RTI.
                            Consider refactoring the code so that the output does not depend on the physical action,
                            or consider using decentralized coordination. To silence this warning, set the target
                            parameter coordination-options with a value like {advance-message-interval: 10 msec}"''')
                }
                initializeTriggerObjects.pr('''
                    _fed.min_delay_from_physical_action_to_federate_output = «minDelay.timeInTargetLanguage»;
                ''')
            }
        }
        
        // For this instance, define what must be done at the start of
        // each time step. This sets up the tables that are used by the
        // _lf_start_time_step() function in reactor_common.c.
        // Note that this function is also run once at the end
        // so that it can deallocate any memory.
        generateStartTimeStep(instance)

        initializeTriggerObjects.pr("//***** End initializing " + fullName)
    }

    /**
     * For each action of the specified reactor instance, generate initialization code
     * for the offset and period fields. 
     * @param instance The reactor.
     */
    private def generateActionInitializations(ReactorInstance instance) {
        initializeTriggerObjects.pr(CActionGenerator.generateInitializers(instance, currentFederate));
    }
        
    /**
     * Initialize actions by creating a lf_token_t in the self struct.
     * This has the information required to allocate memory for the action payload.
     * Skip any action that is not actually used as a trigger.
     * @param reactor The reactor containing the actions.
     */
    private def void generateInitializeActionToken(ReactorInstance reactor) {
        for (action : reactor.actions) {
            // Skip this step if the action is not in use. 
            if (action.parent.triggers.contains(action) 
                && currentFederate.contains(action.definition)
            ) {
                var type = action.definition.inferredType
                var payloadSize = "0"
                if (!type.isUndefined) {
                    var String typeStr = types.getTargetType(type)
                    if (CUtil.isTokenType(type, types)) {
                        typeStr = CUtil.rootType(typeStr)
                    }
                    if (typeStr !== null && !typeStr.equals("") && !typeStr.equals("void")) {
                        payloadSize = '''sizeof(«typeStr»)'''
                    }    
                }
            
                var selfStruct = CUtil.reactorRef(action.parent);
                initializeTriggerObjects.pr(
                    CActionGenerator.generateTokenInitializer(
                        selfStruct, action.name, payloadSize
                    )
                )
                startTimeStepTokens += currentFederate.numRuntimeInstances(action.parent);
            }
        }
    }
    
    /**
     * Generate code that is executed while the reactor instance is being initialized.
     * This is provided as an extension point for subclasses.
     * Normally, the reactions argument is the full list of reactions,
     * but for the top-level of a federate, will be a subset of reactions that
     * is relevant to the federate.
     * @param instance The reactor instance.
     * @param reactions The reactions of this instance.
     */
    def void generateReactorInstanceExtension(ReactorInstance instance) {
        // Do nothing
    }
    
    /**
     * Generate code that initializes the state variables for a given instance.
     * Unlike parameters, state variables are uniformly initialized for all instances
     * of the same reactor.
     * @param instance The reactor class instance
     * @return Initialization code fore state variables of instance
     */
    def generateStateVariableInitializations(ReactorInstance instance) {
        val reactorClass = instance.definition.reactorClass
        val selfRef = CUtil.reactorRef(instance)
        for (stateVar : reactorClass.toDefinition.allStateVars) {
            if (stateVar.initialized) {
                var mode = stateVar.eContainer() instanceof Mode ? 
                    instance.lookupModeInstance(stateVar.eContainer() as Mode) :
                    instance.getMode(false);
                initializeTriggerObjects.pr(CStateGenerator.generateInitializer(
                    instance, 
                    selfRef,
                    stateVar,
                    mode,
                    types,
                    modalStateResetCount
                ))
                if (mode !== null) {
                    modalStateResetCount++
                }
            }
        }
    }
    
    /**
     * Generate code to set the deadline field of the reactions in the
     * specified reactor instance.
     * @param instance The reactor instance.
     */
    private def void generateSetDeadline(ReactorInstance instance) {
        for (reaction : instance.reactions) {
            if (reaction.declaredDeadline !== null
                && currentFederate.contains(reaction.getDefinition())
            ) {
                var deadline = reaction.declaredDeadline.maxDelay
                val selfRef = '''«CUtil.reactorRef(reaction.parent)»->_lf__reaction_«reaction.index»'''
                initializeTriggerObjects.pr('''
                    «selfRef».deadline = «deadline.timeInTargetLanguage»;
                ''')
            }
        }
    }
        
    /**
     * Generate code to initialize modes.
     * @param instance The reactor instance.
     */
    private def void generateModeStructure(ReactorInstance instance) {
        val parentMode = instance.getMode(false);
        val nameOfSelfStruct = CUtil.reactorRef(instance);
        // If this instance is enclosed in another mode
        if (parentMode !== null) {
            val parentModeRef = '''&«CUtil.reactorRef(parentMode.parent)»->_lf__modes[«parentMode.parent.modes.indexOf(parentMode)»]'''
            initializeTriggerObjects.pr("// Setup relation to enclosing mode")

            // If this reactor does not have its own modes, all reactions must be linked to enclosing mode
            if (instance.modes.empty) {
                for (reaction : instance.reactions.indexed) {
                    initializeTriggerObjects.pr('''
                        «CUtil.reactorRef(reaction.value.parent)»->_lf__reaction_«reaction.key».mode = «parentModeRef»;
                    ''')
                }
            } else { // Otherwise, only reactions outside modes must be linked and the mode state itself gets a parent relation
                initializeTriggerObjects.pr('''
                    ((self_base_t*)«nameOfSelfStruct»)->_lf__mode_state.parent_mode = «parentModeRef»;
                ''')
                for (reaction : instance.reactions.filter[it.getMode(true) === null]) {
                    initializeTriggerObjects.pr('''
                        «CUtil.reactorRef(reaction.parent)»->_lf__reaction_«instance.reactions.indexOf(reaction)».mode = «parentModeRef»;
                    ''')
                }
            }
        }
        // If this reactor has modes, register for mode change handling
        if (!instance.modes.empty) {
            initializeTriggerObjects.pr('''
                // Register for transition handling
                _lf_modal_reactor_states[«modalReactorCount++»] = &((self_base_t*)«nameOfSelfStruct»)->_lf__mode_state;
            ''')
        }
    }
    
    /**
     * Generate runtime initialization code for parameters of a given reactor instance
     * @param instance The reactor instance.
     */
    def void generateParameterInitialization(ReactorInstance instance) {
        var selfRef = CUtil.reactorRef(instance)
        for (parameter : instance.parameters) {
            // NOTE: we now use the resolved literal value. For better efficiency, we could
            // store constants in a global array and refer to its elements to avoid duplicate
            // memory allocations.
            // NOTE: If the parameter is initialized with a static initializer for an array
            // or struct (the initialization expression is surrounded by { ... }), then we
            // have to declare a static variable to ensure that the memory is put in data space
            // and not on the stack.
            // FIXME: Is there a better way to determine this than the string comparison? 
            val initializer = CParameterGenerator.getInitializer(parameter);
            if (initializer.startsWith("{")) {
                val temporaryVariableName = parameter.uniqueID
                initializeTriggerObjects.pr('''
                    static «types.getVariableDeclaration(parameter.type, temporaryVariableName, true)» = «initializer»;
                    «selfRef»->«parameter.name» = «temporaryVariableName»;
                ''')
            } else {
                initializeTriggerObjects.pr('''
                    «selfRef»->«parameter.name» = «initializer»;
                ''')
            }
        }
    }
    
    /**
     * Generate code that mallocs memory for any output multiports.
     * @param reactor The reactor instance.
     */
    def initializeOutputMultiports(ReactorInstance reactor) {
        val reactorSelfStruct = CUtil.reactorRef(reactor);
        for (output : reactor.outputs) {
            val portRefName = CUtil.portRefName(output);
            // If the port is a multiport, create an array.
            if (output.isMultiport) {
                val portStructType = variableStructType(output);
                initializeTriggerObjects.pr('''
                    «portRefName»_width = «output.width»;
                    // Allocate memory for multiport output.
                    «portRefName» = («portStructType»*)_lf_allocate(
                            «output.width», sizeof(«portStructType»),
                            &«reactorSelfStruct»->base.allocations); 
                    «portRefName»_pointers = («portStructType»**)_lf_allocate(
                            «output.width», sizeof(«portStructType»*),
                            &«reactorSelfStruct»->base.allocations); 
                    // Assign each output port pointer to be used in
                    // reactions to facilitate user access to output ports
                    for(int i=0; i < «output.width»; i++) {
                         «portRefName»_pointers[i] = &(«portRefName»[i]);
                    }
                ''')
            } else {
                initializeTriggerObjects.pr('''
                    // width of -2 indicates that it is not a multiport.
                    «CUtil.portRefName(output)»_width = -2;
                ''')
            }            
        }
    }
    
    /**
     * Allocate memory for inputs.
     * @param reactor The reactor.
     */
    def initializeInputMultiports(ReactorInstance reactor) {
        val reactorSelfStruct = CUtil.reactorRef(reactor); 
        for (input : reactor.inputs) {
            val portRefName = CUtil.portRefName(input)
            // If the port is a multiport, create an array.
            if (input.isMultiport) {
                initializeTriggerObjects.pr('''
                    «portRefName»_width = «input.width»;
                    // Allocate memory for multiport inputs.
                    «portRefName» = («variableStructType(input)»**)_lf_allocate(
                            «input.width», sizeof(«variableStructType(input)»*),
                            &«reactorSelfStruct»->base.allocations); 
                    // Set inputs by default to an always absent default input.
                    for (int i = 0; i < «input.width»; i++) {
                        «portRefName»[i] = &«CUtil.reactorRef(reactor)»->_lf_default__«input.name»;
                    }
                ''')
            } else {
                initializeTriggerObjects.pr('''
                    // width of -2 indicates that it is not a multiport.
                    «portRefName»_width = -2;
                ''')
            }
        }
    }
    
    /**
     * If the argument is a multiport, return a string that is a valid
     * C expression consisting of an (optional) integer added to any number of
     * parameter references on the specified self struct.
     * @param port The port.
     * @param contained If the port belongs to a contained reactor, then
     *  the contained reactor's instantiation. Otherwise, null.
     * @param reactorInstance The reactor referring to this port. If null, "self" will be used
     *  to reference the reactor.
     * @return The width expression for a multiport or an empty string if it is
     *  not a multiport.
     */
    protected def String multiportWidthSpecInC(Port port, Instantiation contained, ReactorInstance reactorInstance) {
        var result = new StringBuilder()
        var count = 0
        var selfRef = "self"
        if (reactorInstance !== null) { 
            if (contained !== null) {
                // Caution: If port belongs to a contained reactor, the self struct needs to be that
                // of the contained reactor instance, not this container
                selfRef = CUtil.reactorRef(reactorInstance.getChildReactorInstance(contained))
            } else {
                selfRef =CUtil.reactorRef(reactorInstance);
            }
        }
        if (port.widthSpec !== null) {
            if (!port.widthSpec.ofVariableLength) {
                for (term : port.widthSpec.terms) {
                    if (term.parameter !== null) {
                        result.append(selfRef)
                        result.append('->')
                        result.append(term.parameter.name)
                    } else {
                        count += term.width
                    }
                }
            }
        }
        if (count > 0) {
            if (result.length > 0) {
                result.append(' + ')
            }
            result.append(count)
        }
        return result.toString
    }

    override getTargetTypes() {
        return types;
    }

    // //////////////////////////////////////////
    // // Protected methods.

    // Perform set up that does not generate code
    protected def void setUpParameters(LFGeneratorContext context) {
        accommodatePhysicalActionsIfPresent()
        targetConfig.compileDefinitions.put("LOG_LEVEL", targetConfig.logLevel.ordinal.toString);
        targetConfig.compileAdditionalSources.add("ctarget.c");
        targetConfig.compileAdditionalSources.add("core" + File.separator + "mixed_radix.c");
        setCSpecificDefaults(context)
        // Create the main reactor instance if there is a main reactor.
        createMainReactorInstance();        
        // If there are federates, copy the required files for that.
        // Also, create the RTI C file and the launcher script.
        if (isFederated) {
            // Handle target parameters.
            // If the program is federated, then ensure that threading is enabled.
            targetConfig.threading = true
            // Convey to the C runtime the required number of worker threads to 
            // handle network input control reactions.
            targetConfig.compileDefinitions.put(
                "WORKERS_NEEDED_FOR_FEDERATE", 
                CUtil.minThreadsToHandleInputPorts(federates).toString
            );
        }
        if (hasModalReactors) {
            // So that each separate compile knows about modal reactors, do this:
            targetConfig.compileDefinitions.put("MODAL_REACTORS", "");
        }
        if (targetConfig.threading) {
            pickScheduler();
        }
        pickCompilePlatform();
        parseTargetParameters();
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    override generateDelayBody(Action action, VarRef port) { 
        val ref = ASTUtils.generateVarRef(port);
        // Note that the action.type set by the base class is actually
        // the port type.
        if (CUtil.isTokenType(action.inferredType, types)) {
            '''
            if («ref»->is_present) {
                // Put the whole token on the event queue, not just the payload.
                // This way, the length and element_size are transported.
                schedule_token(«action.name», 0, «ref»->token);
            }
            '''
        } else {
            '''
            schedule_copy(«action.name», 0, &«ref»->value, 1);  // Length is 1.
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
        val outputName = ASTUtils.generateVarRef(port)
        if (CUtil.isTokenType(action.inferredType, types)) {
            // Forward the entire token and prevent freeing.
            // Increment the ref_count because it will be decremented
            // by both the action handling code and the input handling code.
            '''
            «DISABLE_REACTION_INITIALIZATION_MARKER»
            self->_lf_«outputName».value = («types.getTargetType(action)»)self->_lf__«action.name».token->value;
            self->_lf_«outputName».token = (lf_token_t*)self->_lf__«action.name».token;
            ((lf_token_t*)self->_lf__«action.name».token)->ref_count++;
            self->_lf_«outputName».is_present = true;
            '''
        } else {
            '''
            SET(«outputName», «action.name»->value);
            '''
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
        return CNetworkGenerator.generateNetworkReceiverBody(
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
            types
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
        return CNetworkGenerator.generateNetworkSenderBody(
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
            types,
            targetConfig.coordination
        )
    }
    
    /**
     * Generate code for the body of a reaction that decides whether the trigger for the given
     * port is going to be present or absent for the current logical time.
     * This reaction is put just before the first reaction that is triggered by the network
     * input port "port" or has it in its sources. If there are only connections to contained 
     * reactors, in the top-level reactor.
     * 
     * @param port The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     */
    override generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP
    ) {
        return CNetworkGenerator.generateNetworkInputControlReactionBody(
            receivingPortID,
            maxSTP,
            isFederatedAndDecentralized
        )
    }

    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     * 
     * @param port The port to generate the control reaction for
     * @param portID The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex The bank index of the sending federate, if it is in a bank.
     * @param sendingChannelIndex The channel if a multiport
     * @param delay The delay value imposed on the connection using after
     */
    override generateNetworkOutputControlReactionBody(
        VarRef port,
        int portID,
        int receivingFederateID,
        int sendingBankIndex,
        int sendingChannelIndex,
        Delay delay
    ) {
        return CNetworkGenerator.generateNetworkOutputControlReactionBody(
            port,
            portID,
            receivingFederateID,
            sendingBankIndex,
            sendingChannelIndex,
            delay
        );
    }
    
    /**
     * Add necessary code to the source and necessary build supports to
     * enable the requested serializer in 'enabledSerializers'
     */  
    override enableSupportForSerializationIfApplicable(CancelIndicator cancelIndicator) {
        if (!targetConfig.protoFiles.isNullOrEmpty) {
            // Enable support for proto serialization
            enabledSerializers.add(SupportedSerializers.PROTO)
        }
        for (serializer : enabledSerializers) {
            switch (serializer) {
                case SupportedSerializers.NATIVE: {
                    // No need to do anything at this point.
                }
                case SupportedSerializers.PROTO: {
                    // Handle .proto files.
                    for (file : targetConfig.protoFiles) {
                        this.processProtoFile(file, cancelIndicator)
                        val dotIndex = file.lastIndexOf('.')
                        var rootFilename = file
                        if (dotIndex > 0) {
                            rootFilename = file.substring(0, dotIndex)
                        }
                        code.pr('#include "' + rootFilename + '.pb-c.h"')
                    }
                }
                case SupportedSerializers.ROS2: {
                    if(!CCppMode) {
                        throw new UnsupportedOperationException(
                            "To use the ROS 2 serializer, please use the CCpp target."
                            )
                    }
                    if (targetConfig.useCmake === false) {
                        throw new UnsupportedOperationException(
                            "Invalid target property \"cmake: false\"" +
                            "To use the ROS 2 serializer, please use the CMake build system (default)"
                            )
                    }
                    val ROSSerializer = new FedROS2CPPSerialization();
                    code.pr(ROSSerializer.generatePreambleForSupport.toString);
                    cMakeExtras = '''
                        «cMakeExtras»
                        «ROSSerializer.generateCompilerExtensionForSupport»
                    '''
                }
                
            }
        }
    }

    /** 
     * Generate code that needs to appear at the top of the generated
     * C file, such as #define and #include statements.
     */
    def void generateDirectives() {
        code.prComment("Code generated by the Lingua Franca compiler from:")
        code.prComment("file:/" + FileUtil.toUnixString(fileConfig.srcFile))
        code.pr(CPreambleGenerator.generateDefineDirectives(
            targetConfig,
            federates.size,
            isFederated, 
            fileConfig.srcGenPath,
            clockSyncIsOn(),
            hasModalReactors
        ))

        code.pr(CPreambleGenerator.generateIncludeStatements(
            targetConfig,
            isFederated
        ))
        
        // Do this after the above includes so that the preamble can
        // call built-in functions
        
    }

    /**
     * Generate top-level preamble code.
     */
    protected def void generateTopLevelPreambles() {
        if (this.mainDef !== null) {
            val mainModel = this.mainDef.reactorClass.toDefinition.eContainer as Model
            for (p : mainModel.preambles) {
                code.pr(p.code.toText)
            }
        }
    }

    /**
     * Parse the target parameters and set flags to the runCommand
     * accordingly.
     */
    def parseTargetParameters() {
        if (targetConfig.fastMode) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add(topLevelName)
            }
            runCommand.add("-f")
            runCommand.add("true")
        }
        if (targetConfig.keepalive) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add(topLevelName)
            }
            runCommand.add("-k")
            runCommand.add("true")
        }
        if (targetConfig.timeout !== null) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add(topLevelName)
            }
            runCommand.add("-o")
            runCommand.add(targetConfig.timeout.magnitude.toString)
            runCommand.add(targetConfig.timeout.unit.canonicalName)
        }
        
    }

    // Regular expression pattern for compiler error messages with resource
    // and line number information. The first match will a resource URI in the
    // form of "file:/path/file.lf". The second match will be a line number.
    // The third match is a character position within the line.
    // The fourth match will be the error message.
    static final Pattern compileErrorPattern = Pattern.compile(
        "^(file:(?<path>.*)):(?<line>[0-9]+):(?<column>[0-9]+):(?<message>.*)$"
    );
    
    /** Given a line of text from the output of a compiler, return
     *  an instance of ErrorFileAndLine if the line is recognized as
     *  the first line of an error message. Otherwise, return null.
     *  @param line A line of output from a compiler or other external
     *   tool that might generate errors.
     *  @return If the line is recognized as the start of an error message,
     *   then return a class containing the path to the file on which the
     *   error occurred (or null if there is none), the line number (or the
     *   string "1" if there is none), the character position (or the string
     *   "0" if there is none), and the message (or an empty string if there
     *   is none).
     */
    override parseCommandOutput(String line) {
        val matcher = compileErrorPattern.matcher(line)
        if (matcher.find()) {
            val result = new ErrorFileAndLine()
            result.filepath = matcher.group("path")
            result.line = matcher.group("line")
            result.character = matcher.group("column")
            result.message = matcher.group("message")
            
            if (!result.message.toLowerCase.contains("error:")) {
                result.isError = false
            }
            return result
        }
        return null as ErrorFileAndLine
    }
    
    ////////////////////////////////////////////
    //// Private methods.        
    protected static var DISABLE_REACTION_INITIALIZATION_MARKER
        = '// **** Do not include initialization code in this reaction.'

    public static var UNDEFINED_MIN_SPACING = -1
    
    /**
     * Extra lines that need to go into the generated CMakeLists.txt.
     */
    var String cMakeExtras = "";
    
       
    /** Returns the Target enum for this generator */
    override getTarget() {
        return Target.C
    }

    override getNetworkBufferType() '''uint8_t*'''

    
    override generateDelayGeneric() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    ////////////////////////////////////////////////////////////
    //// Private methods
    
    /**
     * If a main or federted reactor has been declared, create a ReactorInstance
     * for this top level. This will also assign levels to reactions, then,
     * if the program is federated, perform an AST transformation to disconnect
     * connections between federates.
     */
    private def void createMainReactorInstance() {
        if (this.mainDef !== null) {
            if (this.main === null) {
                // Recursively build instances. This is done once because
                // it is the same for all federates.
                this.main = new ReactorInstance(mainDef.reactorClass.toDefinition, errorReporter, 
                    this.unorderedReactions)
                if (this.main.assignLevels().nodeCount > 0) {
                    errorReporter.reportError("Main reactor has causality cycles. Skipping code generation.");
                    return;
                }
                // Force reconstruction of dependence information.
                if (isFederated) {
                    // Avoid compile errors by removing disconnected network ports.
                    // This must be done after assigning levels.  
                    removeRemoteFederateConnectionPorts(main);
                    // There will be AST transformations that invalidate some info
                    // cached in ReactorInstance.
                    this.main.clearCaches(false);                    
                }
            }   
        }
    }
    
    /**
     * Generate an array of self structs for the reactor
     * and one for each of its children.
     * @param r The reactor instance.
     */
    private def void generateSelfStructs(ReactorInstance r) {
        if (!currentFederate.contains(r)) return;
        // FIXME: For federated execution, if the reactor is a bank, then
        // it may be that only one of the bank members is in the federate,
        // but this creates an array big enough to hold all bank members.
        // Fixing this will require making the functions in CUtil that
        // create references to the runtime instances aware of this exception.
        // For now, we just create a larger array than needed.
        initializeTriggerObjects.pr('''
            «CUtil.selfType(r)»* «CUtil.reactorRefName(r)»[«r.totalWidth»];
        ''')
        for (child : r.children) {
            generateSelfStructs(child);
        }
    }
    
    ////////////////////////////////////////////
    //// Protected fields
    
    /** The main place to put generated code. */
    protected var code = new CodeBuilder();

    /** The current federate for which we are generating code. */
    protected var currentFederate = null as FederateInstance;

    /** Place to collect code to initialize the trigger objects for all reactor instances. */
    protected var initializeTriggerObjects = new CodeBuilder()

    /** 
     * Count of the number of is_present fields of the self struct that
     * need to be reinitialized in _lf_start_time_step().
     */
    protected var startTimeStepIsPresentCount = 0

    ////////////////////////////////////////////
    //// Private fields
    
    /** The command to run the generated code if specified in the target directive. */
    var runCommand = new ArrayList<String>();

    /** Place to collect code to execute at the start of a time step. */
    var startTimeStep = new CodeBuilder()
        
    /** Count of the number of token pointers that need to have their
     *  reference count decremented in _lf_start_time_step().
     */
    var startTimeStepTokens = 0

    var timerCount = 0
    var startupReactionCount = 0
    var shutdownReactionCount = 0
    var modalReactorCount = 0
    var modalStateResetCount = 0
    
    // Indicate whether the generator is in Cpp mode or not
    var boolean CCppMode = false;

    var CTypes types;
}
