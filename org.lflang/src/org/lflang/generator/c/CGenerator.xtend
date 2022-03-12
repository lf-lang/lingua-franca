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
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.regex.Pattern;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
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
import org.lflang.TargetProperty.LogLevel;
import org.lflang.TimeValue;
import org.lflang.federated.CGeneratorExtension;
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
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
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
import org.lflang.generator.c.InteractingContainedReactors;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
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
import org.lflang.util.XtendUtil;

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

            generatePreamble()
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
                if (timerCount > 0) {
                    code.pr('''
                        // Array of pointers to timer triggers to be scheduled in _lf_initialize_timers().
                        trigger_t* _lf_timer_triggers[«timerCount»];
                        int _lf_timer_triggers_size = «timerCount»;
                    ''')
                } else {
                    code.pr('''
                        // Array of pointers to timer triggers to be scheduled in _lf_initialize_timers().
                        trigger_t** _lf_timer_triggers = NULL;
                        int _lf_timer_triggers_size = 0;
                    ''')
                }
                
                // If there are startup reactions, store them in an array.
                if (startupReactionCount > 0) {
                    code.pr('''
                        // Array of pointers to reactions to be scheduled in _lf_trigger_startup_reactions().
                        reaction_t* _lf_startup_reactions[«startupReactionCount»];
                        int _lf_startup_reactions_size = «startupReactionCount»;
                    ''')
                } else {
                    code.pr('''
                        // Array of pointers to reactions to be scheduled in _lf_trigger_startup_reactions().
                        reaction_t** _lf_startup_reactions = NULL;
                        int _lf_startup_reactions_size = 0;
                    ''')
                }
                
                // If there are shutdown reactions, create a table of triggers.
                if (shutdownReactionCount > 0) {
                    code.pr('''
                        // Array of pointers to shutdown triggers.
                        reaction_t* _lf_shutdown_reactions[«shutdownReactionCount»];
                        int _lf_shutdown_reactions_size = «shutdownReactionCount»;
                    ''')
                } else {
                    code.pr('''
                        // Empty array of pointers to shutdown triggers.
                        reaction_t** _lf_shutdown_reactions = NULL;
                        int _lf_shutdown_reactions_size = 0;
                    ''')
                }
                
                // If there are modes, create a table of mode state to be checked for transitions.
                if (hasModalReactors) {
                    code.pr('''
                        // Array of pointers to mode states to be handled in _lf_handle_mode_changes().
                        reactor_mode_state_t* _lf_modal_reactor_states[«modalReactorCount»];
                        int _lf_modal_reactor_states_size = «modalReactorCount»;
                    ''')
                    if (modalStateResetCount > 0) {
                        code.pr('''
                            // Array of reset data for state variables nested in modes. Used in _lf_handle_mode_changes().
                            mode_state_variable_reset_data_t _lf_modal_state_reset[«modalStateResetCount»];
                            int _lf_modal_state_reset_size = «modalStateResetCount»;
                        ''')
                    }
                }
                
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
                generateInitializeTriggerObjects(federate);

                // Generate function to trigger startup reactions for all reactors.
                generateTriggerStartupReactions();

                // Generate function to schedule timers for all reactors.
                if (timerCount > 0) {
                    code.pr('''
                       void _lf_initialize_timers() {
                           for (int i = 0; i < _lf_timer_triggers_size; i++) {
                               if (_lf_timer_triggers[i] != NULL) {
                                   _lf_initialize_timer(_lf_timer_triggers[i]);
                               }
                           }
                       }
                    ''')
                } else {
                    code.pr('''
                        void _lf_initialize_timers() {}
                    ''')
                }

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
                code.pr('''
                    bool _lf_trigger_shutdown_reactions() {                          
                        for (int i = 0; i < _lf_shutdown_reactions_size; i++) {
                            if (_lf_shutdown_reactions[i] != NULL) {
                                _lf_trigger_reaction(_lf_shutdown_reactions[i], -1);
                            }
                        }
                        // Return true if there are shutdown reactions.
                        return (_lf_shutdown_reactions_size > 0);
                    }
                ''')
                
                // Generate an empty termination function for non-federated
                // execution. For federated execution, an implementation is
                // provided in federate.c.  That implementation will resign
                // from the federation and close any open sockets.
                if (!isFederated) {
                    code.pr("void terminate_execution() {}");
                }
                
                if (hasModalReactors) {
                    // Generate mode change detection
                    code.pr('''
                        void _lf_handle_mode_changes() {
                            _lf_process_mode_changes(_lf_modal_reactor_states, _lf_modal_reactor_states_size, «modalStateResetCount > 0 ? "_lf_modal_state_reset" : "NULL"», «modalStateResetCount > 0 ? "_lf_modal_state_reset_size" : 0»);
                        }
                    ''')
                }
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
    
    override transformConflictingConnectionsInModalReactors(Collection<Connection> transform) {
        val factory = LfFactory.eINSTANCE
        for (connection : transform) {
            // Currently only simple transformations are supported
            if (connection.physical || connection.delay !== null || connection.iterated || 
                connection.leftPorts.size > 1 || connection.rightPorts.size > 1
            ) {
                errorReporter.reportError(connection, "Cannot transform connection in modal reactor. Connection uses currently not supported features.");
            } else {
                var reaction = factory.createReaction();
                (connection.eContainer() as Mode).getReactions().add(reaction);
                
                var sourceRef = connection.getLeftPorts().head
                var destRef = connection.getRightPorts().head
                reaction.getTriggers().add(sourceRef);
                reaction.getEffects().add(destRef);
                
                var code = factory.createCode();
                var source = (sourceRef.container !== null ? sourceRef.container.name + "." : "") + sourceRef.variable.name
                var dest = (destRef.container !== null ? destRef.container.name + "." : "") + destRef.variable.name
                code.setBody('''
                    // Generated forwarding reaction for connections with the same destination but located in mutually exclusive modes.
                    SET(«dest», «source»->value);
                ''');
                reaction.setCode(code);
                
                EcoreUtil.remove(connection);
            }
        }
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
     * Generate the _lf_trigger_startup_reactions function.
     */
    private def generateTriggerStartupReactions() {
        code.pr('''
            void _lf_trigger_startup_reactions() {
                «IF startupReactionCount > 0»
                for (int i = 0; i < _lf_startup_reactions_size; i++) {
                    if (_lf_startup_reactions[i] != NULL) {
                        _lf_trigger_reaction(_lf_startup_reactions[i], -1);
                    }
                }
                «ENDIF»
            }
        ''')
    }
    
    /**
     * Generate the _lf_initialize_trigger_objects function for 'federate'.
     */
    private def generateInitializeTriggerObjects(FederateInstance federate) {
        code.pr('''
            void _lf_initialize_trigger_objects() {
        ''')
        code.indent()
        
        // Initialize the LF clock.
        code.pr('''
            // Initialize the _lf_clock
            lf_initialize_clock();
        ''')

        // Initialize tracing if it is enabled
        if (targetConfig.tracing !== null) {
            var traceFileName = topLevelName;
            if (targetConfig.tracing.traceFileName !== null) {
                traceFileName = targetConfig.tracing.traceFileName;
                // Since all federates would have the same name, we need to append the federate name.
                if (isFederated) {
                    traceFileName += "_" + federate.name;
                }
            }
            code.pr('''
                // Initialize tracing
                start_trace("«traceFileName».lft");
            ''') // .lft is for Lingua Franca trace
        }

        // Create the table used to decrement reference counts between time steps.
        if (startTimeStepTokens > 0) {
            // Allocate the initial (before mutations) array of pointers to tokens.
            code.pr('''
                _lf_tokens_with_ref_count_size = «startTimeStepTokens»;
                _lf_tokens_with_ref_count = (token_present_t*)calloc(«startTimeStepTokens», sizeof(token_present_t));
                if (_lf_tokens_with_ref_count == NULL) error_print_and_exit("Out of memory!");
            ''')
        }
        // Create the table to initialize is_present fields to false between time steps.
        if (startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to _is_present fields.
            code.pr('''
                // Create the array that will contain pointers to is_present fields to reset on each step.
                _lf_is_present_fields_size = «startTimeStepIsPresentCount»;
                _lf_is_present_fields = (bool**)calloc(«startTimeStepIsPresentCount», sizeof(bool*));
                if (_lf_is_present_fields == NULL) error_print_and_exit("Out of memory!");
                _lf_is_present_fields_abbreviated = (bool**)calloc(«startTimeStepIsPresentCount», sizeof(bool*));
                if (_lf_is_present_fields_abbreviated == NULL) error_print_and_exit("Out of memory!");
                _lf_is_present_fields_abbreviated_size = 0;
            ''')
        }

        // Allocate the memory for triggers used in federated execution
        code.pr(CGeneratorExtension.allocateTriggersForFederate(federate, this, startTimeStepIsPresentCount));

        code.pr(initializeTriggerObjects.toString)

        // Assign appropriate pointers to the triggers
        // FIXME: For python target, almost surely in the wrong place.
        code.pr(CGeneratorExtension.initializeTriggerForControlReactions(this.main, federate, this).toString());

        var reactionsInFederate = main.reactions.filter[ 
                r | return currentFederate.contains(r.definition);
        ];

        deferredInitialize(main, reactionsInFederate)
        
        deferredInitializeNonNested(main, reactionsInFederate)

        // Next, for every input port, populate its "self" struct
        // fields with pointers to the output port that sends it data.
        deferredConnectInputsToOutputs(main)

        // Put the code here to set up the tables that drive resetting is_present and
        // decrementing reference counts between time steps. This code has to appear
        // in _lf_initialize_trigger_objects() after the code that makes connections
        // between inputs and outputs.
        code.pr(startTimeStep.toString());

        setReactionPriorities(main, code)

        initializeFederate(federate)
        
        initializeScheduler();
        
        code.unindent()
        code.pr("}\n")
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
     * If the number of federates is greater than one, then generate the code
     * that initializes global variables that describe the federate.
     * @param federate The federate instance.
     */
    protected def void initializeFederate(FederateInstance federate) {
        if (isFederated) {
            code.pr('''
                // ***** Start initializing the federated execution. */
            ''')            
            code.pr('''
                // Initialize the socket mutex
                lf_mutex_init(&outbound_socket_mutex);
                lf_cond_init(&port_status_changed);
            ''')
            
            if (isFederatedAndDecentralized) {
                val reactorInstance = main.getChildReactorInstance(federate.instantiation)
                for (param : reactorInstance.parameters) {
                    if (param.name.equalsIgnoreCase("STP_offset") && param.type.isTime) {
                        val stp = param.getInitialValue().get(0).getLiteralTimeValue
                        if (stp !== null) {                        
                            code.pr('''
                                set_stp_offset(«stp.timeInTargetLanguage»);
                            ''')
                        }
                    }
                }
            }
            
            // Set indicator variables that specify whether the federate has
            // upstream logical connections.
            if (federate.dependsOn.size > 0) {
                code.pr('_fed.has_upstream  = true;')
            }
            if (federate.sendsTo.size > 0) {
                code.pr('_fed.has_downstream = true;')
            }
            // Set global variable identifying the federate.
            code.pr('''_lf_my_fed_id = «federate.id»;''');
            
            // We keep separate record for incoming and outgoing p2p connections to allow incoming traffic to be processed in a separate
            // thread without requiring a mutex lock.
            val numberOfInboundConnections = federate.inboundP2PConnections.length;
            val numberOfOutboundConnections  = federate.outboundP2PConnections.length;
            
            code.pr('''
                _fed.number_of_inbound_p2p_connections = «numberOfInboundConnections»;
                _fed.number_of_outbound_p2p_connections = «numberOfOutboundConnections»;
            ''')
            if (numberOfInboundConnections > 0) {
                code.pr('''
                    // Initialize the array of socket for incoming connections to -1.
                    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
                        _fed.sockets_for_inbound_p2p_connections[i] = -1;
                    }
                ''')                    
            }
            if (numberOfOutboundConnections > 0) {                        
                code.pr('''
                    // Initialize the array of socket for outgoing connections to -1.
                    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
                        _fed.sockets_for_outbound_p2p_connections[i] = -1;
                    }
                ''')                    
            }

            // If a test clock offset has been specified, insert code to set it here.
            if (targetConfig.clockSyncOptions.testOffset !== null) {
                code.pr('''
                    set_physical_clock_offset((1 + «federate.id») * «targetConfig.clockSyncOptions.testOffset.toNanoSeconds»LL);
                ''')
            }
            
            code.pr('''
                // Connect to the RTI. This sets _fed.socket_TCP_RTI and _lf_rti_socket_UDP.
                connect_to_rti("«federationRTIProperties.get('host')»", «federationRTIProperties.get('port')»);
            ''');            
            
            // Disable clock synchronization for the federate if it resides on the same host as the RTI,
            // unless that is overridden with the clock-sync-options target property.
            if (targetConfig.clockSync !== ClockSyncMode.OFF
                && (!federationRTIProperties.get('host').toString.equals(federate.host) 
                    || targetConfig.clockSyncOptions.localFederatesOn)
            ) {
                code.pr('''
                    synchronize_initial_physical_clock_with_rti(_fed.socket_TCP_RTI);
                ''')
            }
        
            if (numberOfInboundConnections > 0) {
                code.pr('''
                    // Create a socket server to listen to other federates.
                    // If a port is specified by the user, that will be used
                    // as the only possibility for the server. If not, the port
                    // will start from STARTING_PORT. The function will
                    // keep incrementing the port until the number of tries reaches PORT_RANGE_LIMIT.
                    create_server(«federate.port»);
                    // Connect to remote federates for each physical connection.
                    // This is done in a separate thread because this thread will call
                    // connect_to_federate for each outbound physical connection at the same
                    // time that the new thread is listening for such connections for inbound
                    // physical connections. The thread will live until all connections
                    // have been established.
                    lf_thread_create(&_fed.inbound_p2p_handling_thread_id, handle_p2p_connections_from_federates, NULL);
                ''')
            }

            for (remoteFederate : federate.outboundP2PConnections) {
                code.pr('''connect_to_federate(«remoteFederate.id»);''')
            }
        }
    }
    
    /**
     * Generate code to initialize the scheduler for the threaded C runtime.
     */
    protected def initializeScheduler() {
        if (targetConfig.threading) {
            val numReactionsPerLevel = this.main.assignLevels.getNumReactionsPerLevel();
            code.pr('''
                
                // Initialize the scheduler
                size_t num_reactions_per_level[«numReactionsPerLevel.size»] = 
                    {«numReactionsPerLevel.join(", \\\n")»};
                sched_params_t sched_params = (sched_params_t) {
                                        .num_reactions_per_level = &num_reactions_per_level[0],
                                        .num_reactions_per_level_size = (size_t) «numReactionsPerLevel.size»};
                lf_sched_init(
                    (size_t)_lf_number_of_workers,
                    &sched_params
                );
            ''')
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
        var StringBuilder federatedExtension = new StringBuilder();    
        if (isFederatedAndDecentralized) {
            federatedExtension.append('''
                «types.getTargetTagType» intended_tag;
            ''');
        }
        if (isFederated) {
            federatedExtension.append('''                
                «types.getTargetTimeType» physical_time_of_arrival;
            ''');
        }
        // First, handle inputs.
        for (input : reactor.allInputs) {
            var token = ''
            if (CUtil.isTokenType(input.inferredType, types)) {
                token = '''
                    lf_token_t* token;
                    int length;
                '''
            }
            code.pr(input, '''
                typedef struct {
                    «input.valueDeclaration»
                    bool is_present;
                    int num_destinations;
                    «token»
                    «federatedExtension.toString»
                } «variableStructType(input, decl)»;
            ''')
        }
        // Next, handle outputs.
    for (output : reactor.allOutputs) {
            var token = ''
            if (CUtil.isTokenType(output.inferredType, types)) {
                 token = '''
                    lf_token_t* token;
                    int length;
                 '''
            }
            code.pr(output, '''
                typedef struct {
                    «output.valueDeclaration»
                    bool is_present;
                    int num_destinations;
                    «token»
                    «federatedExtension.toString»
                } «variableStructType(output, decl)»;
            ''')
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
     * For the specified port, return a declaration for port struct to
     * contain the value of the port. A multiport output with width 4 and
     * type int[10], for example, will result in this:
     * ```
     *     int value[10];
     * ```
     * There will be an array of size 4 of structs, each containing this value 
     * array.
     * @param port The port.
     * @return A string providing the value field of the port struct.
     */
    protected def valueDeclaration(Port port) {
        if (port.type === null && target.requiresTypes === true) {
            // This should have been caught by the validator.
            errorReporter.reportError(port, "Port is required to have a type: " + port.name)
            return ''
        }
        // Do not convert to lf_token_t* using lfTypeToTokenType because there
        // will be a separate field pointing to the token.
        return types.getVariableDeclaration(port.inferredType, "value", false) + ";"
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
        generateReactionAndTriggerStructs(body, decl, constructorCode);

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
    
    /**
     * Generate the fields of the self struct and statements for the constructor
     * to create and initialize a reaction_t struct for each reaction in the
     * specified reactor and a trigger_t struct for each trigger (input, action,
     * timer, or output of a contained reactor).
     * @param body The place to put the code for the self struct.
     * @param reactor The reactor.
     * @param constructorCode The place to put the constructor code.
     */
    protected def void generateReactionAndTriggerStructs(
        CodeBuilder body, 
        ReactorDecl decl, 
        CodeBuilder constructorCode 
    ) {
        var reactionCount = 0;
        val reactor = decl.toDefinition
        // Iterate over reactions and create initialize the reaction_t struct
        // on the self struct. Also, collect a map from triggers to the reactions
        // that are triggered by that trigger. Also, collect a set of sources
        // that are read by reactions but do not trigger reactions.
        // Finally, collect a set of triggers and sources that are outputs
        // of contained reactors. 
        val triggerMap = new LinkedHashMap<Variable,LinkedList<Integer>>()
        val sourceSet = new LinkedHashSet<Variable>()
        val outputsOfContainedReactors = new LinkedHashMap<Variable,Instantiation>
        val startupReactions = new LinkedHashSet<Integer>
        val shutdownReactions = new LinkedHashSet<Integer>
        for (reaction : reactor.allReactions) {
            if (currentFederate.contains(reaction)) {
                // Create the reaction_t struct.
                body.pr(reaction, '''reaction_t _lf__reaction_«reactionCount»;''')
                
                // Create the map of triggers to reactions.
                for (trigger : reaction.triggers) {
                    // trigger may not be a VarRef (it could be "startup" or "shutdown").
                    if (trigger instanceof VarRef) {
                        var reactionList = triggerMap.get(trigger.variable)
                        if (reactionList === null) {
                            reactionList = new LinkedList<Integer>()
                            triggerMap.put(trigger.variable, reactionList)
                        }
                        reactionList.add(reactionCount)
                        if (trigger.container !== null) {
                            outputsOfContainedReactors.put(trigger.variable, trigger.container)
                        }
                    }
                    if (trigger.isStartup) {
                        startupReactions.add(reactionCount)
                    }
                    if (trigger.isShutdown) {
                        shutdownReactions.add(reactionCount)
                    }
                }
                // Create the set of sources read but not triggering.
                for (source : reaction.sources) {
                    sourceSet.add(source.variable)
                    if (source.container !== null) {
                        outputsOfContainedReactors.put(source.variable, source.container)
                    }
                }

                var deadlineFunctionPointer = "NULL"
                if (reaction.deadline !== null) {
                    // The following has to match the name chosen in generateReactions
                    val deadlineFunctionName = CReactionGenerator.generateDeadlineFunctionName(decl, reactionCount)
                    deadlineFunctionPointer = "&" + deadlineFunctionName
                }
                
                // Assign the STP handler
                var STPFunctionPointer = "NULL"
                if (reaction.stp !== null) {
                    // The following has to match the name chosen in generateReactions
                    val STPFunctionName = decl.name.toLowerCase + '_STP_function' + reactionCount
                    STPFunctionPointer = "&" + STPFunctionName
                }

                // Set the defaults of the reaction_t struct in the constructor.
                // Since the self struct is allocated using calloc, there is no need to set:
                // self->_lf__reaction_«reactionCount».index = 0;
                // self->_lf__reaction_«reactionCount».chain_id = 0;
                // self->_lf__reaction_«reactionCount».pos = 0;
                // self->_lf__reaction_«reactionCount».status = inactive;
                // self->_lf__reaction_«reactionCount».deadline = 0LL;
                // self->_lf__reaction_«reactionCount».is_STP_violated = false;
                constructorCode.pr(reaction, '''
                    self->_lf__reaction_«reactionCount».number = «reactionCount»;
                    self->_lf__reaction_«reactionCount».function = «CReactionGenerator.generateReactionFunctionName(decl, reactionCount)»;
                    self->_lf__reaction_«reactionCount».self = self;
                    self->_lf__reaction_«reactionCount».deadline_violation_handler = «deadlineFunctionPointer»;
                    self->_lf__reaction_«reactionCount».STP_handler = «STPFunctionPointer»;
                    self->_lf__reaction_«reactionCount».name = "?";
                    «IF reaction.eContainer instanceof Mode»
                        self->_lf__reaction_«reactionCount».mode = &self->_lf__modes[«reactor.modes.indexOf(reaction.eContainer as Mode)»];
                    «ELSE»
                        self->_lf__reaction_«reactionCount».mode = NULL;
                    «ENDIF»
                ''')

            }
            // Increment the reactionCount even if the reaction is not in the federate
            // so that reaction indices are consistent across federates.
            reactionCount++
        }
        
        // Next, create and initialize the trigger_t objects.
        // Start with the timers.
        for (timer : reactor.allTimers) {
            createTriggerT(body, timer, triggerMap, constructorCode)
            // Since the self struct is allocated using calloc, there is no need to set:
            // self->_lf__«timer.name».is_physical = false;
            // self->_lf__«timer.name».drop = false;
            // self->_lf__«timer.name».element_size = 0;
            constructorCode.pr('''
                self->_lf__«timer.name».is_timer = true;
            ''')
            if (isFederatedAndDecentralized) {
                constructorCode.pr('''
                    self->_lf__«timer.name».intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
                ''')
            }
        }
        
        // Handle startup triggers.
        if (startupReactions.size > 0) {
            body.pr('''
                trigger_t _lf__startup;
                reaction_t* _lf__startup_reactions[«startupReactions.size»];
            ''')
            if (isFederatedAndDecentralized) {
                constructorCode.pr('''
                    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
                ''')
            }
            var i = 0
            for (reactionIndex : startupReactions) {
                constructorCode.pr('''
                    self->_lf__startup_reactions[«i++»] = &self->_lf__reaction_«reactionIndex»;
                ''')
            }
            constructorCode.pr('''
                self->_lf__startup.last = NULL;
                self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
                self->_lf__startup.number_of_reactions = «startupReactions.size»;
                self->_lf__startup.is_timer = false;
            ''')
        }
        // Handle shutdown triggers.
        if (shutdownReactions.size > 0) {
            body.pr('''
                trigger_t _lf__shutdown;
                reaction_t* _lf__shutdown_reactions[«shutdownReactions.size»];
            ''')
            if (isFederatedAndDecentralized) {
                constructorCode.pr('''
                    self->_lf__shutdown.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
                ''')
            }
            var i = 0
            for (reactionIndex : shutdownReactions) {
                constructorCode.pr('''
                    self->_lf__shutdown_reactions[«i++»] = &self->_lf__reaction_«reactionIndex»;
                ''')
            }
            constructorCode.pr('''
                self->_lf__shutdown.last = NULL;
                self->_lf__shutdown.reactions = &self->_lf__shutdown_reactions[0];
                self->_lf__shutdown.number_of_reactions = «shutdownReactions.size»;
                self->_lf__shutdown.is_timer = false;
            ''')
        }

        // Next handle actions.
        for (action : reactor.allActions) {
            if (currentFederate.contains(action)) {
                createTriggerT(body, action, triggerMap, constructorCode)
                var isPhysical = "true";
                if (action.origin == ActionOrigin.LOGICAL) {
                    isPhysical = "false";
                }
                var elementSize = "0"
                // If the action type is 'void', we need to avoid generating the code
                // 'sizeof(void)', which some compilers reject.
                var rootType = action.type !== null ? CUtil.rootType(types.getTargetType(action)) : null;
                if (rootType !== null && !rootType.equals("void")) {
                    elementSize = '''sizeof(«rootType»)'''
                }
    
                // Since the self struct is allocated using calloc, there is no need to set:
                // self->_lf__«action.name».is_timer = false;
                constructorCode.pr('''
                    self->_lf__«action.name».is_physical = «isPhysical»;
                    «IF !action.policy.isNullOrEmpty»
                    self->_lf__«action.name».policy = «action.policy»;
                    «ENDIF»
                    self->_lf__«action.name».element_size = «elementSize»;
                ''')
            }
        }

        // Next handle inputs.
        for (input : reactor.allInputs) {
            createTriggerT(body, input, triggerMap, constructorCode)
        }
    }
    
    /**
     * Define the trigger_t object on the self struct, an array of
     * reaction_t pointers pointing to reactions triggered by this variable,
     * and initialize the pointers in the array in the constructor.
     * @param body The place to write the self struct entries.
     * @param variable The trigger variable (Timer, Action, or Input).
     * @param triggerMap A map from Variables to a list of the reaction indices
     *  triggered by the variable.
     * @param constructorCode The place to write the constructor code.
     */
    private def void createTriggerT(
        CodeBuilder body, 
        Variable variable,
        LinkedHashMap<Variable, LinkedList<Integer>> triggerMap,
        CodeBuilder constructorCode
    ) {
        // variable is a port, a timer, or an action.
        body.pr(variable, '''
            trigger_t _lf__«variable.name»;
        ''')
        constructorCode.pr(variable, '''
            self->_lf__«variable.name».last = NULL;
        ''')
        if (isFederatedAndDecentralized) {
            constructorCode.pr(variable, '''
                self->_lf__«variable.name».intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
            ''')
        }
        // Generate the reactions triggered table.
        val reactionsTriggered = triggerMap.get(variable)
        if (reactionsTriggered !== null) {
            body.pr(variable, '''reaction_t* _lf__«variable.name»_reactions[«reactionsTriggered.size»];''')
            var count = 0
            for (reactionTriggered : reactionsTriggered) {
                constructorCode.prSourceLineNumber(variable)
                constructorCode.pr(variable, '''
                    self->_lf__«variable.name»_reactions[«count»] = &self->_lf__reaction_«reactionTriggered»;
                ''')
                count++
            }
            // Set up the trigger_t struct's pointer to the reactions.
            constructorCode.pr(variable, '''
                self->_lf__«variable.name».reactions = &self->_lf__«variable.name»_reactions[0];
                self->_lf__«variable.name».number_of_reactions = «count»;
            ''')
            
            if (isFederated) {
                // Set the physical_time_of_arrival
                constructorCode.pr(variable, '''
                    self->_lf__«variable.name».physical_time_of_arrival = NEVER;
                ''')
            }
        }
        if (variable instanceof Input) {
            val rootType = CUtil.rootType(types.getTargetType(variable))
            // Since the self struct is allocated using calloc, there is no need to set:
            // self->_lf__«input.name».is_timer = false;
            // self->_lf__«input.name».offset = 0LL;
            // self->_lf__«input.name».period = 0LL;
            // self->_lf__«input.name».is_physical = false;
            // self->_lf__«input.name».drop = false;
            // If the input type is 'void', we need to avoid generating the code
            // 'sizeof(void)', which some compilers reject.
            val size = (rootType == 'void') ? '0' : '''sizeof(«rootType»)'''
            constructorCode.pr('''
                self->_lf__«variable.name».element_size = «size»;
            ''')
        
            if (isFederated) {
                body.pr(
                    CGeneratorExtension.createPortStatusFieldForInput(variable, this)                    
                );
            }
        }
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
        val functionName = CReactionGenerator.generateReactionFunctionName(decl, reactionIndex)
        
        
        code.pr('void ' + functionName + '(void* instance_args) {')
        code.indent()
        var body = reaction.code.toText
        
        code.pr(CReactionGenerator.generateInitializationForReaction(body, reaction, decl, reactionIndex, types, errorReporter, mainDef, isFederatedAndDecentralized, target.requiresTypes))
        
        // Code verbatim from 'reaction'
        code.prSourceLineNumber(reaction.code)
        code.pr(body)
        code.unindent()
        code.pr("}")

        // Now generate code for the late function, if there is one
        // Note that this function can only be defined on reactions
        // in federates that have inputs from a logical connection.
        if (reaction.stp !== null) {
            val lateFunctionName = decl.name.toLowerCase + '_STP_function' + reactionIndex

            code.pr('void ' + lateFunctionName + '(void* instance_args) {')
            code.indent();
            code.pr(CReactionGenerator.generateInitializationForReaction(body, reaction, decl, reactionIndex, types, errorReporter, mainDef, isFederatedAndDecentralized, target.requiresTypes))
            // Code verbatim from 'late'
            code.prSourceLineNumber(reaction.stp.code)
            code.pr(reaction.stp.code.toText)
            code.unindent()
            code.pr("}")
        }

        // Now generate code for the deadline violation function, if there is one.
        if (reaction.deadline !== null) {
            // The following name has to match the choice in generateReactionInstances
            val deadlineFunctionName = CReactionGenerator.generateDeadlineFunctionName(decl, reactionIndex)

            code.pr('void ' + deadlineFunctionName + '(void* instance_args) {')
            code.indent();
            code.pr(CReactionGenerator.generateInitializationForReaction(body, reaction, decl, reactionIndex, types, errorReporter, mainDef, isFederatedAndDecentralized, target.requiresTypes))
            // Code verbatim from 'deadline'
            code.prSourceLineNumber(reaction.deadline.code)
            code.pr(reaction.deadline.code.toText)
            code.unindent()
            code.pr("}")
        }
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
                
                startScopedBlock(temp, child, true);

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
                endScopedBlock(temp);

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
                                startScopedBlock(temp);
                                temp.pr("int count = 0;");
                                startScopedBlock(temp, instance, true);
                                startScopedBankChannelIteration(temp, port, null);
                            } else {
                                startScopedBankChannelIteration(temp, port, "count");
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
                                endScopedBlock(temp);
                                endScopedBlock(temp);
                                endScopedBankChannelIteration(temp, port, null);
                            } else {
                                endScopedBankChannelIteration(temp, port, "count");
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
                            startScopedBlock(temp, instance, true);
                            startScopedBankChannelIteration(temp, port, "count");
                            
                            val portRef = CUtil.portRef(port, true, true, null, null, null);
                            
                            temp.pr('''
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token = &«portRef»->token;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status = (port_status_t*)&«portRef»->is_present;
                                _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;
                            ''')
                            startTimeStepTokens += port.width * currentFederate.numRuntimeInstances(port.parent);

                            endScopedBankChannelIteration(temp, port, "count");
                            endScopedBlock(temp);
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
                startScopedBlock(temp, instance, true);
                
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
                endScopedBlock(temp);
            }
        }
        if (foundOne) startTimeStep.pr(temp.toString());
        temp = new CodeBuilder();
        foundOne = false;
        
        // Next, set up the table to mark each output of each contained reactor absent.
        for (child : instance.children) {
            if (currentFederate.contains(child) && child.outputs.size > 0) {
                
                startScopedBlock(temp);
                temp.pr("int count = 0;");
                startScopedBlock(temp, child, true);
        
                var channelCount = 0;
                for (output : child.outputs) {
                    if (!output.dependsOnReactions.isEmpty){
                        foundOne = true;
                        
                        temp.pr('''
                            // Add port «output.getFullName» to array of is_present fields.
                        ''')
                        startChannelIteration(temp, output);
                        
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
                        endChannelIteration(temp, output);
                    }
                }
                startTimeStepIsPresentCount += channelCount * currentFederate.numRuntimeInstances(child);
                endScopedBlock(temp);
                endScopedBlock(temp);
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
                startScopedBlock(startTimeStep, child, true);
                startScopedBlock(initializeTriggerObjects, child, true);
                generateReactorInstance(child);
                endScopedBlock(initializeTriggerObjects);
                endScopedBlock(startTimeStep);
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
                    «nameOfSelfStruct»->_lf__mode_state.parent_mode = «parentModeRef»;
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
                _lf_modal_reactor_states[«modalReactorCount++»] = &«nameOfSelfStruct»->_lf__mode_state;
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
    
    /** 
     * Set the reaction priorities based on dependency analysis.
     * @param reactor The reactor on which to do this.
     * @param builder Where to write the code.
     */
    private def boolean setReactionPriorities(ReactorInstance reactor, CodeBuilder builder) {
        var foundOne = false;

        // Force calculation of levels if it has not been done.
        reactor.assignLevels();
        
        // If any reaction has multiple levels, then we need to create
        // an array with the levels here, before entering the iteration over banks.
        val prolog = new CodeBuilder();
        val epilog = new CodeBuilder();
        for (r : reactor.reactions) {
            if (currentFederate.contains(r.definition)) {
                val levels = r.getLevels();
                if (levels.size != 1) {
                    if (prolog.length() == 0) {
                        startScopedBlock(prolog);
                        endScopedBlock(epilog);
                    }
                    // Cannot use the above set of levels because it is a set, not a list.
                    prolog.pr('''
                        int «r.uniqueID»_levels[] = { «r.getLevelsList().join(", ")» };
                    ''')
                }
            }
        }

        val temp = new CodeBuilder();
        temp.pr("// Set reaction priorities for " + reactor.toString());
        
        startScopedBlock(temp, reactor, true);

        for (r : reactor.reactions) {
            if (currentFederate.contains(r.definition)) {
                foundOne = true;

                // The most common case is that all runtime instances of the
                // reaction have the same level, so deal with that case
                // specially.
                val levels = r.getLevels();
                if (levels.size == 1) {
                    var level = -1;
                    for (l : levels) {
                        level = l;
                    }
                    // xtend doesn't support bitwise operators...
                    val indexValue = XtendUtil.longOr(r.deadline.toNanoSeconds << 16, level)
                    val reactionIndex = "0x" + Long.toString(indexValue, 16) + "LL"

                    temp.pr('''
                        «CUtil.reactionRef(r)».chain_id = «r.chainID.toString»;
                        // index is the OR of level «level» and 
                        // deadline «r.deadline.toNanoSeconds» shifted left 16 bits.
                        «CUtil.reactionRef(r)».index = «reactionIndex»;
                    ''')
                } else {
                    val reactionDeadline = "0x" + Long.toString(r.deadline.toNanoSeconds, 16) + "LL"

                    temp.pr('''
                        «CUtil.reactionRef(r)».chain_id = «r.chainID.toString»;
                        // index is the OR of levels[«CUtil.runtimeIndex(r.parent)»] and 
                        // deadline «r.deadline.toNanoSeconds» shifted left 16 bits.
                        «CUtil.reactionRef(r)».index = («reactionDeadline» << 16) | «r.uniqueID»_levels[«CUtil.runtimeIndex(r.parent)»];
                    ''')
                }
            }
        }
        for (child : reactor.children) {
            if (currentFederate.contains(child)) {
                foundOne = setReactionPriorities(child, temp) || foundOne;
            }
        }
        endScopedBlock(temp);
        
        if (foundOne) {
            builder.pr(prolog.toString());
            builder.pr(temp.toString());
            builder.pr(epilog.toString());            
        }
        return foundOne;
    }

    override getTargetTypes() {
        return types;
    }

    // //////////////////////////////////////////
    // // Protected methods.

    // Perform set up that does not generate code
    protected def setUpParameters(LFGeneratorContext context) {
        accommodatePhysicalActionsIfPresent()
        targetConfig.compileDefinitions.put("LOG_LEVEL", targetConfig.logLevel.ordinal.toString);
        targetConfig.compileAdditionalSources.add("ctarget.c");
        targetConfig.compileAdditionalSources.add("core" + File.separator + "mixed_radix.c");
        setCSpecificDefaults(context)
        parseTargetParameters()

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
        if (targetConfig.threading) {
            pickScheduler();
        }
        pickCompilePlatform();
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
    def void generatePreamble() {
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
        // call built-in functions.
        code.prComment("Code generated by the Lingua Franca compiler from:")
        code.prComment("file:/" + FileUtil.toUnixString(fileConfig.srcFile))
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
        "^(file:/(?<path>.*)):(?<line>[0-9]+):(?<column>[0-9]+):(?<message>.*)$"
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
    
    /**
     * If the specified port is a multiport, then start a specified iteration
     * over the channels of the multiport using as the channel index the
     * variable name returned by {@link CUtil.channelIndex(PortInstance)}.
     * If the port is not a multiport, do nothing.
     * This is required to be followed by {@link endChannelIteration(StringBuilder, PortInstance}.
     * @param builder Where to write the code.
     * @param port The port.
     */
    private def void startChannelIteration(CodeBuilder builder, PortInstance port) {
        if (port.isMultiport) {
            val channel = CUtil.channelIndexName(port);
            builder.pr('''
                // Port «port.fullName» is a multiport. Iterate over its channels.
                for (int «channel» = 0; «channel» < «port.width»; «channel»++) {
            ''')
            builder.indent();
        }
    }
    
    /**
     * If the specified port is a multiport, then start a specified iteration
     * over the channels of the multiport using as the channel index the
     * variable name returned by {@link CUtil.channelIndex(PortInstance)}.
     * If the port is not a multiport, do nothing.
     * This is required to be followed by {@link endChannelIteration(StringBuilder, PortInstance}.
     * @param builder Where to write the code.
     * @param port The port.
     */
    private def void endChannelIteration(CodeBuilder builder, PortInstance port) {
        if (port.isMultiport) {
            builder.unindent();
            builder.pr("}");
        }
    }

    /**
     * Start a scoped block, which is a section of code
     * surrounded by curley braces and indented.
     * This must be followed by an {@link endScopedBlock(StringBuilder)}.
     * @param builder The code emitter into which to write.
     */
    private def void startScopedBlock(CodeBuilder builder) {
        builder.pr("{");
        builder.indent();
    }

    /**
     * Start a scoped block for the specified reactor.
     * If the reactor is a bank, then this starts a for loop
     * that iterates over the bank members using a standard index
     * variable whose name is that returned by {@link CUtil.bankIndex(ReactorInstance)}.
     * If the reactor is null or is not a bank, then this simply
     * starts a scoped block by printing an opening curly brace.
     * This also adds a declaration of a pointer to the self
     * struct of the reactor or bank member.
     * 
     * This block is intended to be nested, where each block is
     * put within a similar block for the reactor's parent.
     * This ensures that all (possibly nested) bank index variables
     * are defined within the block.
     * 
     * This must be followed by an {@link endScopedBlock(StringBuilder)}.
     * 
     * @param builder The place to write the code.
     * @param reactor The reactor instance.
     * @param restrict For federated execution only, if this is true, then
     *  skip iterations where the topmost bank member is not in the federate.
     */
    protected def void startScopedBlock(CodeBuilder builder, ReactorInstance reactor, boolean restrict) {
        // NOTE: This is protected because it is used by the PythonGenerator.
        if (reactor !== null && reactor.isBank) {
            val index = CUtil.bankIndexName(reactor);
            if (reactor.depth == 1 && isFederated && restrict) {
                // Special case: A bank of federates. Instantiate only the current federate.
                startScopedBlock(builder);
                builder.pr('''
                    int «index» = «currentFederate.bankIndex»;
                ''')
            } else {
                builder.pr('''
                    // Reactor is a bank. Iterate over bank members.
                    for (int «index» = 0; «index» < «reactor.width»; «index»++) {
                ''')
                builder.indent();
            }
        } else {
            startScopedBlock(builder);
        }
    }

    /**
     * End a scoped block.
     * @param builder The place to write the code.
     */
    protected def void endScopedBlock(CodeBuilder builder) {
        // NOTE: This is protected because it is used by the PythonGenerator.
        builder.unindent();
        builder.pr("}");
    }
    
    /**
     * Start a scoped block to iterate over bank members and
     * channels for the specified port with a a variable with
     * the name given by count counting the iterations.
     * If this port is a multiport, then the channel index
     * variable name is that returned by {@link CUtil.channelIndex(PortInstance)}.
     *
     * This block is intended to be nested, where each block is
     * put within a similar block for the reactor's parent.
     *
     * This is required to be followed by a call to
     * {@link endScopedBankChannelIteration(StringBuilder, PortInstance, String)}.
     * @param builder Where to write the code.
     * @param port The port.
     * @param count The variable name to use for the counter, or
     *  null to not provide a counter.
     */
    private def void startScopedBankChannelIteration(
        CodeBuilder builder, PortInstance port, String count
    ) {
        if (count !== null) {
            startScopedBlock(builder);
            builder.pr('''int «count» = 0;''');
        }
        startScopedBlock(builder, port.parent, true);
        startChannelIteration(builder, port);
    }

    /**
     * End a scoped block to iterate over bank members and
     * channels for the specified port with a a variable with
     * the name given by count counting the iterations.
     * @param builder Where to write the code.
     * @param port The port.
     * @param count The variable name to use for the counter, or
     *  null to not provide a counter.
     */
    private def void endScopedBankChannelIteration(
        CodeBuilder builder, PortInstance port, String count
    ) {
        if (count !== null) {
            builder.pr(count + "++;");
        }
        endChannelIteration(builder, port);
        endScopedBlock(builder);
        if (count !== null) {
            endScopedBlock(builder);
        }
    }
    
    /**
     * Start a scoped block that iterates over the specified range of port channels.
     * 
     * This must be followed by a call to
     * {@link #endScopedRangeBlock(StringBuilder, RuntimeRange<PortInstance>)}.
     *
     * This block should NOT be nested, where each block is
     * put within a similar block for the reactor's parent.
     * Within the created block, every use of
     * {@link CUtil.reactorRef(ReactorInstance, String)}
     * must provide the second argument, a runtime index variable name,
     * that must match the runtimeIndex parameter given here.
     * 
     * @param builder Where to write the code.
     * @param range The range of port channels.
     * @param runtimeIndex A variable name to use to index the runtime instance of
     *  either port's parent or the port's parent's parent (if nested is true), or
     *  null to use the default, "runtime_index".
     * @param bankIndex A variable name to use to index the bank of the port's parent or null to use the
     *  default, the string returned by {@link CUtil.bankIndexName(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil.channelIndexName(PortInstance)}.
     * @param nested If true, then the runtimeIndex variable will be set
     *  to the bank index of the port's parent's parent rather than the
     *  port's parent.
     * @param restrict For federated execution (only), if this argument
     *  is true, then the iteration will skip over bank members that
     *  are not in the current federate.
     */
    private def void startScopedRangeBlock(
        CodeBuilder builder, 
        RuntimeRange<PortInstance> range, 
        String runtimeIndex,
        String bankIndex,
        String channelIndex,
        boolean nested,
        boolean restrict
    ) {
        
        builder.pr('''
            // Iterate over range «range.toString()».
        ''')
        val ri = (runtimeIndex === null)? "runtime_index" : runtimeIndex;
        val ci = (channelIndex === null)? CUtil.channelIndexName(range.instance) : channelIndex;
        val bi = (bankIndex === null)? CUtil.bankIndexName(range.instance.parent) : bankIndex;
        val rangeMR = range.startMR();
        val sizeMR = rangeMR.getDigits().size();
        val nestedLevel = (nested) ? 2 : 1;

        startScopedBlock(builder);
        if (range.width > 1) {
            builder.pr('''
                int range_start[] =  { «rangeMR.getDigits().join(", ")» };
                int range_radixes[] = { «rangeMR.getRadixes().join(", ")» };
                int permutation[] = { «range.permutation().join(", ")» };
                mixed_radix_int_t range_mr = {
                    «sizeMR»,
                    range_start,
                    range_radixes,
                    permutation
                };
                for (int range_count = «range.start»; range_count < «range.start» + «range.width»; range_count++) {
            ''');
            builder.indent();
            builder.pr('''
                int «ri» = mixed_radix_parent(&range_mr, «nestedLevel»); // Runtime index.
                int «ci» = range_mr.digits[0]; // Channel index.
                int «bi» = «IF sizeMR <= 1»0«ELSE»range_mr.digits[1]«ENDIF»; // Bank index.
            ''')
            if (isFederated) {
                if (restrict) {
                    // In case we have a bank of federates. Need that iteration
                    // only cover the one federate. The last digit of the mixed-radix
                    // number is the bank index (or 0 if this is not a bank of federates).
                    builder.pr('''
                        if (range_mr.digits[range_mr.size - 1] == «currentFederate.bankIndex») {
                    ''')
                    builder.indent();
                } else {
                    startScopedBlock(builder);
                }
            }
        } else {
            val ciValue = rangeMR.getDigits().get(0);
            val riValue = rangeMR.get(nestedLevel);
            val biValue = (sizeMR > 1)? rangeMR.getDigits().get(1) : 0;
            if (isFederated) {
                if (restrict) {
                    // Special case. Have a bank of federates. Need that iteration
                    // only cover the one federate. The last digit of the mixed-radix
                    // number identifies the bank member (or is 0 if not within a bank).
                    builder.pr('''
                        if («rangeMR.get(sizeMR - 1)» == «currentFederate.bankIndex») {
                    ''')
                    builder.indent();
                } else {
                    startScopedBlock(builder);
                }
            }
            builder.pr('''
                int «ri» = «riValue»; // Runtime index.
                int «ci» = «ciValue»; // Channel index.
                int «bi» = «biValue»; // Bank index.
                int range_count = 0;
            ''')
        }
    }

    /**
     * End a scoped block for the specified range.
     * @param builder Where to write the code.
     * @param range The send range.
     */
    private def void endScopedRangeBlock(CodeBuilder builder, RuntimeRange<PortInstance> range) {
        if (isFederated) {
            // Terminate the if statement or block (if not restrict).
            endScopedBlock(builder);
        }
        if (range.width > 1) {
            builder.pr("mixed_radix_incr(&range_mr);");
            endScopedBlock(builder); // Terminate for loop.
        }
        endScopedBlock(builder);
    }
    
    /** Standardized name for channel index variable for a source. */
    static val sc = "src_channel";
    /** Standardized name for bank index variable for a source. */
    static val sb = "src_bank";
    /** Standardized name for runtime index variable for a source. */
    static val sr = "src_runtime";
    /** Standardized name for channel index variable for a destination. */
    static val dc = "dst_channel";
    /** Standardized name for bank index variable for a destination. */
    static val db = "dst_bank";
    /** Standardized name for runtime index variable for a destination. */
    static val dr = "dst_runtime";

    /**
     * Start a scoped block that iterates over the specified pair of ranges.
     * The destination range can be wider than the source range, in which case the
     * source range is reused until the destination range is filled.
     * The following integer variables will be defined within the scoped block:
     * 
     * * src_channel: The channel index for the source.
     * * src_bank: The bank index of the source port's parent.
     * * src_runtime: The runtime index of the source port's parent or
     *   the parent's parent (if the source is an input).
     * 
     * * dst_channel: The channel index for the destination.
     * * dst_bank: The bank index of the destination port's parent.
     * * dst_runtime: The runtime index of the destination port's parent or
     *   the parent's parent (if destination is an output).
     * 
     * For convenience, the above variable names are defined in the private
     * class variables sc, sb, sr, and dc, db, dr.
     *  
     * This block should NOT be nested, where each block is
     * put within a similar block for the reactor's parent.
     * Within the created block, every use of
     * {@link CUtil.reactorRef(ReactorInstance, String, String)}
     * and related functions must provide the above variable names.
     * 
     * This must be followed by a call to
     * {@link #endScopedRangeBlock(StringBuilder, SendRange, RuntimeRange<PortInstance>)}.
     * 
     * @param builder Where to write the code.
     * @param srcRange The send range.
     * @param dstRange The destination range.
     */
    private def void startScopedRangeBlock(
        CodeBuilder builder, 
        SendRange srcRange, 
        RuntimeRange<PortInstance> dstRange 
    ) {
        val srcRangeMR = srcRange.startMR();
        val srcSizeMR = srcRangeMR.radixes.size();
        val srcNestedLevel = (srcRange.instance.isInput) ? 2 : 1;
        val dstNested = dstRange.instance.isOutput;
        
        builder.pr('''
            // Iterate over ranges «srcRange.toString» and «dstRange.toString».
        ''')
        
        if (isFederated && srcRange.width == 1) {
            // Skip this whole block if the src is not in the federate.
            builder.pr('''
                if («srcRangeMR.get(srcRangeMR.numDigits() - 1)» == «currentFederate.bankIndex») {
            ''')
            builder.indent();
        } else {
            startScopedBlock(builder);
        }
        
        if (srcRange.width > 1) {
            builder.pr('''
                int src_start[] =  { «srcRangeMR.getDigits().join(", ")» };
                int src_value[] =  { «srcRangeMR.getDigits().join(", ")» }; // Will be incremented.
                int src_radixes[] = { «srcRangeMR.getRadixes().join(", ")» };
                int src_permutation[] = { «srcRange.permutation().join(", ")» };
                mixed_radix_int_t src_range_mr = {
                    «srcSizeMR»,
                    src_value,
                    src_radixes,
                    src_permutation
                };
            ''');
        } else {
            val ciValue = srcRangeMR.getDigits().get(0);
            val biValue = (srcSizeMR > 1)? srcRangeMR.getDigits().get(1) : 0;
            val riValue = srcRangeMR.get(srcNestedLevel);
            builder.pr('''
                int «sr» = «riValue»; // Runtime index.
                int «sc» = «ciValue»; // Channel index.
                int «sb» = «biValue»; // Bank index.
            ''')
        }
        
        startScopedRangeBlock(builder, dstRange, dr, db, dc, dstNested, true);

        if (srcRange.width > 1) {
            builder.pr('''
                int «sr» = mixed_radix_parent(&src_range_mr, «srcNestedLevel»); // Runtime index.
                int «sc» = src_range_mr.digits[0]; // Channel index.
                int «sb» = «IF srcSizeMR <= 1»0«ELSE»src_range_mr.digits[1]«ENDIF»; // Bank index.
            ''')
        }
        
        // The above startScopedRangeBlock() call will skip any iteration where the destination
        // is a bank member is not in the federation. Here, we skip any iteration where the
        // source is a bank member not in the federation.
        if (isFederated && srcRange.width > 1) {
            // The last digit of the mixed radix
            // number identifies the bank (or is 0 if no bank).
            builder.pr('''
                if (src_range_mr.digits[src_range_mr.size - 1] == «currentFederate.bankIndex») {
            ''')
            builder.indent();
        }
    }

    /**
     * End a scoped block that iterates over the specified pair of ranges.
     * 
     * @param builder Where to write the code.
     * @param srcRange The send range.
     * @param dstRange The destination range.
     */
    private def void endScopedRangeBlock(
        CodeBuilder builder, 
        SendRange srcRange, 
        RuntimeRange<PortInstance> dstRange 
    ) {
        // Do not use endScopedRangeBlock because we need things nested.
        if (isFederated) {
            if (srcRange.width > 1) {
                // Terminate the if statement.
                endScopedBlock(builder);
            }
            // Terminate the if statement or block (if not restrict).
            endScopedBlock(builder);
        }
        if (srcRange.width > 1) {
            builder.pr('''
                mixed_radix_incr(&src_range_mr);
                if (mixed_radix_to_int(&src_range_mr) >= «srcRange.start» + «srcRange.width») {
                    // Start over with the source.
                    for (int i = 0; i < src_range_mr.size; i++) {
                        src_range_mr.digits[i] = src_start[i];
                    }
                }
            ''');
        }
        if (dstRange.width > 1) {
            builder.pr("mixed_radix_incr(&range_mr);");
            endScopedBlock(builder); // Terminate for loop.
        }
        // Terminate unconditional scope block in startScopedRangeBlock calls.
        endScopedBlock(builder);
        endScopedBlock(builder);
    }    

    /**
     * Generate assignments of pointers in the "self" struct of a destination
     * port's reactor to the appropriate entries in the "self" struct of the
     * source reactor. If this port is an input, then it is being written
     * to by a reaction belonging to the parent of the port's parent.
     * If it is an output, then it is being written to by a reaction belonging
     * to the port's parent.
     * @param port A port that is written to by reactions.
     */
    private def void connectPortToEventualDestinations(PortInstance src) {
        if (!currentFederate.contains(src.parent)) return;
        for (srcRange: src.eventualDestinations()) {
            for (dstRange : srcRange.destinations) {
                val dst = dstRange.instance;
                val destStructType = variableStructType(dst)
                
                // NOTE: For federated execution, dst.parent should always be contained
                // by the currentFederate because an AST transformation removes connections
                // between ports of distinct federates. So the following check is not
                // really necessary.
                if (currentFederate.contains(dst.parent)) {
                    
                    val mod = (dst.isMultiport || (src.isInput && src.isMultiport))? "" : "&";
                    
                    code.pr('''
                        // Connect «srcRange.toString» to port «dstRange.toString»
                    ''')
                    startScopedRangeBlock(code, srcRange, dstRange);
                    
                    if (src.isInput) {
                        // Source port is written to by reaction in port's parent's parent
                        // and ultimate destination is further downstream.
                        code.pr('''
                            «CUtil.portRef(dst, dr, db, dc)» = («destStructType»*)«mod»«CUtil.portRefNested(src, sr, sb, sc)»;
                        ''')
                    } else if (dst.isOutput) {
                        // An output port of a contained reactor is triggering a reaction.
                        code.pr('''
                            «CUtil.portRefNested(dst, dr, db, dc)» = («destStructType»*)&«CUtil.portRef(src, sr, sb, sc)»;
                        ''')
                    } else {
                        // An output port is triggering
                        code.pr('''
                            «CUtil.portRef(dst, dr, db, dc)» = («destStructType»*)&«CUtil.portRef(src, sr, sb, sc)»;
                        ''')
                    }
                    endScopedRangeBlock(code, srcRange, dstRange);
                }
            }
        }
    }
    
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
     * Perform initialization functions that must be performed after
     * all reactor runtime instances have been created.
     * This function creates nested loops over nested banks.
     * @param reactor The container.
     * @param federate The federate (used to determine whether a
     *  reaction belongs to the federate).
     */
    private def void deferredInitialize(
        ReactorInstance reactor, Iterable<ReactionInstance> reactions
    ) {
        if (!currentFederate.contains(reactor)) {
            return;
        }
        
        code.pr('''// **** Start deferred initialize for «reactor.getFullName()»''')
        
        // First batch of initializations is within a for loop iterating
        // over bank members for the reactor's parent.
        startScopedBlock(code, reactor, true);
        
        // If the child has a multiport that is an effect of some reaction in its container,
        // then we have to generate code to allocate memory for arrays pointing to
        // its data. If the child is a bank, then memory is allocated for the entire
        // bank width because a reaction cannot specify which bank members it writes
        // to so we have to assume it can write to any.
        deferredAllocationForEffectsOnInputs(reactor);

        deferredReactionMemory(reactions);

        // For outputs that are not primitive types (of form type* or type[]),
        // create a default token on the self struct.
        deferredCreateDefaultTokens(reactor);

        for (child: reactor.children) {
            if (currentFederate.contains(child)) {
                deferredInitialize(child, child.reactions);
            }
        }
                
        endScopedBlock(code)
        
        code.pr('''// **** End of deferred initialize for «reactor.getFullName()»''')
    }
    
    /**
     * Perform initialization functions that must be performed after
     * all reactor runtime instances have been created.
     * This function does not create nested loops over nested banks,
     * so each function it calls must handle its own iteration
     * over all runtime instance.
     * @param reactor The container.
     * @param federate The federate (used to determine whether a
     *  reaction belongs to the federate).
     */
    private def void deferredInitializeNonNested(
        ReactorInstance reactor, Iterable<ReactionInstance> reactions
    ) {
        code.pr('''// **** Start non-nested deferred initialize for «reactor.getFullName()»''')
                
        // Initialize the num_destinations fields of port structs on the self struct.
        // This needs to be outside the above scoped block because it performs
        // its own iteration over ranges.
        deferredInputNumDestinations(reactions);
        
        // Second batch of initializes cannot be within a for loop
        // iterating over bank members because they iterate over send
        // ranges which may span bank members.
        deferredOutputNumDestinations(reactor); // NOTE: Does nothing for top level.
        deferredFillTriggerTable(reactions);
        
        deferredOptimizeForSingleDominatingReaction(reactor);
        
        for (child: reactor.children) {
            if (currentFederate.contains(child)) {
                deferredInitializeNonNested(child, child.reactions);
            }
        }
        
        code.pr('''// **** End of non-nested deferred initialize for «reactor.getFullName()»''')
    }

    /**
     * Generate assignments of pointers in the "self" struct of a destination
     * port's reactor to the appropriate entries in the "self" struct of the
     * source reactor. This has to be done after all reactors have been created
     * because inputs point to outputs that are arbitrarily far away.
     * @param instance The reactor instance.
     */
    private def void deferredConnectInputsToOutputs(ReactorInstance instance) {
        code.pr('''// Connect inputs and outputs for reactor «instance.getFullName».''')
        
        // Iterate over all ports of this reactor that depend on reactions.
        for (input : instance.inputs) {
            if (!input.dependsOnReactions.isEmpty()) {
                // Input is written to by reactions in the parent of the port's parent.
                connectPortToEventualDestinations(input); 
            }
        }
        for (output : instance.outputs) {
            if (!output.dependsOnReactions.isEmpty()) {
                // Output is written to by reactions in the port's parent.
                connectPortToEventualDestinations(output); 
            }
        }
        for (child: instance.children) {
            deferredConnectInputsToOutputs(child);
        }
    }
    
    /** 
     * For each output of the specified reactor that has a token type
     * (type* or type[]), create a default token and put it on the self struct.
     * @param parent The reactor.
     */
    private def void deferredCreateDefaultTokens(ReactorInstance reactor) {
        // Look for outputs with token types.
        for (output : reactor.outputs) {
            val type = (output.definition as Output).inferredType;
            if (CUtil.isTokenType(type, types)) {
                // Create the template token that goes in the trigger struct.
                // Its reference count is zero, enabling it to be used immediately.
                var rootType = CUtil.rootType(types.getTargetType(type));
                // If the rootType is 'void', we need to avoid generating the code
                // 'sizeof(void)', which some compilers reject.
                val size = (rootType == 'void') ? '0' : '''sizeof(«rootType»)'''
                startChannelIteration(code, output);
                code.pr('''
                    «CUtil.portRef(output)».token = _lf_create_token(«size»);
                ''')
                endChannelIteration(code, output);
            }
        }
    }
    
    /**
     * For each output port of the specified reactor,
     * set the num_destinations field of port structs on its self struct
     * equal to the total number of destination reactors. This is used
     * to initialize reference counts in dynamically allocated tokens
     * sent to other reactors.
     * @param reactor The reactor instance.
     */
    private def void deferredOutputNumDestinations(ReactorInstance reactor) {
        // For top-level, ignore this.
        if (reactor == main) return;
        
        // Reference counts are decremented by each destination reactor
        // at the conclusion of a time step. Hence, the initial reference
        // count should equal the number of destination _reactors_, not the
        // number of destination ports nor the number of destination reactions.
        // One of the destination reactors may be the container of this
        // instance because it may have a reaction to an output of this instance.
        for (output : reactor.outputs) {
            for (sendingRange : output.eventualDestinations) {
                code.pr("// For reference counting, set num_destinations for port " + output.fullName + ".");
                
                startScopedRangeBlock(code, sendingRange, sr, sb, sc, sendingRange.instance.isInput, true);
                
                code.pr('''
                    «CUtil.portRef(output, sr, sb, sc)».num_destinations = «sendingRange.getNumberOfDestinationReactors()»;
                ''')
                
                endScopedRangeBlock(code, sendingRange);
            }
        }
    }
    
    /**
     * For each input port of a contained reactor that receives data
     * from one or more of the specified reactions, set the num_destinations
     * field of the corresponding port structs on the self struct of
     * the reaction's parent reactor equal to the total number of
     * destination reactors. This is used to initialize reference
     * counts in dynamically allocated tokens sent to other reactors.
     * @param reactions The reactions.
     */
    private def void deferredInputNumDestinations(Iterable<ReactionInstance> reactions) {
        // Reference counts are decremented by each destination reactor
        // at the conclusion of a time step. Hence, the initial reference
        // count should equal the number of destination _reactors_, not the
        // number of destination ports nor the number of destination reactions.
        // One of the destination reactors may be the container of this
        // instance because it may have a reaction to an output of this instance.

        // Since a port may be written to by multiple reactions,
        // ensure that this is done only once.
        val portsHandled = new HashSet<PortInstance>();
        for (reaction : reactions) {
            for (port : reaction.effects.filter(PortInstance)) {
                if (port.isInput && !portsHandled.contains(port)) {
                    // Port is an input of a contained reactor that gets data from a reaction of this reactor.
                    portsHandled.add(port);
                    
                    code.pr('''
                        // For reference counting, set num_destinations for port «port.parent.name».«port.name».
                    ''')
                    
                    // The input port may itself have multiple destinations.
                    for (sendingRange : port.eventualDestinations) {
                    
                        startScopedRangeBlock(code, sendingRange, sr, sb, sc, sendingRange.instance.isInput, true);
                        
                        // Syntax is slightly different for a multiport output vs. single port.
                        val connector = (port.isMultiport())? "->" : ".";
                        code.pr('''
                            «CUtil.portRefNested(port, sr, sb, sc)»«connector»num_destinations = «sendingRange.getNumberOfDestinationReactors»;
                        ''')

                        endScopedRangeBlock(code, sendingRange);
                    }
                }
            }
        }
    }

    /**
     * If any reaction of the specified reactor provides input
     * to a contained reactor, then generate code to allocate
     * memory to store the data produced by those reactions.
     * The allocated memory is pointed to by a field called
     * `_lf_containername.portname` on the self struct of the reactor.
     * @param reactor The reactor.
     */
    private def void deferredAllocationForEffectsOnInputs(ReactorInstance reactor) {
        // Keep track of ports already handled. There may be more than one reaction
        // in the container writing to the port, but we want only one memory allocation.
        val portsHandled = new HashSet<PortInstance>();

        val reactorSelfStruct = CUtil.reactorRef(reactor); 
        
        // Find parent reactions that mention multiport inputs of this reactor.
        for (reaction : reactor.reactions) { 
            for (effect : reaction.effects.filter(PortInstance)) {
                if (effect.parent.depth > reactor.depth // port of a contained reactor.
                    && effect.isMultiport
                    && !portsHandled.contains(effect)
                    && currentFederate.contains(effect.parent)
                ) {
                    code.pr("// A reaction writes to a multiport of a child. Allocate memory.")
                    portsHandled.add(effect);
                    
                    val portStructType = variableStructType(effect)
                            
                    startScopedBlock(code, effect.parent, true);
                    
                    val effectRef = CUtil.portRefNestedName(effect);

                    code.pr('''
                        «effectRef»_width = «effect.width»;
                        // Allocate memory to store output of reaction feeding 
                        // a multiport input of a contained reactor.
                        «effectRef» = («portStructType»**)_lf_allocate(
                                «effect.width», sizeof(«portStructType»*),
                                &«reactorSelfStruct»->base.allocations); 
                        for (int i = 0; i < «effect.width»; i++) {
                            «effectRef»[i] = («portStructType»*)_lf_allocate(
                                    1, sizeof(«portStructType»),
                                    &«reactorSelfStruct»->base.allocations); 
                        }
                    ''')
                    
                    endScopedBlock(code);
                }
            }
        }
    }
    
    /**
     * For each reaction of the specified reactor,
     * Set the last_enabling_reaction field of the reaction struct to point
     * to the single dominating upstream reaction, if there is one, or to be
     * NULL if not.
     * 
     * @param reactor The reactor.
     */
    private def deferredOptimizeForSingleDominatingReaction (ReactorInstance r) {
        for (reaction : r.reactions) {
            if (currentFederate.contains(reaction.definition)
                && currentFederate.contains(reaction.parent)
            ) {
                
                // For federated systems, the above test may not be enough if there is a bank
                // of federates.  Calculate the divisor needed to compute the federate bank
                // index from the instance index of the reaction.
                var divisor = 1;
                if (isFederated) {
                    var parent = reaction.parent;
                    while (parent.depth > 1) {
                        divisor *= parent.width;
                        parent = parent.parent;
                    }
                }
                
                // The following code attempts to gather into a loop assignments of successive
                // bank members relations between reactions to avoid large chunks of inline code
                // when a large bank sends to a large bank or when a large bank receives from
                // one reaction that is either multicasting or sending through a multiport.
                var start = 0;
                var end = 0;
                var domStart = 0;
                var same = false; // Set to true when finding a string of identical dominating reactions.
                var previousRuntime = null as ReactionInstance.Runtime;
                var first = true;  //First time through the loop.
                for (runtime : reaction.getRuntimeInstances()) {
                    if (!first) { // Not the first time through the loop.
                        if (same) { // Previously seen at least two identical dominating.
                            if (runtime.dominating != previousRuntime.dominating) {
                                // End of streak of same dominating reaction runtime instance.
                                printOptimizeForSingleDominatingReaction(
                                    previousRuntime, start, end, domStart, same, divisor
                                );
                                same = false;
                                start = runtime.id;
                                domStart = (runtime.dominating !== null) ? runtime.dominating.id : 0;
                            }
                        } else if (runtime.dominating == previousRuntime.dominating) {
                            // Start of a streak of identical dominating reaction runtime instances.
                            same = true;
                        } else if (runtime.dominating !== null && previousRuntime.dominating !== null
                            && runtime.dominating.reaction == previousRuntime.dominating.reaction
                        ) {
                            // Same dominating reaction even if not the same dominating runtime.
                            if (runtime.dominating.id != previousRuntime.dominating.id + 1) {
                                // End of a streak of contiguous runtimes.
                                printOptimizeForSingleDominatingReaction(
                                    previousRuntime, start, end, domStart, same, divisor
                                );
                                same = false;
                                start = runtime.id;
                                domStart = runtime.dominating.id;
                            }
                        } else {
                            // Different dominating reaction.
                            printOptimizeForSingleDominatingReaction(
                                    previousRuntime, start, end, domStart, same, divisor
                            );
                            same = false;
                            start = runtime.id;
                            domStart = (runtime.dominating !== null) ? runtime.dominating.id : 0;
                        }
                    }
                    first = false;
                    previousRuntime = runtime;
                    end++;
                }
                if (end > start) {
                    printOptimizeForSingleDominatingReaction(previousRuntime, start, end, domStart, same, divisor);
                }
            }
        }
    }
    
    /**
     * Print statement that sets the last_enabling_reaction field of a reaction.
     */
    private def printOptimizeForSingleDominatingReaction(
        ReactionInstance.Runtime runtime, int start, int end, int domStart, boolean same, int divisor
    ) {
        var domDivisor = 1;
        if (isFederated && runtime.dominating !== null) {
            val domReaction = runtime.dominating.getReaction();
            // No need to do anything if the dominating reaction is not in the federate.
            // Note that this test is imperfect because the current federate may be a
            // bank member.
            if (!currentFederate.contains(domReaction.definition)
                    || !currentFederate.contains(domReaction.getParent())) {
                return;
            }
            // To really know whether the dominating reaction is in the federate,
            // we need to calculate a divisor for its runtime index. 
            var parent = runtime.dominating.getReaction().parent;
            while (parent.depth > 1) {
                domDivisor *= parent.width;
                parent = parent.parent;
            }
        }
        
        var dominatingRef = "NULL";
                
        if (end > start + 1) {
            startScopedBlock(code);
            val reactionRef = CUtil.reactionRef(runtime.reaction, "i");
            if (runtime.dominating !== null) {
                if (same) {
                    dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.reaction, "" + domStart) + ")";
                } else {
                    dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.reaction, "j++") + ")";
                }
            }
            code.pr('''
                // «runtime.reaction.getFullName» dominating upstream reaction.
                int j = «domStart»;
                for (int i = «start»; i < «end»; i++) {
                    «IF isFederated»
                    if (i / «divisor» != «currentFederate.bankIndex») continue; // Reaction is not in the federate.
                    «IF runtime.dominating !== null»
                    if (j / «domDivisor» != «currentFederate.bankIndex») continue; // Dominating reaction is not in the federate.
                    «ENDIF»
                    «ENDIF»
                    «reactionRef».last_enabling_reaction = «dominatingRef»;
                }
            ''')
           endScopedBlock(code);
        } else if (end == start + 1) {
            val reactionRef = CUtil.reactionRef(runtime.reaction, "" + start);
            if (runtime.dominating !== null
                && (domDivisor == 1 || domStart/domDivisor == currentFederate.bankIndex)
            ) {
                dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.reaction, "" + domStart) + ")";
            }
            if (!isFederated 
                || (start/divisor == currentFederate.bankIndex) 
                && (runtime.dominating === null || domStart/domDivisor == currentFederate.bankIndex)
            ) {
                code.pr('''
                    // «runtime.reaction.getFullName» dominating upstream reaction.
                    «reactionRef».last_enabling_reaction = «dominatingRef»;
                ''')
            }
        }
    }
    
    /**
     * Generate code to allocate the memory needed by reactions for triggering
     * downstream reactions.
     * @param reactions A list of reactions.
     */
    private def void deferredReactionMemory(Iterable<ReactionInstance> reactions) {
        // For each reaction instance, allocate the arrays that will be used to
        // trigger downstream reactions.
        for (reaction : reactions) {
            deferredReactionOutputs(reaction);

            val reactorSelfStruct = CUtil.reactorRef(reaction.parent);
            
            // Next handle triggers of the reaction that come from a multiport output
            // of a contained reactor.  Also, handle startup and shutdown triggers.
            for (trigger : reaction.triggers.filter(PortInstance)) {
                // If the port is a multiport, then we need to create an entry for each
                // individual port.
                if (trigger.isMultiport() && trigger.parent !== null && trigger.isOutput) {
                    // Trigger is an output of a contained reactor or bank.
                    code.pr('''
                        // Allocate memory to store pointers to the multiport output «trigger.name» 
                        // of a contained reactor «trigger.parent.getFullName»
                    ''')
                    startScopedBlock(code, trigger.parent, true);
                    
                    val width = trigger.width;
                    val portStructType = variableStructType(trigger)

                    code.pr('''
                        «CUtil.reactorRefNested(trigger.parent)».«trigger.name»_width = «width»;
                        «CUtil.reactorRefNested(trigger.parent)».«trigger.name»
                                = («portStructType»**)_lf_allocate(
                                        «width», sizeof(«portStructType»*),
                                        &«reactorSelfStruct»->base.allocations); 
                    ''')
                    
                    endScopedBlock(code);
                }
            }
        }
    }
    
    /**
     * For the specified reaction, for ports that it writes to,
     * set up the arrays that store the results (if necessary) and
     * that are used to trigger downstream reactions if an effect is actually
     * produced.  The port may be an output of the reaction's parent
     * or an input to a reactor contained by the parent.
     * 
     * @param The reaction instance.
     */
    private def void deferredReactionOutputs(ReactionInstance reaction) {
        // val selfRef = CUtil.reactorRef(reaction.parent);
        val name = reaction.parent.getFullName;
        // Insert a string name to facilitate debugging.                 
        if (targetConfig.logLevel >= LogLevel.LOG) {
            code.pr('''
                «CUtil.reactionRef(reaction)».name = "«name» reaction «reaction.index»";
            ''')
        }

        val reactorSelfStruct = CUtil.reactorRef(reaction.parent);

        // Count the output ports and inputs of contained reactors that
        // may be set by this reaction. This ignores actions in the effects.
        // Collect initialization statements for the output_produced array for the reaction
        // to point to the is_present field of the appropriate output.
        // These statements must be inserted after the array is malloc'd,
        // but we construct them while we are counting outputs.
        var outputCount = 0;
        val init = new CodeBuilder()

        startScopedBlock(init);
        init.pr("int count = 0;")
        for (effect : reaction.effects.filter(PortInstance)) {
            // Create the entry in the output_produced array for this port.
            // If the port is a multiport, then we need to create an entry for each
            // individual channel.
            
            // If the port is an input of a contained reactor, then, if that
            // contained reactor is a bank, we will have to iterate over bank
            // members.
            var bankWidth = 1;
            var portRef = "";
            if (effect.isInput) {
                init.pr("// Reaction writes to an input of a contained reactor.")
                bankWidth = effect.parent.width;
                startScopedBlock(init, effect.parent, true);
                portRef = CUtil.portRefNestedName(effect);
            } else {
                startScopedBlock(init);
                portRef = CUtil.portRefName(effect);
            }
            
            if (effect.isMultiport()) {
                // Form is slightly different for inputs vs. outputs.
                var connector = ".";
                if (effect.isInput) connector = "->";
                
                // Point the output_produced field to where the is_present field of the port is.
                init.pr('''
                    for (int i = 0; i < «effect.width»; i++) {
                        «CUtil.reactionRef(reaction)».output_produced[i + count]
                                = &«portRef»[i]«connector»is_present;
                    }
                    count += «effect.getWidth()»;
                ''')
                outputCount += effect.width * bankWidth;
            } else {
                // The effect is not a multiport.
                init.pr('''
                    «CUtil.reactionRef(reaction)».output_produced[count++]
                            = &«portRef».is_present;
                ''')
                outputCount += bankWidth;
            }
            endScopedBlock(init);
        }
        endScopedBlock(init);
        code.pr('''
            // Total number of outputs (single ports and multiport channels)
            // produced by «reaction.toString».
            «CUtil.reactionRef(reaction)».num_outputs = «outputCount»;
        ''')
        if (outputCount > 0) {
            code.pr('''
                // Allocate memory for triggers[] and triggered_sizes[] on the reaction_t
                // struct for this reaction.
                «CUtil.reactionRef(reaction)».triggers = (trigger_t***)_lf_allocate(
                        «outputCount», sizeof(trigger_t**),
                        &«reactorSelfStruct»->base.allocations); 
                «CUtil.reactionRef(reaction)».triggered_sizes = (int*)_lf_allocate(
                        «outputCount», sizeof(int),
                        &«reactorSelfStruct»->base.allocations); 
                «CUtil.reactionRef(reaction)».output_produced = (bool**)_lf_allocate(
                        «outputCount», sizeof(bool*),
                        &«reactorSelfStruct»->base.allocations); 
            ''')
        }
        
        code.pr('''
            «init.toString»
            // ** End initialization for reaction «reaction.index» of «name»
        ''')
    }
    
    /**
     * For the specified reaction, for ports that it writes to,
     * fill the trigger table for triggering downstream reactions.
     * 
     * @param reactions The reactions.
     */
    private def void deferredFillTriggerTable(Iterable<ReactionInstance> reactions) {
        for (reaction: reactions) {
            val name = reaction.parent.getFullName;
            
            val reactorSelfStruct = CUtil.reactorRef(reaction.parent, sr);

            var foundPort = false;
            
            for (port : reaction.effects.filter(PortInstance)) {
                if (!foundPort) {
                    // Need a separate index for the triggers array for each bank member.
                    startScopedBlock(code);
                    code.pr('''
                        int triggers_index[«reaction.parent.totalWidth»] = { 0 }; // Number of bank members with the reaction.
                    ''')
                    foundPort = true;
                }
                // If the port is a multiport, then its channels may have different sets
                // of destinations. For ordinary ports, there will be only one range and
                // its width will be 1.
                // We generate the code to fill the triggers array first in a temporary code buffer,
                // so that we can simultaneously calculate the size of the total array.
                for (SendRange srcRange : port.eventualDestinations()) {
                    val srcNested = (port.isInput)? true : false;
                    startScopedRangeBlock(code, srcRange, sr, sb, sc, srcNested, true);
                    
                    var triggerArray = '''«CUtil.reactionRef(reaction, sr)».triggers[triggers_index[«sr»]++]'''
                    // Skip ports whose parent is not in the federation.
                    // This can happen with reactions in the top-level that have
                    // as an effect a port in a bank.
                    if (currentFederate.contains(port.parent)) {
                        code.pr('''
                            // Reaction «reaction.index» of «name» triggers «srcRange.destinations.size» downstream reactions
                            // through port «port.getFullName».
                            «CUtil.reactionRef(reaction, sr)».triggered_sizes[triggers_index[«sr»]] = «srcRange.destinations.size»;
                            // For reaction «reaction.index» of «name», allocate an
                            // array of trigger pointers for downstream reactions through port «port.getFullName»
                            trigger_t** trigger_array = (trigger_t**)_lf_allocate(
                                    «srcRange.destinations.size», sizeof(trigger_t*),
                                    &«reactorSelfStruct»->base.allocations); 
                            «triggerArray» = trigger_array;
                        ''')
                    } else {
                        // Port is not in the federate or has no destinations.
                        // Set the triggered_width fields to 0.
                        code.pr('''
                            «CUtil.reactionRef(reaction, sr)».triggered_sizes[«sc»] = 0;
                        ''')
                    }
                    endScopedRangeBlock(code, srcRange);
                }
            }
            var cumulativePortWidth = 0;
            for (port : reaction.effects.filter(PortInstance)) {
                code.pr('''
                    for (int i = 0; i < «reaction.parent.totalWidth»; i++) triggers_index[i] = «cumulativePortWidth»;
                ''')
                for (SendRange srcRange : port.eventualDestinations()) {
                    if (currentFederate.contains(port.parent)) {
                        val srcNested = srcRange.instance.isInput;
                        var multicastCount = 0;
                        for (dstRange : srcRange.destinations) {
                            val dst = dstRange.instance;
                                                        
                            startScopedRangeBlock(code, srcRange, dstRange);
                            
                            // If the source is nested, need to take into account the parent's bank index
                            // when indexing into the triggers array.
                            var triggerArray = "";
                            if (srcNested && port.parent.width > 1 && !(isFederated && port.parent.depth == 1)) {
                                triggerArray = '''
                                    «CUtil.reactionRef(reaction, sr)».triggers[triggers_index[«sr»] + «sc» + src_range_mr.digits[1] * src_range_mr.radixes[0]]
                                '''
                            } else {
                                triggerArray = '''«CUtil.reactionRef(reaction, sr)».triggers[triggers_index[«sr»] + «sc»]'''
                            }
                                                                                        
                            if (dst.isOutput) {
                                // Include this destination port only if it has at least one
                                // reaction in the federation.
                                var belongs = false;
                                for (destinationReaction : dst.dependentReactions) {
                                    if (currentFederate.contains(destinationReaction.parent)) {
                                        belongs = true
                                    }
                                }
                                if (belongs) {
                                    code.pr('''
                                        // Port «port.getFullName» has reactions in its parent's parent.
                                        // Point to the trigger struct for those reactions.
                                        «triggerArray»[«multicastCount»] = &«CUtil.triggerRefNested(dst, dr, db)»;
                                    ''')
                                } else {
                                    // Put in a NULL pointer.
                                    code.pr('''
                                        // Port «port.getFullName» has reactions in its parent's parent.
                                        // But those are not in the federation.
                                        «triggerArray»[«multicastCount»] = NULL;
                                    ''')
                                }
                            } else {
                                // Destination is an input port.
                                code.pr('''
                                    // Point to destination port «dst.getFullName»'s trigger struct.
                                    «triggerArray»[«multicastCount»] = &«CUtil.triggerRef(dst, dr)»;
                                ''')
                            }
                            endScopedRangeBlock(code, srcRange, dstRange);
                            multicastCount++;
                        }
                    }
                }
                cumulativePortWidth += port.width;
            }
            if (foundPort) endScopedBlock(code);
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
