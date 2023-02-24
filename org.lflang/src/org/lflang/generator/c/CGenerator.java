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

import static org.lflang.ASTUtils.allActions;
import static org.lflang.ASTUtils.allPorts;
import static org.lflang.ASTUtils.allReactions;
import static org.lflang.ASTUtils.allStateVars;
import static org.lflang.ASTUtils.convertToEmptyListIfNull;
import static org.lflang.ASTUtils.getInferredType;
import static org.lflang.ASTUtils.isInitialized;
import static org.lflang.ASTUtils.toDefinition;
import static org.lflang.ASTUtils.toText;
import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.StringExtensions;

import org.lflang.ASTUtils;
import org.lflang.generator.DockerComposeGenerator;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.Platform;

import org.lflang.federated.extensions.CExtensionUtils;

import org.lflang.ast.DelayedConnectionTransformation;

import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DockerGenerator;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.GeneratorUtils;

import org.lflang.generator.DelayBodyGenerator;

import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.LFResource;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.TimerInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Code;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;
import org.lflang.lf.Variable;
import org.lflang.util.ArduinoUtil;
import org.lflang.util.FileUtil;

import com.google.common.base.Objects;
import com.google.common.collect.Iterables;

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
 * has fields `is_present` and `value`, where the type of `value` matches the port type.
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
 *   can access state variables as `self->s`.
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
 *   set when connections are made or changed. It is used to determine for
 *   a mutable input destination whether a copy needs to be made.
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
 * * _lf_shutdown_triggers: An array of pointers to trigger_t structs for shutdown
 *   reactions. The length of this table is in the _lf_shutdown_triggers_size
 *   variable.
 *
 * * _lf_timer_triggers: An array of pointers to trigger_t structs for timers that
 *   need to be started when the program runs. The length of this table is in the
 *   _lf_timer_triggers_size variable.
 *
 * * _lf_action_table: For a federated execution, each federate will have this table
 *   that maps port IDs to the corresponding action struct, which can be cast to
 *   action_base_t.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Mehrdad Niknami
 * @author Christian Menard
 * @author Matt Weber
 * @author Soroush Bateni
 * @author Alexander Schulz-Rosengarten
 * @author Hou Seng Wong
 * @author Anirudh Rengarajan
 */
@SuppressWarnings("StaticPseudoFunctionalStyleMethod")
public class CGenerator extends GeneratorBase {
    // Regular expression pattern for compiler error messages with resource
    // and line number information. The first match will a resource URI in the
    // form of "file:/path/file.lf". The second match will be a line number.
    // The third match is a character position within the line.
    // The fourth match will be the error message.
    static final Pattern compileErrorPattern = Pattern.compile(
        "^(?<path>.*):(?<line>\\d+):(?<column>\\d+):(?<message>.*)$"
    );

    public static int UNDEFINED_MIN_SPACING = -1;

    ////////////////////////////////////////////
    //// Protected fields

    /** The main place to put generated code. */
    protected CodeBuilder code = new CodeBuilder();

    /** Place to collect code to initialize the trigger objects for all reactor instances. */
    protected CodeBuilder initializeTriggerObjects = new CodeBuilder();

    protected final CFileConfig fileConfig;

    /**
     * Count of the number of is_present fields of the self struct that
     * need to be reinitialized in _lf_start_time_step().
     */
    protected int startTimeStepIsPresentCount = 0;

    ////////////////////////////////////////////
    //// Private fields
    /**
     * Extra lines that need to go into the generated CMakeLists.txt.
     */
    private String cMakeExtras = "";

    /** Place to collect code to execute at the start of a time step. */
    private CodeBuilder startTimeStep = new CodeBuilder();

    /** Count of the number of token pointers that need to have their
     *  reference count decremented in _lf_start_time_step().
     */
    private int timerCount = 0;
    private int startupReactionCount = 0;
    private int shutdownReactionCount = 0;
    private int resetReactionCount = 0;
    private int modalReactorCount = 0;
    private int modalStateResetCount = 0;

    // Indicate whether the generator is in Cpp mode or not
    private final boolean CCppMode;

    private final CTypes types;

    private final CCmakeGenerator cmakeGenerator;

    protected CGenerator(
        LFGeneratorContext context,
        boolean CCppMode,
        CTypes types,
        CCmakeGenerator cmakeGenerator,
        DelayBodyGenerator delayBodyGenerator
    ) {
        super(context);
        this.fileConfig = (CFileConfig) context.getFileConfig();
        this.CCppMode = CCppMode;
        this.types = types;
        this.cmakeGenerator = cmakeGenerator;

        // Register the delayed connection transformation to be applied by GeneratorBase.
        // transform both after delays and physical connections
        registerTransformation(new DelayedConnectionTransformation(delayBodyGenerator, types, fileConfig.resource, true, true));
    }

    public CGenerator(LFGeneratorContext context, boolean ccppMode) {
        this(
            context,
            ccppMode,
            new CTypes(context.getErrorReporter()),
            new CCmakeGenerator(context.getFileConfig(), List.of()),
            new CDelayBodyGenerator(new CTypes(context.getErrorReporter()))
        );
    }

    /**
     * Look for physical actions in all resources.
     * If found, set threads to be at least one to allow asynchronous schedule calls.
     */
    public void accommodatePhysicalActionsIfPresent() {
        // If there are any physical actions, ensure the threaded engine is used and that
        // keepalive is set to true, unless the user has explicitly set it to false.
        for (Resource resource : GeneratorUtils.getResources(reactors)) {
            for (Action action : ASTUtils.allElementsOfClass(resource, Action.class)) {
                if (Objects.equal(action.getOrigin(), ActionOrigin.PHYSICAL)) {
                    // If the unthreaded runtime is not requested by the user, use the threaded runtime instead
                    // because it is the only one currently capable of handling asynchronous events.
                    if (!targetConfig.threading && !targetConfig.setByUser.contains(TargetProperty.THREADING)) {
                        targetConfig.threading = true;
                        errorReporter.reportWarning(
                            action,
                            "Using the threaded C runtime to allow for asynchronous handling of physical action " +
                            action.getName()
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
    protected boolean isOSCompatible() {
        if (GeneratorUtils.isHostWindows()) {
            if (CCppMode) {
                errorReporter.reportError(
                    "LF programs with a CCpp target are currently not supported on Windows. " +
                    "Exiting code generation."
                );
                // FIXME: The incompatibility between our C runtime code and the
                //  Visual Studio compiler is extensive.
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
    @Override
    public void doGenerate(Resource resource, LFGeneratorContext context) {
        super.doGenerate(resource, context);
        if (!GeneratorUtils.canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return;
        if (!isOSCompatible()) return; // Incompatible OS and configuration

        // Perform set up that does not generate code
        setUpGeneralParameters();

        FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSrcGenPath().toFile());
        FileUtil.createDirectoryIfDoesNotExist(fileConfig.binPath.toFile());
        FileUtil.createDirectoryIfDoesNotExist(fileConfig.getIncludePath().toFile());
        handleProtoFiles();

        // Derive target filename from the .lf filename.
        var lfModuleName = fileConfig.name;
        var cFilename = CCompiler.getTargetFileName(lfModuleName, this.CCppMode, targetConfig);
        var targetFile = fileConfig.getSrcGenPath() + File.separator + cFilename;
        try {
                generateCodeFor(lfModuleName);


                String srcPrefix = targetConfig.platformOptions.platform == Platform.ARDUINO ? "src/" : "";

                // Copy the core lib
                FileUtil.copyDirectoryFromClassPath(
                    "/lib/c/reactor-c/core",
                    fileConfig.getSrcGenPath().resolve(srcPrefix + "core"),
                    true
                );
                // Copy the C target files
                copyTargetFiles();

                // For the Zephyr target, copy default config and board files.
                if (targetConfig.platformOptions.platform == Platform.ZEPHYR) {
                    FileUtil.copyDirectoryFromClassPath(
                        "/lib/platform/zephyr/boards",
                        fileConfig.getSrcGenPath().resolve("boards"),
                        false
                    );
                    FileUtil.copyFileFromClassPath(
                        "/lib/platform/zephyr/prj_lf.conf",
                        fileConfig.getSrcGenPath().resolve("prj_lf.conf"),
                        true
                    );

                    FileUtil.copyFileFromClassPath(
                        "/lib/platform/zephyr/Kconfig",
                        fileConfig.getSrcGenPath().resolve("Kconfig"),
                        true
                    );
                }

                generateHeaders();

                // Write the generated code
                code.writeToFile(targetFile);
            } catch (IOException e) {
                //noinspection ThrowableNotThrown,ResultOfMethodCallIgnored
                Exceptions.sneakyThrow(e);
            }

            // Create docker file.
            if (targetConfig.dockerOptions != null && mainDef != null) {
                try {
                    var dockerData = getDockerGenerator(context).generateDockerData();
                    dockerData.writeDockerFile();
                    (new DockerComposeGenerator(context)).writeDockerComposeFile(List.of(dockerData));
                } catch (IOException e) {
                    throw new RuntimeException("Error while writing Docker files", e);
                }
            }

            // If cmake is requested, generate the CMakeLists.txt
            if (targetConfig.platformOptions.platform != Platform.ARDUINO) {
                var cmakeFile = fileConfig.getSrcGenPath() + File.separator + "CMakeLists.txt";
                var sources = reactors.stream().map(CUtil::getName).map(it -> it + ".c").collect(Collectors.toList());
                sources.add(cFilename);
                var cmakeCode = cmakeGenerator.generateCMakeCode(
                    sources,
                    lfModuleName,
                    errorReporter,
                    CCppMode,
                    mainDef != null,
                    cMakeExtras,
                    targetConfig
                );
                try {
                    cmakeCode.writeToFile(cmakeFile);
                } catch (IOException e) {
                    //noinspection ThrowableNotThrown,ResultOfMethodCallIgnored
                    Exceptions.sneakyThrow(e);
                }
            } else {
                try {
                    Path include = fileConfig.getSrcGenPath().resolve("include/");
                    Path src = fileConfig.getSrcGenPath().resolve("src/");
                    FileUtil.arduinoDeleteHelper(src, targetConfig.threading);
                    FileUtil.relativeIncludeHelper(src, include);
                    FileUtil.relativeIncludeHelper(include, include);
                } catch (IOException e) {
                    //noinspection ThrowableNotThrown,ResultOfMethodCallIgnored
                    Exceptions.sneakyThrow(e);
                }

                if (!targetConfig.noCompile) {
                    ArduinoUtil arduinoUtil = new ArduinoUtil(context, commandFactory, errorReporter);
                    arduinoUtil.buildArduino(fileConfig, targetConfig);
                    context.finish(
                        GeneratorResult.Status.COMPILED, null
                    );
                } else {
                    System.out.println("********");
                    System.out.println("To compile your program, run the following command to see information about the board you plugged in:\n\n\tarduino-cli board list\n\nGrab the FQBN and PORT from the command and run the following command in the generated sources directory:\n\n\tarduino-cli compile -b <FQBN> --build-property compiler.c.extra_flags='-DLF_UNTHREADED -DPLATFORM_ARDUINO -DINITIAL_EVENT_QUEUE_SIZE=10 -DINITIAL_REACT_QUEUE_SIZE=10' --build-property compiler.cpp.extra_flags='-DLF_UNTHREADED -DPLATFORM_ARDUINO -DINITIAL_EVENT_QUEUE_SIZE=10 -DINITIAL_REACT_QUEUE_SIZE=10' .\n\nTo flash/upload your generated sketch to the board, run the following command in the generated sources directory:\n\n\tarduino-cli upload -b <FQBN> -p <PORT>\n");
                    // System.out.println("For a list of all boards installed on your computer, you can use the following command:\n\n\tarduino-cli board listall\n");
                    context.finish(
                        GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, null)
                    );
                }
                GeneratorUtils.refreshProject(resource, context.getMode());
                return;
            }

        // Dump the additional compile definitions to a file to keep the generated project
        // self-contained. In this way, third-party build tools like PlatformIO, west, arduino-cli can
        // take over and do the rest of compilation.
        try {
            String compileDefs = targetConfig.compileDefinitions.keySet().stream()
                                                                .map(key -> key + "=" + targetConfig.compileDefinitions.get(key))
                                                                .collect(Collectors.joining("\n"));
            FileUtil.writeToFile(
                compileDefs,
                Path.of(fileConfig.getSrcGenPath() + File.separator + "CompileDefinitions.txt")
            );
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // If this code generator is directly compiling the code, compile it now so that we
        // clean it up after, removing the #line directives after errors have been reported.
        if (
            !targetConfig.noCompile && targetConfig.dockerOptions == null
                && IterableExtensions.isNullOrEmpty(targetConfig.buildCommands)
                // This code is unreachable in LSP_FAST mode, so that check is omitted.
                && context.getMode() != LFGeneratorContext.Mode.LSP_MEDIUM
        ) {
            // FIXME: Currently, a lack of main is treated as a request to not produce
            // a binary and produce a .o file instead. There should be a way to control
            // this.
            // Create an anonymous Runnable class and add it to the compileThreadPool
            // so that compilation can happen in parallel.
            var cleanCode = code.removeLines("#line");

            var execName = lfModuleName;
            var threadFileConfig = fileConfig;
            var generator = this; // FIXME: currently only passed to report errors with line numbers in the Eclipse IDE
            var CppMode = CCppMode;
            // generatingContext.reportProgress(
            //     String.format("Generated code for %d/%d executables. Compiling...", federateCount, federates.size()),
            //     100 * federateCount / federates.size()
            // ); // FIXME: Move to FedGenerator
            // Create the compiler to be used later

            var cCompiler = new CCompiler(targetConfig, threadFileConfig, errorReporter, CppMode);
            try {
                if (!cCompiler.runCCompiler(generator, context)) {
                    // If compilation failed, remove any bin files that may have been created.
                    CUtil.deleteBinFiles(threadFileConfig);
                    // If finish has already been called, it is illegal and makes no sense. However,
                    //  if finish has already been called, then this must be a federated execution.
                    context.unsuccessfulFinish();
                } else {
                    context.finish(
                        GeneratorResult.Status.COMPILED, null
                    );
                }
                cleanCode.writeToFile(targetFile);
            } catch (IOException e) {
                Exceptions.sneakyThrow(e);
            }

        }

        // If a build directive has been given, invoke it now.
        // Note that the code does not get cleaned in this case.
        if (!targetConfig.noCompile) {
            if (!IterableExtensions.isNullOrEmpty(targetConfig.buildCommands)) {
                CUtil.runBuildCommand(
                    fileConfig,
                    targetConfig,
                    commandFactory,
                    errorReporter,
                    this::reportCommandErrors,
                    context.getMode()
                );
                context.finish(
                    GeneratorResult.Status.COMPILED, null
                );
            }
            System.out.println("Compiled binary is in " + fileConfig.binPath);
        } else {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, null));
        }

        // In case we are in Eclipse, make sure the generated code is visible.
        GeneratorUtils.refreshProject(resource, context.getMode());
    }

    private void generateCodeFor(
        String lfModuleName
    ) throws IOException {
        startTimeStepIsPresentCount = 0;
        code.pr(generateDirectives());
        code.pr(generateTopLevelPreambles());
        code.pr(new CMainFunctionGenerator(targetConfig).generateCode());
        // Generate code for each reactor.
        generateReactorDefinitions();

        // Generate main instance, if there is one.
        // Note that any main reactors in imported files are ignored.
        // Skip generation if there are cycles.
        if (main != null) {
            initializeTriggerObjects.pr(String.join("\n",
                "int _lf_startup_reactions_count = 0;",
                "SUPPRESS_UNUSED_WARNING(_lf_startup_reactions_count);",
                "int _lf_shutdown_reactions_count = 0;",
                "SUPPRESS_UNUSED_WARNING(_lf_shutdown_reactions_count);",
                "int _lf_reset_reactions_count = 0;",
                "SUPPRESS_UNUSED_WARNING(_lf_reset_reactions_count);",
                "int _lf_timer_triggers_count = 0;",
                "SUPPRESS_UNUSED_WARNING(_lf_timer_triggers_count);",
                "int bank_index;",
                "SUPPRESS_UNUSED_WARNING(bank_index);"
            ));
            // Add counters for modal initialization
            initializeTriggerObjects.pr(CModesGenerator.generateModalInitalizationCounters(hasModalReactors));

            // Create an array of arrays to store all self structs.
            // This is needed because connections cannot be established until
            // all reactor instances have self structs because ports that
            // receive data reference the self structs of the originating
            // reactors, which are arbitarily far away in the program graph.
            generateSelfStructs(main);
            generateReactorInstance(main);

            // If there are timers, create a table of timers to be initialized.
            code.pr(CTimerGenerator.generateDeclarations(timerCount));

            // If there are startup reactions, create a table of triggers.
            code.pr(CReactionGenerator.generateBuiltinTriggersTable(startupReactionCount, "startup"));

            // If there are shutdown reactions, create a table of triggers.
            code.pr(CReactionGenerator.generateBuiltinTriggersTable(shutdownReactionCount, "shutdown"));

            // If there are reset reactions, create a table of triggers.
            code.pr(CReactionGenerator.generateBuiltinTriggersTable(resetReactionCount, "reset"));

            // If there are modes, create a table of mode state to be checked for transitions.
            code.pr(CModesGenerator.generateModeStatesTable(
                hasModalReactors,
                modalReactorCount,
                modalStateResetCount
            ));

            // Generate function to initialize the trigger objects for all reactors.
            code.pr(CTriggerObjectsGenerator.generateInitializeTriggerObjects(
                main,
                targetConfig,
                initializeTriggerObjects,
                startTimeStep,
                types,
                lfModuleName,
                startTimeStepIsPresentCount
            ));

            // Generate function to trigger startup reactions for all reactors.
            code.pr(CReactionGenerator.generateLfTriggerStartupReactions(startupReactionCount, hasModalReactors));

            // Generate function to schedule timers for all reactors.
            code.pr(CTimerGenerator.generateLfInitializeTimer(timerCount));

            // Generate a function that will either do nothing
            // (if there is only one federate or the coordination
            // is set to decentralized) or, if there are
            // downstream federates, will notify the RTI
            // that the specified logical time is complete.
            if (CCppMode || targetConfig.platformOptions.platform == Platform.ARDUINO) code.pr("extern \"C\"");
            code.pr(String.join("\n",
                "void logical_tag_complete(tag_t tag_to_send) {",
                CExtensionUtils.surroundWithIfFederatedCentralized(
                "        _lf_logical_tag_complete(tag_to_send);"
                ),
                "}"
            ));

            // Generate function to schedule shutdown reactions if any
            // reactors have reactions to shutdown.
            code.pr(CReactionGenerator.generateLfTriggerShutdownReactions(shutdownReactionCount, hasModalReactors));

            // Generate an empty termination function for non-federated
            // execution. For federated execution, an implementation is
            // provided in federate.c.  That implementation will resign
            // from the federation and close any open sockets.
            code.pr("""
                #ifndef FEDERATED
                void terminate_execution() {}
                #endif"""
            );


            // Generate functions for modes
            code.pr(CModesGenerator.generateLfInitializeModes(
                hasModalReactors
            ));
            code.pr(CModesGenerator.generateLfHandleModeChanges(
                hasModalReactors,
                modalStateResetCount
            ));
            code.pr(CReactionGenerator.generateLfModeTriggeredReactions(
                startupReactionCount,
                resetReactionCount,
                hasModalReactors
            ));
        }
    }

    @Override
    public void checkModalReactorSupport(boolean __) {
        // Modal reactors are currently only supported for non federated applications
        super.checkModalReactorSupport(true);
    }

    @Override
    protected String getConflictingConnectionsInModalReactorsBody(String source, String dest) {
        return String.join("\n",
            "// Generated forwarding reaction for connections with the same destination",
            "// but located in mutually exclusive modes.",
            "lf_set("+dest+", "+source+"->value);"
        );
    }

    /** Set the scheduler type in the target config as needed. */
    private void pickScheduler() {
        // Don't use a scheduler that does not prioritize reactions based on deadlines
        // if the program contains a deadline (handler). Use the GEDF_NP scheduler instead.
        if (!targetConfig.schedulerType.prioritizesDeadline()) {
            // Check if a deadline is assigned to any reaction
            if (hasDeadlines(reactors)) {
                if (!targetConfig.setByUser.contains(TargetProperty.SCHEDULER)) {
                    targetConfig.schedulerType = TargetProperty.SchedulerOption.GEDF_NP;
                }
            }
        }
    }

    private boolean hasDeadlines(List<Reactor> reactors) {
        for (Reactor reactor : reactors) {
            for (Reaction reaction : allReactions(reactor)) {
                if (reaction.getDeadline() != null) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Look at the 'reactor' eResource.
     * If it is an imported .lf file, incorporate it into the current
     * program in the following manner:
     * - Merge its target property with `targetConfig`
     * - If there are any preambles, add them to the preambles of the reactor.
     */
    private void inspectReactorEResource(ReactorDecl reactor) {
        // If the reactor is imported, look at the
        // target definition of the .lf file in which the reactor is imported from and
        // append any cmake-include.
        // Check if the reactor definition is imported
        if (reactor.eResource() != mainDef.getReactorClass().eResource()) {
            // Find the LFResource corresponding to this eResource
            LFResource lfResource = null;
            for (var resource : resources) {
                if (resource.getEResource() == reactor.eResource()) {
                    lfResource = resource;
                    break;
                }
            }
            // Copy the user files and cmake-includes to the src-gen path of the main .lf file
            if (lfResource != null) {
                copyUserFiles(lfResource.getTargetConfig(), lfResource.getFileConfig());
            }
            // Extract the contents of the imported file for the preambles
            var contents = toDefinition(reactor).eResource().getContents();
            var model = (Model) contents.get(0);
            // Add the preambles from the imported .lf file
            toDefinition(reactor).getPreambles().addAll(model.getPreambles());
        }
    }

    /**
     * Copy all files or directories listed in the target property `files`, `cmake-include`,
     * and `_fed_setup` into the src-gen folder of the main .lf file
     *
     * @param targetConfig The targetConfig to read the target properties from.
     * @param fileConfig The fileConfig used to make the copy and resolve paths.
     */
    @Override
    public void copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {
        super.copyUserFiles(targetConfig, fileConfig);
        // Make sure the target directory exists.
        var targetDir = this.fileConfig.getSrcGenPath();
        try {
            Files.createDirectories(targetDir);
        } catch (IOException e) {
            //noinspection ThrowableNotThrown,ResultOfMethodCallIgnored
            Exceptions.sneakyThrow(e);
        }

        for (String filename : targetConfig.fileNames) {
            var relativeFileName = CUtil.copyFileOrResource(
                    filename,
                    fileConfig.srcFile.getParent(),
                    targetDir);
            if (StringExtensions.isNullOrEmpty(relativeFileName)) {
                errorReporter.reportError(
                    "Failed to find file " + filename + " specified in the" +
                    " files target property."
                );
            } else {
                targetConfig.filesNamesWithoutPath.add(
                    relativeFileName
                );
            }
        }

        for (String filename : targetConfig.cmakeIncludes) {
            var relativeCMakeIncludeFileName =
                CUtil.copyFileOrResource(
                    filename,
                    fileConfig.srcFile.getParent(),
                    targetDir);
            // Check if the file exists
            if (StringExtensions.isNullOrEmpty(relativeCMakeIncludeFileName)) {
                errorReporter.reportError(
                    "Failed to find cmake-include file " + filename
                );
            } else {
                this.targetConfig.cmakeIncludesWithoutPath.add(
                    relativeCMakeIncludeFileName
                );
            }
        }

        if (!StringExtensions.isNullOrEmpty(targetConfig.fedSetupPreamble)) {
            try {
                FileUtil.copyFile(fileConfig.srcFile.getParent().resolve(targetConfig.fedSetupPreamble),
                                  targetDir.resolve(targetConfig.fedSetupPreamble));
            } catch (IOException e) {
                errorReporter.reportError("Failed to find _fed_setup file " + targetConfig.fedSetupPreamble);
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
    private void generateReactorDefinitions() throws IOException {
        var generatedReactors = new LinkedHashSet<Reactor>();
        if (this.main != null) {
            generateReactorChildren(this.main, generatedReactors);
        }

        if (this.mainDef != null) {
            generateReactorClass(ASTUtils.toDefinition(this.mainDef.getReactorClass()));
        }

        if (mainDef == null) {
            // Generate code for each reactor that was not instantiated in main or its children.
            for (Reactor r : reactors) {
                // Get the declarations for reactors that are instantiated somewhere.
                // A declaration is either a reactor definition or an import statement.;
                var declarations = this.instantiationGraph.getDeclarations(r);
                // If the reactor has no instantiations and there is no main reactor, then
                // generate code for it anyway (at a minimum, this means that the compiler is invoked
                // so that reaction bodies are checked).
                if (declarations.isEmpty()) {
                    generateReactorClass(r);
                }
            }
        }
    }

    private void generateHeaders() throws IOException {
        FileUtil.deleteDirectory(fileConfig.getIncludePath());
        FileUtil.copyDirectoryFromClassPath(
            fileConfig.getRuntimeIncludePath(),
            fileConfig.getIncludePath(),
            false
        );
        for (Reactor r : reactors) {
            CReactorHeaderFileGenerator.doGenerate(types, r, fileConfig, this::generateAuxiliaryStructs);
        }
        FileUtil.copyDirectory(fileConfig.getIncludePath(), fileConfig.getSrcGenPath().resolve("include"), false);
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
    private void generateReactorChildren(
        ReactorInstance reactor,
        LinkedHashSet<Reactor> generatedReactors
    ) throws IOException {
        for (ReactorInstance r : reactor.children) {
            if (r.reactorDeclaration != null &&
                  !generatedReactors.contains(r.reactorDefinition)) {
                generatedReactors.add(r.reactorDefinition);
                generateReactorChildren(r, generatedReactors);
                inspectReactorEResource(r.reactorDeclaration);
                generateReactorClass(r.reactorDefinition);
            }
        }
    }

    /**
     * Choose which platform files to compile with according to the OS.
     * If there is no main reactor, then compilation will produce a .o file requiring further linking.
     * Also, if useCmake is set to true, we don't need to add platform files. The CMakeLists.txt file
     * will detect and use the appropriate platform file based on the platform that cmake is invoked on.
     */
    private void pickCompilePlatform() {
        var osName = System.getProperty("os.name").toLowerCase();
        // if platform target was set, use given platform instead
        if (targetConfig.platformOptions.platform != Platform.AUTO) {
            osName = targetConfig.platformOptions.platform.toString();
        } else if (Stream.of("mac", "darwin", "win", "nux").noneMatch(osName::contains)) {
            errorReporter.reportError("Platform " + osName + " is not supported");
        }
    }


    /**
     * Copy target-specific header file to the src-gen directory.
     */
    protected void copyTargetFiles() throws IOException {

        String srcPrefix = targetConfig.platformOptions.platform == Platform.ARDUINO ? "src/" : "";

        FileUtil.copyDirectoryFromClassPath(
            "/lib/c/reactor-c/lib",
            fileConfig.getSrcGenPath().resolve(srcPrefix + "lib"),
            false
        );
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
    private void generateReactorClass(Reactor reactor) throws IOException {
        // FIXME: Currently we're not reusing definitions for declarations that point to the same definition.
        CodeBuilder header = new CodeBuilder();
        CodeBuilder src = new CodeBuilder();
        final String headerName = CUtil.getName(reactor) + ".h";
        header.pr("#include \"include/core/reactor.h\"");
        src.pr("#include \"" + headerName + "\"");
        src.pr("#include \"api.h\"");
        src.pr("#include \"set.h\"");
        generateIncludes(reactor);

        // Preamble code contains state declarations with static initializers.
        generateUserPreamblesForReactor(reactor);

        // Some of the following methods create lines of code that need to
        // go into the constructor.  Collect those lines of code here:
        var constructorCode = new CodeBuilder();
        generateSelfStruct(header, reactor, constructorCode);
        generateMethods(src, reactor);
        generateReactions(src, reactor);
        generateConstructor(src, header, reactor, constructorCode);
        FileUtil.writeToFile(header.toString(), fileConfig.getSrcGenPath().resolve(headerName), true);
        FileUtil.writeToFile(src.toString(), fileConfig.getSrcGenPath().resolve(CUtil.getName(reactor) + ".c"), true);
    }

    /**
     * Generate methods for {@code reactor}.
     */
    protected void generateMethods(CodeBuilder src, ReactorDecl reactor) {
        CMethodGenerator.generateMethods(reactor, src, types);
    }

    /**
     * Generates preambles defined by user for a given reactor
     * @param reactor The given reactor
     */
    protected void generateUserPreamblesForReactor(Reactor reactor) {
        for (Preamble p : convertToEmptyListIfNull(reactor.getPreambles())) {
            code.pr("// *********** From the preamble, verbatim:");
            code.prSourceLineNumber(p.getCode());
            code.pr(toText(p.getCode()));
            code.pr("\n// *********** End of preamble.");
        }
    }

    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    protected void generateConstructor(
        CodeBuilder src, CodeBuilder header, Reactor reactor, CodeBuilder constructorCode
    ) {
        header.pr(CConstructorGenerator.generateConstructorPrototype(reactor));
        src.pr(CConstructorGenerator.generateConstructor(
            reactor,
            constructorCode.toString()
        ));
    }

    protected void generateIncludes(Reactor r) {
        if (CCppMode) code.pr("extern \"C\" {");
        code.pr("#include \"" + CUtil.getName(r) + ".h\"");
        code.pr("#include \"include/" + CReactorHeaderFileGenerator.outputPath(fileConfig, r) + "\"");
        if (CCppMode) code.pr("}");
    }

    /**
     * Generate the struct type definitions for inputs, outputs, and
     * actions of the specified reactor.
     */
    protected void generateAuxiliaryStructs(CodeBuilder builder, Reactor r) {
        // In the case where there are incoming
        // p2p logical connections in decentralized
        // federated execution, there will be an
        // intended_tag field added to accommodate
        // the case where a reaction triggered by a
        // port or action is late due to network
        // latency, etc..
        var federatedExtension = new CodeBuilder();
        federatedExtension.pr("""
            #ifdef FEDERATED
            #ifdef FEDERATED_DECENTRALIZED
            %s intended_tag;
            #endif
            %s physical_time_of_arrival;
            #endif
            """.formatted(types.getTargetTagType(), types.getTargetTimeType())
        );
        for (Port p : allPorts(r)) {
            builder.pr(CPortGenerator.generateAuxiliaryStruct(
                r,
                p,
                getTarget(),
                errorReporter,
                types,
                federatedExtension
            ));
        }
        // The very first item on this struct needs to be
        // a trigger_t* because the struct will be cast to (trigger_t*)
        // by the lf_schedule() functions to get to the trigger.
        for (Action action : allActions(r)) {
            builder.pr(CActionGenerator.generateAuxiliaryStruct(
                r,
                action,
                getTarget(),
                types,
                federatedExtension
            ));
        }
    }

    /**
     * Generate the self struct type definition for the specified reactor
     * in the specified federate.
     * @param decl The parsed reactor data structure.
     * @param constructorCode Place to put lines of code that need to
     *  go into the constructor.
     */
    private void generateSelfStruct(CodeBuilder builder, ReactorDecl decl, CodeBuilder constructorCode) {
        var reactor = toDefinition(decl);
        var selfType = CUtil.selfType(ASTUtils.toDefinition(decl));

        // Construct the typedef for the "self" struct.
        // Create a type name for the self struct.
        var body = new CodeBuilder();

        // Extensions can add functionality to the CGenerator
        generateSelfStructExtension(body, decl, constructorCode);

        // Next handle parameters.
        body.pr(CParameterGenerator.generateDeclarations(reactor, types));

        // Next handle states.
        body.pr(CStateGenerator.generateDeclarations(reactor, types));

        // Next handle actions.
        CActionGenerator.generateDeclarations(reactor, body, constructorCode);

        // Next handle inputs and outputs.
        CPortGenerator.generateDeclarations(reactor, decl, body, constructorCode);

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
            body,
            reactor,
            constructorCode,
            types
        );

        // Next, generate fields for modes
        CModesGenerator.generateDeclarations(reactor, body, constructorCode);

        // The first field has to always be a pointer to the list of
        // of allocated memory that must be freed when the reactor is freed.
        // This means that the struct can be safely cast to self_base_t.
        builder.pr("typedef struct {");
        builder.indent();
        builder.pr("struct self_base_t base;");
        builder.pr(body.toString());
        builder.unindent();
        builder.pr("} " + selfType + ";");
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
    private void generateInteractingContainedReactors(
        Reactor reactor,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        // The contents of the struct will be collected first so that
        // we avoid duplicate entries and then the struct will be constructed.
        var contained = new InteractingContainedReactors(reactor);
        // Next generate the relevant code.
        for (Instantiation containedReactor : contained.containedReactors()) {
            Reactor containedReactorType = ASTUtils.toDefinition(containedReactor.getReactorClass());
            // First define an _width variable in case it is a bank.
            var array = "";
            var width = -2;
            // If the instantiation is a bank, find the maximum bank width
            // to define an array.
            if (containedReactor.getWidthSpec() != null) {
                width = CReactionGenerator.maxContainedReactorBankWidth(containedReactor, null, 0, mainDef);
                array = "[" + width + "]";
            }
            // NOTE: The following needs to be done for each instance
            // so that the width can be parameter, not in the constructor.
            // Here, we conservatively use a width that is the largest of all isntances.
            constructorCode.pr(String.join("\n",
                "// Set the _width variable for all cases. This will be -2",
                "// if the reactor is not a bank of reactors.",
                "self->_lf_"+containedReactor.getName()+"_width = "+width+";"
            ));

            // Generate one struct for each contained reactor that interacts.
            body.pr("struct {");
            body.indent();
            for (Port port : contained.portsOfInstance(containedReactor)) {
                if (port instanceof Input) {
                    // If the variable is a multiport, then the place to store the data has
                    // to be malloc'd at initialization.
                    if (!ASTUtils.isMultiport(port)) {
                        // Not a multiport.
                        body.pr(port, variableStructType(port, containedReactorType)+" "+port.getName()+";");
                    } else {
                        // Is a multiport.
                        // Memory will be malloc'd in initialization.
                        body.pr(port, String.join("\n",
                            variableStructType(port, containedReactorType)+"** "+port.getName()+";",
                            "int "+port.getName()+"_width;"
                        ));
                    }
                } else {
                    // Must be an output port.
                    // Outputs of contained reactors are pointers to the source of data on the
                    // self struct of the container.
                    if (!ASTUtils.isMultiport(port)) {
                        // Not a multiport.
                        body.pr(port, variableStructType(port, containedReactorType)+"* "+port.getName()+";");
                    } else {
                        // Is a multiport.
                        // Here, we will use an array of pointers.
                        // Memory will be malloc'd in initialization.
                        body.pr(port, String.join("\n",
                            variableStructType(port, containedReactorType)+"** "+port.getName()+";",
                            "int "+port.getName()+"_width;"
                        ));
                    }
                    body.pr(port, "trigger_t "+port.getName()+"_trigger;");
                    var reactorIndex = "";
                    if (containedReactor.getWidthSpec() != null) {
                        reactorIndex = "[reactor_index]";
                        constructorCode.pr("for (int reactor_index = 0; reactor_index < self->_lf_"+containedReactor.getName()+"_width; reactor_index++) {");
                        constructorCode.indent();
                    }
                    var portOnSelf = "self->_lf_"+containedReactor.getName()+reactorIndex+"."+port.getName();

                    constructorCode.pr(
                        port,
                        CExtensionUtils.surroundWithIfFederatedDecentralized(
                            portOnSelf+"_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};"
                        )
                    );

                    var triggered = contained.reactionsTriggered(containedReactor, port);
                    //noinspection StatementWithEmptyBody
                    if (triggered.size() > 0) {
                        body.pr(port, "reaction_t* "+port.getName()+"_reactions["+triggered.size()+"];");
                        var triggeredCount = 0;
                        for (Integer index : triggered) {
                            constructorCode.pr(port, portOnSelf+"_reactions["+triggeredCount+++"] = &self->_lf__reaction_"+index+";");
                        }
                        constructorCode.pr(port, portOnSelf+"_trigger.reactions = "+portOnSelf+"_reactions;");
                    } else {
                        // Since the self struct is created using calloc, there is no need to set
                        // self->_lf_"+containedReactor.getName()+"."+port.getName()+"_trigger.reactions = NULL
                    }
                    // Since the self struct is created using calloc, there is no need to set falsy fields.
                    constructorCode.pr(port, String.join("\n",
                        portOnSelf+"_trigger.last = NULL;",
                        portOnSelf+"_trigger.number_of_reactions = "+triggered.size()+";"
                    ));


                    // Set the physical_time_of_arrival
                    constructorCode.pr(
                        port,
                        CExtensionUtils.surroundWithIfFederated(
                            portOnSelf+"_trigger.physical_time_of_arrival = NEVER;"
                        )
                    );

                    if (containedReactor.getWidthSpec() != null) {
                        constructorCode.unindent();
                        constructorCode.pr("}");
                    }
                }
            }
            body.unindent();
            body.pr(String.join("\n",
                "} _lf_"+containedReactor.getName()+array+";",
                "int _lf_"+containedReactor.getName()+"_width;"
            ));
        }
    }

    /**
     * This function is provided to allow extensions of the CGenerator to append the structure of the self struct
     * @param body The body of the self struct
     * @param decl The reactor declaration for the self struct
     * @param constructorCode Code that is executed when the reactor is instantiated
     */
    protected void generateSelfStructExtension(
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
     *  @param r The reactor.
     */
    public void generateReactions(CodeBuilder src, Reactor r) {
        var reactionIndex = 0;
        var reactor = ASTUtils.toDefinition(r);
        for (Reaction reaction : allReactions(reactor)) {
            generateReaction(src, reaction, r, reactionIndex);
            // Increment reaction index even if the reaction is not in the federate
            // so that across federates, the reaction indices are consistent.
            reactionIndex++;
        }
    }

    /** Generate a reaction function definition for a reactor.
     *  This function will have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reaction The reaction.
     *  @param r The reactor.
     *  @param reactionIndex The position of the reaction within the reactor.
     */
    protected void generateReaction(CodeBuilder src, Reaction reaction, Reactor r, int reactionIndex) {
        src.pr(CReactionGenerator.generateReaction(
            reaction,
            r,
            reactionIndex,
            mainDef,
            errorReporter,
            types,
            targetConfig,
            getTarget().requiresTypes
        ));
    }

    /**
     * Record startup, shutdown, and reset reactions.
     * @param instance A reactor instance.
     */
    private void recordBuiltinTriggers(ReactorInstance instance) {
        // For each reaction instance, allocate the arrays that will be used to
        // trigger downstream reactions.
        for (ReactionInstance reaction : instance.reactions) {
            var reactor = reaction.getParent();
            var temp = new CodeBuilder();
            var foundOne = false;

            var reactionRef = CUtil.reactionRef(reaction);

            // Next handle triggers of the reaction that come from a multiport output
            // of a contained reactor.  Also, handle startup and shutdown triggers.
            for (TriggerInstance<?> trigger : reaction.triggers) {
                if (trigger.isStartup()) {
                    temp.pr("_lf_startup_reactions[_lf_startup_reactions_count++] = &"+reactionRef+";");
                    startupReactionCount += reactor.getTotalWidth();
                    foundOne = true;
                } else if (trigger.isShutdown()) {
                    temp.pr("_lf_shutdown_reactions[_lf_shutdown_reactions_count++] = &"+reactionRef+";");
                    foundOne = true;
                    shutdownReactionCount += reactor.getTotalWidth();

                    if (targetConfig.tracing != null) {
                        var description = CUtil.getShortenedName(reactor);
                        var reactorRef = CUtil.reactorRef(reactor);
                        temp.pr(String.join("\n",
                            "_lf_register_trace_event("+reactorRef+", &("+reactorRef+"->_lf__shutdown),",
                            "trace_trigger, "+addDoubleQuotes(description+".shutdown")+");"
                        ));
                    }
                } else if (trigger.isReset()) {
                    temp.pr("_lf_reset_reactions[_lf_reset_reactions_count++] = &"+reactionRef+";");
                    resetReactionCount += reactor.getTotalWidth();
                    foundOne = true;
                }
            }
            if (foundOne) initializeTriggerObjects.pr(temp.toString());
        }
    }



    /**
     * Generate code to set up the tables used in _lf_start_time_step to decrement reference
     * counts and mark outputs absent between time steps. This function puts the code
     * into startTimeStep.
     */
    private void generateStartTimeStep(ReactorInstance instance) {
        // Avoid generating dead code if nothing is relevant.
        var foundOne = false;
        var temp = new CodeBuilder();
        var containerSelfStructName = CUtil.reactorRef(instance);

        // Handle inputs that get sent data from a reaction rather than from
        // another contained reactor and reactions that are triggered by an
        // output of a contained reactor.
        // Note that there may be more than one reaction reacting to the same
        // port so we have to avoid listing the port more than once.
        var portsSeen = new LinkedHashSet<PortInstance>();
        for (ReactionInstance reaction : instance.reactions) {
            for (PortInstance port : Iterables.filter(reaction.effects, PortInstance.class)) {
                if (port.getDefinition() instanceof Input && !portsSeen.contains(port)) {
                    portsSeen.add(port);
                    // This reaction is sending to an input. Must be
                    // the input of a contained reactor in the federate.
                    // NOTE: If instance == main and the federate is within a bank,
                    // this assumes that the reaction writes only to the bank member in the federate.
                    foundOne = true;

                    temp.pr("// Add port "+port.getFullName()+" to array of is_present fields.");

                    if (!Objects.equal(port.getParent(), instance)) {
                        // The port belongs to contained reactor, so we also have
                        // iterate over the instance bank members.
                        temp.startScopedBlock();
                        temp.pr("int count = 0; SUPPRESS_UNUSED_WARNING(count);");
                        temp.startScopedBlock(instance);
                        temp.startScopedBankChannelIteration(port, null);
                    } else {
                        temp.startScopedBankChannelIteration(port, "count");
                    }
                    var portRef = CUtil.portRefNested(port);
                    var con = (port.isMultiport()) ? "->" : ".";

                    temp.pr("_lf_is_present_fields["+startTimeStepIsPresentCount+" + count] = &"+portRef+con+"is_present;");
                    // Intended_tag is only applicable to ports in federated execution.
                    temp.pr(
                        CExtensionUtils.surroundWithIfFederatedDecentralized(
                        "_lf_intended_tag_fields["+startTimeStepIsPresentCount+" + count] = &"+portRef+con+"intended_tag;"
                        )
                    );

                    startTimeStepIsPresentCount += port.getWidth() * port.getParent().getTotalWidth();

                    if (!Objects.equal(port.getParent(), instance)) {
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
        if (foundOne) startTimeStep.pr(temp.toString());
        temp = new CodeBuilder();
        foundOne = false;

        for (ActionInstance action : instance.actions) {
            foundOne = true;
            temp.startScopedBlock(instance);

            temp.pr(String.join("\n",
                "// Add action "+action.getFullName()+" to array of is_present fields.",
                "_lf_is_present_fields["+startTimeStepIsPresentCount+"] ",
                "        = &"+containerSelfStructName+"->_lf_"+action.getName()+".is_present;"
            ));

            // Intended_tag is only applicable to actions in federated execution with decentralized coordination.
            temp.pr(
                CExtensionUtils.surroundWithIfFederatedDecentralized(
                    String.join("\n",
                                "// Add action " + action.getFullName()
                                    + " to array of intended_tag fields.",
                                "_lf_intended_tag_fields["
                                    + startTimeStepIsPresentCount + "] ",
                                "        = &" + containerSelfStructName
                                    + "->_lf_" + action.getName()
                                    + ".intended_tag;"
                    )));

            startTimeStepIsPresentCount += action.getParent().getTotalWidth();
            temp.endScopedBlock();
        }
        if (foundOne) startTimeStep.pr(temp.toString());
        temp = new CodeBuilder();
        foundOne = false;

        // Next, set up the table to mark each output of each contained reactor absent.
        for (ReactorInstance child : instance.children) {
            if (child.outputs.size() > 0) {

                temp.startScopedBlock();
                temp.pr("int count = 0; SUPPRESS_UNUSED_WARNING(count);");
                temp.startScopedBlock(child);

                var channelCount = 0;
                for (PortInstance output : child.outputs) {
                    if (!output.getDependsOnReactions().isEmpty()){
                        foundOne = true;
                        temp.pr("// Add port "+output.getFullName()+" to array of is_present fields.");
                        temp.startChannelIteration(output);
                        temp.pr("_lf_is_present_fields["+startTimeStepIsPresentCount+" + count] = &"+CUtil.portRef(output)+".is_present;");

                        // Intended_tag is only applicable to ports in federated execution with decentralized coordination.
                        temp.pr(
                            CExtensionUtils.surroundWithIfFederatedDecentralized(
                                String.join("\n",
                                            "// Add port "+output.getFullName()+" to array of intended_tag fields.",
                                                "_lf_intended_tag_fields["+startTimeStepIsPresentCount+" + count] = &"+CUtil.portRef(output)+".intended_tag;"
                                )));

                        temp.pr("count++;");
                        channelCount += output.getWidth();
                        temp.endChannelIteration(output);
                    }
                }
                startTimeStepIsPresentCount += channelCount * child.getTotalWidth();
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
    private void generateTimerInitializations(ReactorInstance instance) {
        for (TimerInstance timer : instance.timers) {
            if (!timer.isStartup()) {
                initializeTriggerObjects.pr(CTimerGenerator.generateInitializer(timer));
                timerCount += timer.getParent().getTotalWidth();
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
     public void processProtoFile(String filename) {
        var protoc = commandFactory.createCommand(
            "protoc-c",
            List.of("--c_out="+this.fileConfig.getSrcGenPath(), filename),
            fileConfig.srcPath);
        if (protoc == null) {
            errorReporter.reportError("Processing .proto files requires protoc-c >= 1.3.3.");
            return;
        }
        var returnCode = protoc.run();
        if (returnCode == 0) {
            var nameSansProto = filename.substring(0, filename.length() - 6);
            targetConfig.compileAdditionalSources.add(
                fileConfig.getSrcGenPath().resolve(nameSansProto + ".pb-c.c").toString()
            );

            targetConfig.compileLibraries.add("-l");
            targetConfig.compileLibraries.add("protobuf-c");
            targetConfig.compilerFlags.add("-lprotobuf-c");
        } else {
            errorReporter.reportError("protoc-c returns error code " + returnCode);
        }
    }

    /**
     * Construct a unique type for the struct of the specified
     * typed variable (port or action) of the specified reactor class.
     * This is required to be the same as the type name returned by
     * {@link #variableStructType(TriggerInstance)}.
     * @param variable The variable.
     * @param reactor The reactor class.
     * @return The name of the self struct.
     */
    public static String variableStructType(Variable variable, Reactor reactor) {
        return CUtil.getName(reactor)+"_"+variable.getName()+"_t";
    }

    /**
     * Construct a unique type for the struct of the specified
     * instance (port or action).
     * This is required to be the same as the type name returned by
     * {@link #variableStructType(Variable, Reactor)}.
     * @param portOrAction The port or action instance.
     * @return The name of the self struct.
     */
    public static String variableStructType(TriggerInstance<?> portOrAction) {
        return CUtil.getName(portOrAction.getParent().reactorDefinition)+"_"+portOrAction.getName()+"_t";
    }

    /**
     * If tracing is turned on, then generate code that records
     * the full name of the specified reactor instance in the
     * trace table. If tracing is not turned on, do nothing.
     * @param instance The reactor instance.
     */
    private void generateTraceTableEntries(ReactorInstance instance) {
        if (targetConfig.tracing != null) {
            initializeTriggerObjects.pr(
                CTracingGenerator.generateTraceTableEntries(instance)
            );
        }
    }

    /**
     * Generate code to instantiate the specified reactor instance and
     * initialize it.
     * @param instance A reactor instance.
     */
    public void generateReactorInstance(ReactorInstance instance) {
        var reactorClass = ASTUtils.toDefinition(instance.getDefinition().getReactorClass());
        var fullName = instance.getFullName();
        initializeTriggerObjects.pr(
                "// ***** Start initializing " + fullName + " of class " + reactorClass.getName());
        // Generate the instance self struct containing parameters, state variables,
        // and outputs (the "self" struct).
        initializeTriggerObjects.pr(CUtil.reactorRefName(instance)+"["+CUtil.runtimeIndex(instance)+"] = new_"+CUtil.getName(reactorClass)+"();");
        // Generate code to initialize the "self" struct in the
        // _lf_initialize_trigger_objects function.
        generateTraceTableEntries(instance);
        generateReactorInstanceExtension(instance);
        generateParameterInitialization(instance);
        initializeOutputMultiports(instance);
        initializeInputMultiports(instance);
        recordBuiltinTriggers(instance);

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
        for (ReactorInstance child : instance.children) {
            // If this reactor is a placeholder for a bank of reactors, then generate
            // an array of instances of reactors and create an enclosing for loop.
            // Need to do this for each of the builders into which the code writes.
            startTimeStep.startScopedBlock(child);
            initializeTriggerObjects.startScopedBlock(child);
            generateReactorInstance(child);
            initializeTriggerObjects.endScopedBlock();
            startTimeStep.endScopedBlock();
        }

        // For this instance, define what must be done at the start of
        // each time step. This sets up the tables that are used by the
        // _lf_start_time_step() function in reactor_common.c.
        // Note that this function is also run once at the end
        // so that it can deallocate any memory.
        generateStartTimeStep(instance);
        initializeTriggerObjects.pr("//***** End initializing " + fullName);
    }

    /**
     * For each action of the specified reactor instance, generate initialization code
     * for the offset and period fields.
     * @param instance The reactor.
     */
    private void generateActionInitializations(ReactorInstance instance) {
        initializeTriggerObjects.pr(CActionGenerator.generateInitializers(instance));
    }

    /**
     * Initialize actions by creating a lf_token_t in the self struct.
     * This has the information required to allocate memory for the action payload.
     * Skip any action that is not actually used as a trigger.
     * @param reactor The reactor containing the actions.
     */
    private void generateInitializeActionToken(ReactorInstance reactor) {
        for (ActionInstance action : reactor.actions) {
            // Skip this step if the action is not in use.
            if (action.getParent().getTriggers().contains(action)
            ) {
                var type = getInferredType(action.getDefinition());
                var payloadSize = "0";
                if (!type.isUndefined()) {
                    var typeStr = types.getTargetType(type);
                    if (CUtil.isTokenType(type, types)) {
                        typeStr = CUtil.rootType(typeStr);
                    }
                    if (typeStr != null && !typeStr.equals("") && !typeStr.equals("void")) {
                        payloadSize = "sizeof("+typeStr+")";
                    }
                }

                var selfStruct = CUtil.reactorRef(action.getParent());
                initializeTriggerObjects.pr(
                    CActionGenerator.generateTokenInitializer(
                        selfStruct, action.getName(), payloadSize
                    )
                );
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
     */
    protected void generateReactorInstanceExtension(ReactorInstance instance) {
        // Do nothing
    }

    /**
     * Generate code that initializes the state variables for a given instance.
     * Unlike parameters, state variables are uniformly initialized for all instances
     * of the same reactor.
     * @param instance The reactor class instance
     */
    protected void generateStateVariableInitializations(ReactorInstance instance) {
        var reactorClass = instance.getDefinition().getReactorClass();
        var selfRef = CUtil.reactorRef(instance);
        for (StateVar stateVar : allStateVars(toDefinition(reactorClass))) {
            if (isInitialized(stateVar)) {
                var mode = stateVar.eContainer() instanceof Mode ?
                    instance.lookupModeInstance((Mode) stateVar.eContainer()) :
                    instance.getMode(false);
                initializeTriggerObjects.pr(CStateGenerator.generateInitializer(
                    instance,
                    selfRef,
                    stateVar,
                    mode,
                    types
                ));
                if (mode != null && stateVar.isReset()) {
                    modalStateResetCount += instance.getTotalWidth();
                }
            }
        }
    }

    /**
     * Generate code to set the deadline field of the reactions in the
     * specified reactor instance.
     * @param instance The reactor instance.
     */
    private void generateSetDeadline(ReactorInstance instance) {
        for (ReactionInstance reaction : instance.reactions) {
            var selfRef = CUtil.reactorRef(reaction.getParent())+"->_lf__reaction_"+reaction.index;
            if (reaction.declaredDeadline != null) {
                var deadline = reaction.declaredDeadline.maxDelay;
                initializeTriggerObjects.pr(selfRef+".deadline = "+GeneratorBase.timeInTargetLanguage(deadline)+";");
            } else { // No deadline.
                initializeTriggerObjects.pr(selfRef+".deadline = NEVER;");
            }
        }
    }

    /**
     * Generate code to initialize modes.
     * @param instance The reactor instance.
     */
    private void generateModeStructure(ReactorInstance instance) {
        CModesGenerator.generateModeStructure(instance, initializeTriggerObjects);
        if (!instance.modes.isEmpty()) {
            modalReactorCount += instance.getTotalWidth();
        }
    }

    /**
     * Generate runtime initialization code for parameters of a given reactor instance
     * @param instance The reactor instance.
     */
    protected void generateParameterInitialization(ReactorInstance instance) {
        var selfRef = CUtil.reactorRef(instance);
        // Set the local bank_index variable so that initializers can use it.
        initializeTriggerObjects.pr("bank_index = "+CUtil.bankIndex(instance)+";"
                + " SUPPRESS_UNUSED_WARNING(bank_index);");
        for (ParameterInstance parameter : instance.parameters) {
            // NOTE: we now use the resolved literal value. For better efficiency, we could
            // store constants in a global array and refer to its elements to avoid duplicate
            // memory allocations.
            // NOTE: If the parameter is initialized with a static initializer for an array
            // or struct (the initialization expression is surrounded by { ... }), then we
            // have to declare a static variable to ensure that the memory is put in data space
            // and not on the stack.
            // FIXME: Is there a better way to determine this than the string comparison?
            var initializer = CParameterGenerator.getInitializer(parameter);
            if (initializer.startsWith("{")) {
                var temporaryVariableName = parameter.uniqueID();
                initializeTriggerObjects.pr(String.join("\n",
                    "static "+types.getVariableDeclaration(parameter.type, temporaryVariableName, true)+" = "+initializer+";",
                    selfRef+"->"+parameter.getName()+" = "+temporaryVariableName+";"
                ));
            } else {
                initializeTriggerObjects.pr(selfRef+"->"+parameter.getName()+" = "+initializer+";");
            }
        }
    }

    /**
     * Generate code that mallocs memory for any output multiports.
     * @param reactor The reactor instance.
     */
    private void initializeOutputMultiports(ReactorInstance reactor) {
        var reactorSelfStruct = CUtil.reactorRef(reactor);
        for (PortInstance output : reactor.outputs) {
            initializeTriggerObjects.pr(CPortGenerator.initializeOutputMultiport(
                output,
                reactorSelfStruct
            ));
        }
    }

    /**
     * Allocate memory for inputs.
     * @param reactor The reactor.
     */
    private void initializeInputMultiports(ReactorInstance reactor) {
        var reactorSelfStruct = CUtil.reactorRef(reactor);
        for (PortInstance input : reactor.inputs) {
            initializeTriggerObjects.pr(CPortGenerator.initializeInputMultiport(
                input,
                reactorSelfStruct
            ));
        }
    }

    @Override
    public TargetTypes getTargetTypes() {
        return types;
    }

    /**
     *
     * @param context
     * @return
     */
    protected DockerGenerator getDockerGenerator(LFGeneratorContext context) {
        return new CDockerGenerator(context);
    }

    // //////////////////////////////////////////
    // // Protected methods.

    // Perform set up that does not generate code
    protected void setUpGeneralParameters() {
        accommodatePhysicalActionsIfPresent();
        targetConfig.compileDefinitions.put("LOG_LEVEL", targetConfig.logLevel.ordinal() + "");
        targetConfig.compileAdditionalSources.addAll(CCoreFilesUtils.getCTargetSrc());
        // Create the main reactor instance if there is a main reactor.
        createMainReactorInstance();
        if (hasModalReactors) {
            // So that each separate compile knows about modal reactors, do this:
            targetConfig.compileDefinitions.put("MODAL_REACTORS", "TRUE");
        }
        if (targetConfig.threading && targetConfig.platformOptions.platform == Platform.ARDUINO
            && (targetConfig.platformOptions.board == null || !targetConfig.platformOptions.board.contains("mbed"))) {
            //non-MBED boards should not use threading
            System.out.println("Threading is incompatible on your current Arduino flavor. Setting threading to false.");
            targetConfig.threading = false;
        }

        if (targetConfig.platformOptions.platform == Platform.ARDUINO && !targetConfig.noCompile
            && targetConfig.platformOptions.board == null) {
            System.out.println("To enable compilation for the Arduino platform, you must specify the fully-qualified board name (FQBN) in the target property. For example, platform: {name: arduino, board: arduino:avr:leonardo}. Entering \"no-compile\" mode and generating target code only.");
            targetConfig.noCompile = true;
        }
        if (targetConfig.threading) {  // FIXME: This logic is duplicated in CMake
            pickScheduler();
            // FIXME: this and pickScheduler should be combined.
            targetConfig.compileDefinitions.put(
                "SCHEDULER",
                targetConfig.schedulerType.name()
            );
            targetConfig.compileDefinitions.put(
                "NUMBER_OF_WORKERS",
                String.valueOf(targetConfig.workers)
            );
        }
        pickCompilePlatform();
    }


//    // Perform set up that does not generate code
//    protected void setUpFederateSpecificParameters(FederateInstance federate, CodeBuilder commonCode) {
//        currentFederate = federate;
//        if (isFederated) {
//            // Reset the cmake-includes and files, to be repopulated for each federate individually.
//            // This is done to enable support for separately
//            // adding cmake-includes/files for different federates to prevent linking and mixing
//            // all federates' supporting libraries/files together.
//            targetConfig.cmakeIncludes.clear();
//            targetConfig.cmakeIncludesWithoutPath.clear();
//            targetConfig.fileNames.clear();
//            targetConfig.filesNamesWithoutPath.clear();
//
//            // Re-apply the cmake-include target property of the main .lf file.
//            var target = GeneratorUtils.findTarget(mainDef.getReactorClass().eResource());
//            if (target.getConfig() != null) {
//                // Update the cmake-include
//                TargetProperty.updateOne(
//                    this.targetConfig,
//                    TargetProperty.CMAKE_INCLUDE,
//                    convertToEmptyListIfNull(target.getConfig().getPairs()),
//                    errorReporter
//                );
//                // Update the files
//                TargetProperty.updateOne(
//                    this.targetConfig,
//                    TargetProperty.FILES,
//                    convertToEmptyListIfNull(target.getConfig().getPairs()),
//                    errorReporter
//                );
//            }
//            // Clear out previously generated code.
//            code = new CodeBuilder(commonCode);
//            initializeTriggerObjects = new CodeBuilder();
//            // Enable clock synchronization if the federate
//            // is not local and clock-sync is enabled
//            initializeClockSynchronization();
//            startTimeStep = new CodeBuilder();
//        }
//    }

    protected void handleProtoFiles() {
        // Handle .proto files.
        for (String file : targetConfig.protoFiles) {
            this.processProtoFile(file);
            var dotIndex = file.lastIndexOf(".");
            var rootFilename = file;
            if (dotIndex > 0) {
                rootFilename = file.substring(0, dotIndex);
            }
            code.pr("#include " + addDoubleQuotes(rootFilename + ".pb-c.h"));
        }
    }

    /**
     * Generate code that needs to appear at the top of the generated
     * C file, such as #define and #include statements.
     */
    public String generateDirectives() {
        CodeBuilder code = new CodeBuilder();
        code.prComment("Code generated by the Lingua Franca compiler from:");
        code.prComment("file:/" + FileUtil.toUnixString(fileConfig.srcFile));
        code.pr(CPreambleGenerator.generateDefineDirectives(
            targetConfig,
            fileConfig.getSrcGenPath(),
            hasModalReactors
        ));
        code.pr(CPreambleGenerator.generateIncludeStatements(
            targetConfig,
            CCppMode
        ));
        return code.toString();
    }

    /**
     * Generate top-level preamble code.
     */
    protected String generateTopLevelPreambles() {
        CodeBuilder code = new CodeBuilder();

        // preamble for federated execution setup
        if (targetConfig.fedSetupPreamble != null) {
            if (targetLanguageIsCpp()) code.pr("extern \"C\" {");
            code.pr("#include \"" + targetConfig.fedSetupPreamble + "\"");
            if (targetLanguageIsCpp()) code.pr("}");
        }

        // user preambles
        if (this.mainDef != null) {
            var mainModel = (Model) toDefinition(mainDef.getReactorClass()).eContainer();
            for (Preamble p : mainModel.getPreambles()) {
                code.pr(toText(p.getCode()));
            }
        }
        return code.toString();
    }

    protected boolean targetLanguageIsCpp() {
        return CCppMode;
    }

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
    @Override
    public GeneratorBase.ErrorFileAndLine parseCommandOutput(String line) {
        var matcher = compileErrorPattern.matcher(line);
        if (matcher.find()) {
            var result = new ErrorFileAndLine();
            result.filepath = matcher.group("path");
            result.line = matcher.group("line");
            result.character = matcher.group("column");
            result.message = matcher.group("message");

            if (!result.message.toLowerCase().contains("error:")) {
                result.isError = false;
            }
            return result;
        }
        return null;
    }

    ////////////////////////////////////////////
    //// Private methods.
    /** Returns the Target enum for this generator */
    @Override
    public Target getTarget() {
        return Target.C;
    }

    ////////////////////////////////////////////////////////////
    //// Private methods

    /**
     * If a main or federated reactor has been declared, create a ReactorInstance
     * for this top level. This will also assign levels to reactions, then,
     * if the program is federated, perform an AST transformation to disconnect
     * connections between federates.
     */
    private void createMainReactorInstance() {
        if (this.mainDef != null) {
            if (this.main == null) {
                // Recursively build instances.
                this.main = new ReactorInstance(toDefinition(mainDef.getReactorClass()), errorReporter);
                var reactionInstanceGraph = this.main.assignLevels();
                if (reactionInstanceGraph.nodeCount() > 0) {
                    errorReporter.reportError("Main reactor has causality cycles. Skipping code generation.");
                    return;
                }
                if (hasDeadlines) {
                    this.main.assignDeadlines();
                }
                // Inform the run-time of the breadth/parallelism of the reaction graph
                var breadth = reactionInstanceGraph.getBreadth();
                if (breadth == 0) {
                    errorReporter.reportWarning("The program has no reactions");
                } else {
                    targetConfig.compileDefinitions.put(
                      "LF_REACTION_GRAPH_BREADTH",
                      String.valueOf(reactionInstanceGraph.getBreadth())
                    );
                }
            }
        }

    }

    /**
     * Generate an array of self structs for the reactor
     * and one for each of its children.
     * @param r The reactor instance.
     */
    private void generateSelfStructs(ReactorInstance r) {
        initializeTriggerObjects.pr(CUtil.selfType(r)+"* "+CUtil.reactorRefName(r)+"["+r.getTotalWidth()+"];");
        initializeTriggerObjects.pr("SUPPRESS_UNUSED_WARNING("+CUtil.reactorRefName(r)+");");
        for (ReactorInstance child : r.children) {
            generateSelfStructs(child);
        }
    }
}
