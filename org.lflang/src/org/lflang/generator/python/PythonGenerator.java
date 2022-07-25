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

package org.lflang.generator.python;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.xbase.lib.Exceptions;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.OldFedFileConfig;
import org.lflang.federated.launcher.FedPyLauncher;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.CodeMap;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.SubContext;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.c.CDockerGenerator;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Input;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

import com.google.common.base.Objects;

import org.lflang.util.StringUtil;


/**
 * Generator for Python target. This class generates Python code defining each
 * reactor
 * class given in the input .lf file and imported .lf files.
 *
 * Each class will contain all the reaction functions defined by the user in
 * order, with the necessary ports/actions given as parameters.
 * Moreover, each class will contain all state variables in native Python
 * format.
 *
 * A backend is also generated using the CGenerator that interacts with the C
 * code library (see CGenerator.xtend).
 * The backend is responsible for passing arguments to the Python reactor
 * functions.
 *
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
public class PythonGenerator extends CGenerator {

    // Used to add statements that come before reactor classes and user code
    private CodeBuilder pythonPreamble = new CodeBuilder();

    // Used to add module requirements to setup.py (delimited with ,)
    private List<String> pythonRequiredModules = new ArrayList<>();

    private PythonTypes types;

    public PythonGenerator(FileConfig fileConfig, ErrorReporter errorReporter) {
        this(fileConfig, errorReporter, new PythonTypes(errorReporter));
    }

    private PythonGenerator(FileConfig fileConfig, ErrorReporter errorReporter, PythonTypes types) {
        super(fileConfig, errorReporter, false, types);
        this.targetConfig.compiler = "gcc";
        this.targetConfig.compilerFlags = new ArrayList<>();
        this.targetConfig.linkerFlags = "";
        this.types = types;
    }

    /**
     * Generic struct for ports with primitive types and
     * statically allocated arrays in Lingua Franca.
     * This template is defined as
     * typedef struct {
     * PyObject* value;
     * bool is_present;
     * int num_destinations;
     * FEDERATED_CAPSULE_EXTENSION
     * } generic_port_instance_struct;
     *
     * @see `reactor-c-py/lib/pythontarget.h`
     */
    String genericPortType = "generic_port_instance_struct";

    /**
     * Generic struct for actions.
     * This template is defined as
     * typedef struct {
     * trigger_t* trigger;
     * PyObject* value;
     * bool is_present;
     * bool has_value;
     * lf_token_t* token;
     * FEDERATED_CAPSULE_EXTENSION
     * } generic_action_instance_struct;
     *
     * @see `reactor-c-py/lib/pythontarget.h`
     */
    String genericActionType = "generic_action_instance_struct";

    /** Returns the Target enum for this generator */
    @Override
    public Target getTarget() {
        return Target.Python;
    }

    private Set<String> protoNames = new HashSet<>();

    // //////////////////////////////////////////
    // // Public methods
    @Override
    public TargetTypes getTargetTypes() {
        return types;
    }

    // //////////////////////////////////////////
    // // Protected methods

    /**
     * Generate all Python classes if they have a reaction
     *
     */
    public String generatePythonReactorClasses() {
        CodeBuilder pythonClasses = new CodeBuilder();
        CodeBuilder pythonClassesInstantiation = new CodeBuilder();

        // Generate reactor classes in Python
        pythonClasses.pr(PythonReactorGenerator.generatePythonClass(main, main, types));

        // Create empty lists to hold reactor instances
        pythonClassesInstantiation.pr(PythonReactorGenerator.generateListsToHoldClassInstances(main));

        // Instantiate generated classes
        pythonClassesInstantiation.pr(PythonReactorGenerator.generatePythonClassInstantiations(main, main));

        return String.join("\n",
                           pythonClasses.toString(),
                           "",
                           "# Instantiate classes",
                           pythonClassesInstantiation.toString()
        );
    }

    /**
     * Generate the Python code constructed from reactor classes and
     * user-written classes.
     *
     * @return the code body
     */
    public String generatePythonCode(String pyModuleName) {
        return String.join("\n",
                           "import os",
                           "import sys",
                           "sys.path.append(os.path.dirname(__file__))",
                           "# List imported names, but do not use pylint's --extension-pkg-allow-list option",
                           "# so that these names will be assumed present without having to compile and install.",
                           "from " + pyModuleName
                               + " import (  # pylint: disable=no-name-in-module, import-error",
                           "    Tag, action_capsule_t, compare_tags, get_current_tag, get_elapsed_logical_time,",
                           "    get_elapsed_physical_time, get_logical_time, get_microstep, get_physical_time,",
                           "    get_start_time, port_capsule, request_stop, schedule_copy,",
                           "    start",
                           ")",
                           "# pylint: disable=c-extension-no-member",
                           "import " + pyModuleName + " as lf",
                           "try:",
                           "    from LinguaFrancaBase.constants import BILLION, FOREVER, NEVER, instant_t, interval_t",
                           "    from LinguaFrancaBase.functions import (",
                           "        DAY, DAYS, HOUR, HOURS, MINUTE, MINUTES, MSEC, MSECS, NSEC, NSECS, SEC, SECS, USEC,",
                           "        USECS, WEEK, WEEKS",
                           "    )",
                           "    from LinguaFrancaBase.classes import Make",
                           "except ModuleNotFoundError:",
                           "    print(\"No module named \'LinguaFrancaBase\'. \"",
                           "          \"Install using \\\"pip3 install LinguaFrancaBase\\\".\")",
                           "    sys.exit(1)",
                           "import copy",
                           "",
                           pythonPreamble.toString(),
                           "",
                           generatePythonReactorClasses(),
                           "",
                           PythonMainGenerator.generateCode()
        );
    }

    /**
     * Generate the setup.py required to compile and install the module.
     * Currently, the package name is based on filename which does not support
     * sharing the setup.py for multiple .lf files.
     * TODO: use an alternative package name (possibly based on folder name)
     *
     * If the LF program itself is threaded or if tracing is enabled, NUMBER_OF_WORKERS is added as a macro
     * so that platform-specific C files will contain the appropriate functions.
     */
    public String generatePythonSetupFile(String lfModuleName, String pyModuleName) {
        List<String> sources = new ArrayList<>(targetConfig.compileAdditionalSources);
        sources.add(lfModuleName + ".c");
        sources = sources.stream()
                         .map(Paths::get)
                         .map(FileUtil::toUnixString)
                         .map(StringUtil::addDoubleQuotes)
                         .collect(Collectors.toList());

        List<String> macros = new ArrayList<>();
        macros.add(generateMacroEntry("MODULE_NAME", pyModuleName));

        for (var entry : targetConfig.compileDefinitions.entrySet()) {
            macros.add(generateMacroEntry(entry.getKey(), entry.getValue()));
        }

        if (targetConfig.threading || targetConfig.tracing != null) {
            macros.add(generateMacroEntry("NUMBER_OF_WORKERS", String.valueOf(targetConfig.workers)));
        }

        List<String> installRequires = new ArrayList<>(pythonRequiredModules);
        installRequires.add("LinguaFrancaBase");
        installRequires.replaceAll(StringUtil::addDoubleQuotes);

        return String.join("\n",
                           """
                               import sys
                               assert (sys.version_info.major >= 3 and sys.version_info.minor >= 6), \
                                   "The Python target requires Python version >= 3.6."
                                           
                               from setuptools import setup, Extension
                               """,
                           "linguafranca" + lfModuleName + "module = Extension("
                               + StringUtil.addDoubleQuotes(pyModuleName) + ",",
                           "                                            sources = ["
                               + String.join(", ", sources) + "],",
                           "                                            define_macros=["
                               + String.join(", ", macros) + "])",
                           "",
                           "setup(name="
                               + StringUtil.addDoubleQuotes(pyModuleName)
                               + ", version=\"1.0\",",
                           "        ext_modules = [linguafranca" + lfModuleName
                               + "module],",
                           "        install_requires=["
                               + String.join(", ", installRequires) + "])"
        );
    }

    /**
     * Generate the necessary Python files.
     */
    public Map<Path, CodeMap> generatePythonFiles(
        String lfModuleName,
        String pyModuleName,
        String pyFileName
    ) throws IOException {
        Path filePath = fileConfig.getSrcGenPath().resolve(pyFileName);
        File file = filePath.toFile();
        Files.deleteIfExists(filePath);
        // Create the necessary directories
        if (!file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }
        Map<Path, CodeMap> codeMaps = new HashMap<>();
        codeMaps.put(filePath, CodeMap.fromGeneratedCode(
            generatePythonCode(pyModuleName).toString()));
        FileUtil.writeToFile(codeMaps.get(filePath).getGeneratedCode(), filePath);

        Path setupPath = fileConfig.getSrcGenPath().resolve("setup.py");
        // Handle Python setup
        System.out.println("Generating setup file to " + setupPath);
        Files.deleteIfExists(setupPath);

        // Create the setup file
        FileUtil.writeToFile(generatePythonSetupFile(lfModuleName, pyModuleName), setupPath);
        return codeMaps;
    }

    /**
     * Execute the command that compiles and installs the current Python module
     */
    public void pythonCompileCode(LFGeneratorContext context) {
        // Look for python3
        var pythonCommand = "python3";
        if (LFCommand.get("python3", List.of("--version"), true, fileConfig.getSrcGenPath())
            == null) {
            // Look for python instead
            if (LFCommand.get("python", List.of("--version"), true, fileConfig.getSrcGenPath())
                != null) {
                pythonCommand = "python";
            } else {
                errorReporter.reportError(
                    """
                        Could not find "python3" or "python".
                        The Python target requires Python >= 3.6 and setuptools >= 45.2.0-1 to build the generated extension.
                        See https://www.lf-lang.org/docs/handbook/target-language-details.
                        Auto-compiling can be disabled using the "no-compile: true" target property.
                        """
                );
                return;
            }
        }

        // if we found the compile command, we will also find the install command
        LFCommand buildCmd = commandFactory.createCommand(
            pythonCommand, List.of("setup.py", "--quiet", "build_ext", "--inplace"), fileConfig.getSrcGenPath()
        );
        buildCmd.setQuiet();

        // Set compile time environment variables
        buildCmd.setEnvironmentVariable("CC", targetConfig.compiler); // Use gcc as the compiler
        buildCmd.setEnvironmentVariable("LDFLAGS", targetConfig.linkerFlags); // The linker complains about including pythontarget.h twice (once in the generated code and once in pythontarget.c)
        // To avoid this, we force the linker to allow multiple definitions. Duplicate names would still be caught by the
        // compiler.
        if (buildCmd.run(context.getCancelIndicator()) == 0) {
            System.out.println("Successfully built Python extension.");
        } else {
            errorReporter.reportError(
                "Failed to build Python extension due to the following error(s):\n"
                    +
                    buildCmd.getErrors());
        }
    }

    /**
     * Generate code that needs to appear at the top of the generated
     * C file, such as #define and #include statements.
     */
    @Override
    public String generateDirectives() {
        CodeBuilder code = new CodeBuilder();
        code.prComment("Code generated by the Lingua Franca compiler from:");
        code.prComment("file:/" + FileUtil.toUnixString(fileConfig.srcFile));
        code.pr(PythonPreambleGenerator.generateCDefineDirectives(
            targetConfig, fileConfig.getSrcGenPath(), hasModalReactors));
        code.pr(PythonPreambleGenerator.generateCIncludeStatements(
            targetConfig, hasModalReactors));
        return code.toString();
    }

    /**
     * Override generate top-level preambles, but put the user preambles in the
     * .py file rather than the C file. Also handles including the federated
     * execution setup preamble specified in the target config.
     */
    @Override
    protected String generateTopLevelPreambles() {
        // user preambles
        Set<Model> models = new LinkedHashSet<>();
        for (Reactor r : ASTUtils.convertToEmptyListIfNull(reactors)) {
            // The following assumes all reactors have a container.
            // This means that generated reactors **have** to be
            // added to a resource; not doing so will result in a NPE.
            models.add((Model) ASTUtils.toDefinition(r).eContainer());
        }
        // Add the main reactor if it is defined
        if (this.mainDef != null) {
            models.add((Model) ASTUtils.toDefinition(this.mainDef.getReactorClass()).eContainer());
        }
        for (Model m : models) {
            pythonPreamble.pr(PythonPreambleGenerator.generatePythonPreambles(m.getPreambles()));
        }

        // C preamble for federated execution setup
        String ret = "";
        if (targetConfig.fedSetupPreamble != null) {
            ret = "#include \"" + targetConfig.fedSetupPreamble + "\"";
        }
        return ret;
    }

    /**
     * Process a given .proto file.
     *
     * Run, if possible, the proto-c protocol buffer code generator to produce
     * the required .h and .c files.
     *
     * @param filename Name of the file to process.
     */
    @Override
    public void processProtoFile(String filename, CancelIndicator cancelIndicator) {
        LFCommand protoc = commandFactory.createCommand(
            "protoc", List.of("--python_out="
                                  + fileConfig.getSrcGenPath(), filename), fileConfig.srcPath);

        if (protoc == null) {
            errorReporter.reportError("Processing .proto files requires libprotoc >= 3.6.1");
            return;
        }
        int returnCode = protoc.run(cancelIndicator);
        if (returnCode == 0) {
            pythonRequiredModules.add("google-api-python-client");
        } else {
            errorReporter.reportError(
                "protoc returns error code " + returnCode);
        }
    }

    /**
     * Generate the aliases for inputs, outputs, and struct type definitions for
     * actions of the specified reactor in the specified federate.
     *
     * @param decl The parsed reactor decleration data structure.
     */
    @Override
    public void generateAuxiliaryStructs(
        ReactorDecl decl
    ) {
        Reactor reactor = ASTUtils.toDefinition(decl);
        // First, handle inputs.
        for (Input input : ASTUtils.allInputs(reactor)) {
            generateAuxiliaryStructsForPort(decl, input);
        }
        // Next, handle outputs.
        for (Output output : ASTUtils.allOutputs(reactor)) {
            generateAuxiliaryStructsForPort(decl, output);
        }
        // Finally, handle actions.
        for (Action action : ASTUtils.allActions(reactor)) {
            generateAuxiliaryStructsForAction(decl, action);
        }
    }

    private void generateAuxiliaryStructsForPort(ReactorDecl decl,
                                                 Port port) {
        boolean isTokenType = CUtil.isTokenType(ASTUtils.getInferredType(port), types);
        code.pr(port,
                PythonPortGenerator.generateAliasTypeDef(decl, port, isTokenType,
                                                         genericPortType));
    }

    private void generateAuxiliaryStructsForAction(ReactorDecl decl,
                                                   Action action) {
        code.pr(action, PythonActionGenerator.generateAliasTypeDef(decl, action, genericActionType));
    }

    /**
     * Return true if the host operating system is compatible and
     * otherwise report an error and return false.
     */
    @Override
    public boolean isOSCompatible() {
        return true;
    }

    /**
     * Generate C code from the Lingua Franca model contained by the
     * specified resource. This is the main entry point for code
     * generation.
     *
     * @param resource The resource containing the source code.
     * @param context  Context relating to invocation of the code generator.
     */
    @Override
    public void doGenerate(Resource resource, LFGeneratorContext context) {
        // Set the threading to false by default, unless the user has
        // specifically asked for it.
        if (!targetConfig.setByUser.contains(TargetProperty.THREADING)) {
            targetConfig.threading = false;
        }
        // Prevent the CGenerator from compiling the C code.
        // The PythonGenerator will compiler it.
        boolean compileStatus = targetConfig.noCompile;
        targetConfig.noCompile = true;
        targetConfig.useCmake = false; // Force disable the CMake because
        // it interferes with the Python target functionality
        int cGeneratedPercentProgress =
            (IntegratedBuilder.VALIDATED_PERCENT_PROGRESS + 100) / 2;
        super.doGenerate(resource, new SubContext(
            context,
            IntegratedBuilder.VALIDATED_PERCENT_PROGRESS,
            cGeneratedPercentProgress
        ));
        SubContext compilingFederatesContext = new SubContext(context, cGeneratedPercentProgress, 100);
        targetConfig.noCompile = compileStatus;

        if (errorsOccurred()) {
            context.unsuccessfulFinish();
            return;
        }

        Map<Path, CodeMap> codeMaps = new HashMap<>();
        var lfModuleName = fileConfig.name;
        // Don't generate code if there is no main reactor
        if (this.main != null) {
            try {
                Map<Path, CodeMap> codeMapsForFederate = generatePythonFiles(lfModuleName, generatePythonModuleName(lfModuleName), generatePythonFileName(lfModuleName));
                codeMaps.putAll(codeMapsForFederate);
                copyTargetFiles();
                if (!targetConfig.noCompile) {
                    // If there are no federates, compile and install the generated code
                    new PythonValidator(fileConfig, errorReporter, codeMaps, protoNames).doValidate(context);
                    if (!errorsOccurred()
                        && !Objects.equal(context.getMode(), LFGeneratorContext.Mode.LSP_MEDIUM)) {
                        pythonCompileCode(context); // Why is this invoked here if the current federate is not a parameter?
                    }
                } else {
                    System.out.println(PythonInfoGenerator.generateSetupInfo(fileConfig));
                }
            } catch (Exception e) {
                throw Exceptions.sneakyThrow(e);
            }

            System.out.println(PythonInfoGenerator.generateRunInfo(fileConfig, lfModuleName));
        }

        if (errorReporter.getErrorsOccurred()) {
            context.unsuccessfulFinish();
        } else {
            context.finish(GeneratorResult.Status.COMPILED, fileConfig.name
                               + ".py", fileConfig.getSrcGenPath(), fileConfig,
                           codeMaps, "python3");
        }
    }

    @Override
    protected CDockerGenerator getDockerGenerator() {
        return new PythonDockerGenerator(false, targetConfig);
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     *
     * @param action The action to schedule
     * @param port   The port to read from
     */
    @Override
    public String generateDelayBody(Action action, VarRef port) {
        return PythonReactionGenerator.generateCDelayBody(action, port, CUtil.isTokenType(ASTUtils.getInferredType(action), types));
    }

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port. This realizes
     * the receiving end of a logical delay specified with the 'after'
     * keyword.
     *
     * @param action The action that triggers the reaction
     * @param port   The port to write to.
     */
    @Override
    public String generateForwardBody(Action action, VarRef port) {
        String outputName = ASTUtils.generateVarRef(port);
        if (CUtil.isTokenType(ASTUtils.getInferredType(action), types)) {
            return super.generateForwardBody(action, port);
        } else {
            return "lf_set(" + outputName + ", " + action.getName()
                + "->token->value);";
        }
    }

    /**
     * Generate a reaction function definition for a reactor.
     * This function has a single argument that is a void* pointing to
     * a struct that contains parameters, state variables, inputs (triggering or
     * not),
     * actions (triggering or produced), and outputs.
     *
     * @param reaction      The reaction.
     * @param decl       The reactor declaration.
     * @param reactionIndex The position of the reaction within the reactor.
     */
    @Override
    public void generateReaction(Reaction reaction, ReactorDecl decl, int reactionIndex) {
        Reactor reactor = ASTUtils.toDefinition(decl);

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.getName().contains(GEN_DELAY_CLASS_NAME) ||
            ((mainDef != null && decl == mainDef.getReactorClass()
                || mainDef == decl) && reactor.isFederated())) {
            super.generateReaction(reaction, decl, reactionIndex);
            return;
        }
        code.pr(PythonReactionGenerator.generateCReaction(reaction, decl, reactionIndex, mainDef, errorReporter, types));
    }

    /**
     * Generate code that initializes the state variables for a given instance.
     * Unlike parameters, state variables are uniformly initialized for all
     * instances
     * of the same reactor. This task is left to Python code to allow for more
     * liberal
     * state variable assignments.
     *
     * @param instance The reactor class instance
     * @return Initialization code fore state variables of instance
     */
    @Override
    public void generateStateVariableInitializations(ReactorInstance instance) {
        // Do nothing
    }

    /**
     * Generate runtime initialization code in C for parameters of a given
     * reactor instance
     *
     * @param instance The reactor instance.
     */
    @Override
    public void generateParameterInitialization(ReactorInstance instance) {
        // Do nothing
        // Parameters are initialized in Python
    }

    /**
     * Generate C preambles defined by user for a given reactor
     * Since the Python generator expects preambles written in C,
     * this function is overridden and does nothing.
     *
     * @param reactor The given reactor
     */
    @Override
    public void generateUserPreamblesForReactor(Reactor reactor) {
        // Do nothing
    }

    /**
     * Generate code that is executed while the reactor instance is being
     * initialized.
     * This wraps the reaction functions in a Python function.
     *
     * @param instance  The reactor instance.
     */
    @Override
    public void generateReactorInstanceExtension(
        ReactorInstance instance
    ) {
        initializeTriggerObjects.pr(PythonReactionGenerator.generateCPythonReactionLinkers(instance, mainDef));
    }

    /**
     * This function is provided to allow extensions of the CGenerator to append
     * the structure of the self struct
     *
     * @param selfStructBody  The body of the self struct
     * @param decl            The reactor declaration for the self struct
     * @param constructorCode Code that is executed when the reactor is
     *                        instantiated
     */
    @Override
    public void generateSelfStructExtension(
        CodeBuilder selfStructBody,
        ReactorDecl decl,
        CodeBuilder constructorCode
    ) {
        Reactor reactor = ASTUtils.toDefinition(decl);
        // Add the name field
        selfStructBody.pr("char *_lf_name;");
        int reactionIndex = 0;
        for (Reaction reaction : ASTUtils.allReactions(reactor)) {
            // Create a PyObject for each reaction
            selfStructBody.pr("PyObject* "
                                  + PythonReactionGenerator.generateCPythonReactionFunctionName(reactionIndex)
                                  + ";");
            if (reaction.getStp() != null) {
                selfStructBody.pr("PyObject* "
                                      + PythonReactionGenerator.generateCPythonSTPFunctionName(reactionIndex)
                                      + ";");
            }
            if (reaction.getDeadline() != null) {
                selfStructBody.pr("PyObject* "
                                      + PythonReactionGenerator.generateCPythonDeadlineFunctionName(reactionIndex)
                                      + ";");
            }
            reactionIndex++;
        }
    }

    @Override
    protected String getConflictingConnectionsInModalReactorsBody(String source, String dest) {
        // NOTE: Strangely, a newline is needed at the beginning or indentation
        // gets swallowed.
        return String.join("\n",
                           "\n# Generated forwarding reaction for connections with the same destination",
                           "# but located in mutually exclusive modes.",
                           dest + ".set(" + source + ".value)\n"
        );
    }

    @Override
    protected void setUpGeneralParameters() {
        super.setUpGeneralParameters();
        if (hasModalReactors) {
            targetConfig.compileAdditionalSources.add("modal_models/impl.c");
        }
    }

    @Override
    protected void additionalPostProcessingForModes() {
        if (!hasModalReactors) {
            return;
        }
        PythonModeGenerator.generateResetReactionsIfNeeded(reactors);
    }

    /**
     * Generate a (`key`, `val`) tuple pair for the `define_macros` field
     * of the Extension class constructor from setuptools.
     *
     * @param key The key of the macro entry
     * @param val The value of the macro entry
     * @return A (`key`, `val`) tuple pair as String
     */
    private static String generateMacroEntry(String key, String val) {
        return "(" + StringUtil.addDoubleQuotes(key) + ", "
            + StringUtil.addDoubleQuotes(val) + ")";
    }

    /**
     * Generate the name of the python module.
     *
     * Ideally, this function would belong in a class like `PyFileConfig`
     * that specifies all the paths to the generated code.
     *
     * @param lfModuleName The name of the LF module.
     * @return The name of the python module.
     */
    private static String generatePythonModuleName(String lfModuleName) {
        return "LinguaFranca" + lfModuleName;
    }

    /**
     * Generate the python file name given an `lfModuleName`.
     *
     * Ideally, this function would belong in a class like `PyFileConfig`
     * that specifies all the paths to the generated code.
     *
     * @param lfModuleName The name of the LF module
     * @return The name of the generated python file.
     */
    private static String generatePythonFileName(String lfModuleName) {
        return lfModuleName + ".py";
    }

    /**
     * Copy Python specific target code to the src-gen directory
     */
    private void copyTargetFiles() throws IOException {
        // Copy the required target language files into the target file system.
        // This will also overwrite previous versions.
        FileUtil.copyDirectoryFromClassPath(
            "/lib/py/reactor-c-py/include",
            fileConfig.getSrcGenPath(),
            false
        );
        FileUtil.copyDirectoryFromClassPath(
            "/lib/py/reactor-c-py/lib",
            fileConfig.getSrcGenPath(),
            false
        );
    }
}
