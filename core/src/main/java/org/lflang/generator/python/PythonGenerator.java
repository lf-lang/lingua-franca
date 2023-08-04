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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.AttributeUtils;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.CodeMap;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.SubContext;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.c.CCmakeGenerator;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Action;
import org.lflang.lf.Input;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;
import org.lflang.util.StringUtil;

/**
 * Generator for Python target. This class generates Python code defining each reactor class given
 * in the input .lf file and imported .lf files.
 *
 * <p>Each class will contain all the reaction functions defined by the user in order, with the
 * necessary ports/actions given as parameters. Moreover, each class will contain all state
 * variables in native Python format.
 *
 * <p>A backend is also generated using the CGenerator that interacts with the C code library (see
 * CGenerator.xtend). The backend is responsible for passing arguments to the Python reactor
 * functions.
 *
 * @author Soroush Bateni
 */
public class PythonGenerator extends CGenerator {

  // Used to add statements that come before reactor classes and user code
  private final CodeBuilder pythonPreamble = new CodeBuilder();

  // Used to add module requirements to setup.py (delimited with ,)
  private final List<String> pythonRequiredModules = new ArrayList<>();

  private final PythonTypes types;

  public PythonGenerator(LFGeneratorContext context) {
    this(
        context,
        new PythonTypes(),
        new CCmakeGenerator(
            context.getFileConfig(),
            List.of(
                "lib/python_action.c",
                "lib/python_port.c",
                "lib/python_tag.c",
                "lib/python_time.c",
                "lib/pythontarget.c"),
            PythonGenerator::setUpMainTarget,
            generateCmakeInstall(context.getFileConfig())));
  }

  private PythonGenerator(
      LFGeneratorContext context, PythonTypes types, CCmakeGenerator cmakeGenerator) {
    super(context, false, types, cmakeGenerator, new PythonDelayBodyGenerator(types));
    this.targetConfig.compiler = "gcc";
    this.targetConfig.compilerFlags = new ArrayList<>();
    this.targetConfig.linkerFlags = "";
    this.types = types;
  }

  /**
   * Generic struct for ports with primitive types and statically allocated arrays in Lingua Franca.
   * This template is defined as typedef struct { bool is_present; lf_sparse_io_record_t*
   * sparse_record; // NULL if there is no sparse record. int destination_channel; // -1 if there is
   * no destination. PyObject* value; int num_destinations; lf_token_t* token; int length; void
   * (*destructor) (void* value); void* (*copy_constructor) (void* value);
   * FEDERATED_GENERIC_EXTENSION } generic_port_instance_struct;
   *
   * <p>See reactor-c/python/lib/pythontarget.h for details.
   */
  String genericPortType = "generic_port_instance_struct";

  /**
   * Generic struct for actions. This template is defined as typedef struct { trigger_t* trigger;
   * PyObject* value; bool is_present; bool has_value; lf_token_t* token;
   * FEDERATED_CAPSULE_EXTENSION } generic_action_instance_struct;
   *
   * <p>See reactor-c/python/lib/pythontarget.h for details.
   */
  String genericActionType = "generic_action_instance_struct";

  /** Returns the Target enum for this generator */
  @Override
  public Target getTarget() {
    return Target.Python;
  }

  private final Set<String> protoNames = new HashSet<>();

  // //////////////////////////////////////////
  // // Public methods
  @Override
  public TargetTypes getTargetTypes() {
    return types;
  }

  // //////////////////////////////////////////
  // // Protected methods

  /** Generate all Python classes if they have a reaction */
  public String generatePythonReactorClasses() {
    CodeBuilder pythonClasses = new CodeBuilder();
    CodeBuilder pythonClassesInstantiation = new CodeBuilder();

    // Generate reactor classes in Python
    pythonClasses.pr(PythonReactorGenerator.generatePythonClass(main, main, types));

    // Create empty lists to hold reactor instances
    pythonClassesInstantiation.pr(PythonReactorGenerator.generateListsToHoldClassInstances(main));

    // Instantiate generated classes
    pythonClassesInstantiation.pr(
        PythonReactorGenerator.generatePythonClassInstantiations(main, main));

    return String.join(
        "\n",
        pythonClasses.toString(),
        "",
        "# Instantiate classes",
        pythonClassesInstantiation.toString());
  }

  /**
   * Generate the Python code constructed from reactor classes and user-written classes.
   *
   * @return the code body
   */
  public String generatePythonCode(String pyModuleName) {
    return String.join(
        "\n",
        "import os",
        "import sys",
        "sys.path.append(os.path.dirname(__file__))",
        "# List imported names, but do not use pylint's --extension-pkg-allow-list option",
        "# so that these names will be assumed present without having to compile and install.",
        "# pylint: disable=no-name-in-module, import-error",
        "from " + pyModuleName + " import (",
        "    Tag, action_capsule_t, port_capsule, request_stop, schedule_copy, start",
        ")",
        "# pylint: disable=c-extension-no-member",
        "import " + pyModuleName + " as lf",
        "try:",
        "    from LinguaFrancaBase.constants import BILLION, FOREVER, NEVER, instant_t, interval_t",
        "    from LinguaFrancaBase.functions import (",
        "        DAY, DAYS, HOUR, HOURS, MINUTE, MINUTES, MSEC, MSECS, NSEC, NSECS, SEC, SECS,"
            + " USEC,",
        "        USECS, WEEK, WEEKS",
        "    )",
        "    from LinguaFrancaBase.classes import Make",
        "except ModuleNotFoundError:",
        "    print(\"No module named 'LinguaFrancaBase'. \"",
        "          \"Install using \\\"pip3 install LinguaFrancaBase\\\".\")",
        "    sys.exit(1)",
        "import copy",
        "",
        pythonPreamble.toString(),
        "",
        generatePythonReactorClasses(),
        "",
        PythonMainFunctionGenerator.generateCode());
  }

  /** Generate the necessary Python files. */
  public Map<Path, CodeMap> generatePythonFiles(
      String lfModuleName, String pyModuleName, String pyFileName) throws IOException {
    Path filePath = fileConfig.getSrcGenPath().resolve(pyFileName);
    File file = filePath.toFile();
    Files.deleteIfExists(filePath);
    // Create the necessary directories
    if (!file.getParentFile().exists()) {
      if (!file.getParentFile().mkdirs()) {
        throw new IOException(
            "Failed to create directories required for the Python code generator.");
      }
    }
    Map<Path, CodeMap> codeMaps = new HashMap<>();
    codeMaps.put(filePath, CodeMap.fromGeneratedCode(generatePythonCode(pyModuleName)));
    FileUtil.writeToFile(codeMaps.get(filePath).getGeneratedCode(), filePath);
    return codeMaps;
  }

  /**
   * Generate code that needs to appear at the top of the generated C file, such as #define and
   * #include statements.
   */
  @Override
  public String generateDirectives() {
    CodeBuilder code = new CodeBuilder();
    code.prComment("Code generated by the Lingua Franca compiler from:");
    code.prComment("file:/" + FileUtil.toUnixString(fileConfig.srcFile));
    code.pr(
        PythonPreambleGenerator.generateCDefineDirectives(
            targetConfig, fileConfig.getSrcGenPath(), hasModalReactors));
    return code.toString();
  }

  /**
   * Override generate top-level preambles, but put the user preambles in the .py file rather than
   * the C file. Also handles including the federated execution setup preamble specified in the
   * target config.
   */
  @Override
  protected String generateTopLevelPreambles(Reactor ignored) {
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
    return PythonPreambleGenerator.generateCIncludeStatements(
        targetConfig, targetLanguageIsCpp(), hasModalReactors);
  }

  @Override
  protected void handleProtoFiles() {
    for (String name : targetConfig.protoFiles) {
      this.processProtoFile(name);
      int dotIndex = name.lastIndexOf(".");
      String rootFilename = dotIndex > 0 ? name.substring(0, dotIndex) : name;
      pythonPreamble.pr("import " + rootFilename + "_pb2 as " + rootFilename);
      protoNames.add(rootFilename);
    }
  }

  /**
   * Process a given .proto file.
   *
   * <p>Run, if possible, the proto-c protocol buffer code generator to produce the required .h and
   * .c files.
   *
   * @param filename Name of the file to process.
   */
  @Override
  public void processProtoFile(String filename) {
    LFCommand protoc =
        commandFactory.createCommand(
            "protoc",
            List.of("--python_out=" + fileConfig.getSrcGenPath(), filename),
            fileConfig.srcPath);

    if (protoc == null) {
      messageReporter.nowhere().error("Processing .proto files requires libprotoc >= 3.6.1");
      return;
    }
    int returnCode = protoc.run();
    if (returnCode == 0) {
      pythonRequiredModules.add("google-api-python-client");
    } else {
      messageReporter.nowhere().error("protoc returns error code " + returnCode);
    }
  }

  /**
   * Generate the aliases for inputs, outputs, and struct type definitions for actions of the
   * specified reactor in the specified federate.
   *
   * @param tpr The concrete reactor class.
   */
  @Override
  public void generateAuxiliaryStructs(
      CodeBuilder builder, TypeParameterizedReactor tpr, boolean userFacing) {
    for (Input input : ASTUtils.allInputs(tpr.reactor())) {
      generateAuxiliaryStructsForPort(builder, tpr, input);
    }
    for (Output output : ASTUtils.allOutputs(tpr.reactor())) {
      generateAuxiliaryStructsForPort(builder, tpr, output);
    }
    for (Action action : ASTUtils.allActions(tpr.reactor())) {
      generateAuxiliaryStructsForAction(builder, tpr, action);
    }
  }

  private void generateAuxiliaryStructsForPort(
      CodeBuilder builder, TypeParameterizedReactor tpr, Port port) {
    boolean isTokenType = CUtil.isTokenType(ASTUtils.getInferredType(port), types);
    builder.pr(
        port, PythonPortGenerator.generateAliasTypeDef(tpr, port, isTokenType, genericPortType));
  }

  private void generateAuxiliaryStructsForAction(
      CodeBuilder builder, TypeParameterizedReactor tpr, Action action) {
    builder.pr(action, PythonActionGenerator.generateAliasTypeDef(tpr, action, genericActionType));
  }

  /**
   * Return true if the host operating system is compatible and otherwise report an error and return
   * false.
   */
  @Override
  public boolean isOSCompatible() {
    return true;
  }

  /**
   * Generate C code from the Lingua Franca model contained by the specified resource. This is the
   * main entry point for code generation.
   *
   * @param resource The resource containing the source code.
   * @param context Context relating to invocation of the code generator.
   */
  @Override
  public void doGenerate(Resource resource, LFGeneratorContext context) {
    // Set the threading to false by default, unless the user has
    // specifically asked for it.
    if (!targetConfig.setByUser.contains(TargetProperty.THREADING)) {
      targetConfig.threading = false;
    }
    int cGeneratedPercentProgress = (IntegratedBuilder.VALIDATED_PERCENT_PROGRESS + 100) / 2;
    code.pr(
        PythonPreambleGenerator.generateCIncludeStatements(
            targetConfig, targetLanguageIsCpp(), hasModalReactors));
    super.doGenerate(
        resource,
        new SubContext(
            context, IntegratedBuilder.VALIDATED_PERCENT_PROGRESS, cGeneratedPercentProgress));

    if (errorsOccurred()) {
      context.unsuccessfulFinish();
      return;
    }

    Map<Path, CodeMap> codeMaps = new HashMap<>();
    var lfModuleName = fileConfig.name;
    // Don't generate code if there is no main reactor
    if (this.main != null) {
      try {
        Map<Path, CodeMap> codeMapsForFederate =
            generatePythonFiles(
                lfModuleName,
                generatePythonModuleName(lfModuleName),
                generatePythonFileName(lfModuleName));
        codeMaps.putAll(codeMapsForFederate);
        copyTargetFiles();
        new PythonValidator(fileConfig, messageReporter, codeMaps, protoNames).doValidate(context);
      } catch (Exception e) {
        //noinspection ConstantConditions
        throw Exceptions.sneakyThrow(e);
      }
    }

    if (messageReporter.getErrorsOccurred()) {
      context.unsuccessfulFinish();
    } else {
      context.finish(GeneratorResult.Status.COMPILED, codeMaps);
    }
  }

  @Override
  protected PythonDockerGenerator getDockerGenerator(LFGeneratorContext context) {
    return new PythonDockerGenerator(context);
  }

  /**
   * Generate a reaction function definition for a reactor. This function has a single argument that
   * is a void* pointing to a struct that contains parameters, state variables, inputs (triggering
   * or not), actions (triggering or produced), and outputs.
   *
   * @param reaction The reaction.
   * @param tpr The reactor.
   * @param reactionIndex The position of the reaction within the reactor.
   */
  @Override
  protected void generateReaction(
      CodeBuilder src, Reaction reaction, TypeParameterizedReactor tpr, int reactionIndex) {
    Reactor reactor = ASTUtils.toDefinition(tpr.reactor());

    // Reactions marked with a {@code @_c_body} attribute are generated in C
    if (AttributeUtils.hasCBody(reaction)) {
      super.generateReaction(src, reaction, tpr, reactionIndex);
      return;
    }
    src.pr(
        PythonReactionGenerator.generateCReaction(
            reaction, tpr, reactor, reactionIndex, mainDef, messageReporter, types));
  }

  /**
   * Generate code that initializes the state variables for a given instance. Unlike parameters,
   * state variables are uniformly initialized for all instances of the same reactor. This task is
   * left to Python code to allow for more liberal state variable assignments.
   *
   * @param instance The reactor class instance
   * @return Initialization code fore state variables of instance
   */
  @Override
  protected void generateStateVariableInitializations(ReactorInstance instance) {
    // Do nothing
  }

  /**
   * Generate runtime initialization code in C for parameters of a given reactor instance
   *
   * @param instance The reactor instance.
   */
  @Override
  protected void generateParameterInitialization(ReactorInstance instance) {
    // Do nothing
    // Parameters are initialized in Python
  }

  /**
   * Do nothing. Methods are generated in Python not C.
   *
   * @see PythonMethodGenerator
   */
  @Override
  protected void generateMethods(CodeBuilder src, TypeParameterizedReactor reactor) {}

  /**
   * Generate C preambles defined by user for a given reactor Since the Python generator expects
   * preambles written in C, this function is overridden and does nothing.
   *
   * @param reactor The given reactor
   */
  @Override
  protected void generateUserPreamblesForReactor(Reactor reactor, CodeBuilder src) {
    // Do nothing
  }

  @Override
  protected void generateReactorClassHeaders(
      TypeParameterizedReactor tpr, String headerName, CodeBuilder header, CodeBuilder src) {
    header.pr(
        PythonPreambleGenerator.generateCIncludeStatements(
            targetConfig, targetLanguageIsCpp(), hasModalReactors));
    super.generateReactorClassHeaders(tpr, headerName, header, src);
  }

  /**
   * Generate code that is executed while the reactor instance is being initialized. This wraps the
   * reaction functions in a Python function.
   *
   * @param instance The reactor instance.
   */
  @Override
  protected void generateReactorInstanceExtension(ReactorInstance instance) {
    initializeTriggerObjects.pr(
        PythonReactionGenerator.generateCPythonReactionLinkers(instance, mainDef));
  }

  /**
   * This function is provided to allow extensions of the CGenerator to append the structure of the
   * self struct
   *
   * @param selfStructBody The body of the self struct
   * @param reactor The reactor declaration for the self struct
   * @param constructorCode Code that is executed when the reactor is instantiated
   */
  @Override
  protected void generateSelfStructExtension(
      CodeBuilder selfStructBody, Reactor reactor, CodeBuilder constructorCode) {
    // Add the name field
    selfStructBody.pr("char *_lf_name;");
    int reactionIndex = 0;
    for (Reaction reaction : ASTUtils.allReactions(reactor)) {
      // Create a PyObject for each reaction
      selfStructBody.pr(
          "PyObject* "
              + PythonReactionGenerator.generateCPythonReactionFunctionName(reactionIndex)
              + ";");
      if (reaction.getStp() != null) {
        selfStructBody.pr(
            "PyObject* "
                + PythonReactionGenerator.generateCPythonSTPFunctionName(reactionIndex)
                + ";");
      }
      if (reaction.getDeadline() != null) {
        selfStructBody.pr(
            "PyObject* "
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
    return String.join(
        "\n",
        "\n# Generated forwarding reaction for connections with the same destination",
        "# but located in mutually exclusive modes.",
        dest + ".set(" + source + ".value)\n");
  }

  @Override
  protected void setUpGeneralParameters() {
    super.setUpGeneralParameters();
    if (hasModalReactors) {
      targetConfig.compileAdditionalSources.add("lib/modal_models/impl.c");
    }
  }

  @Override
  protected void additionalPostProcessingForModes() {
    if (!hasModalReactors) {
      return;
    }
    PythonModeGenerator.generateResetReactionsIfNeeded(reactors);
  }

  private static String setUpMainTarget(
      boolean hasMain, String executableName, Stream<String> cSources) {
    return ("""
            set(CMAKE_POSITION_INDEPENDENT_CODE ON)
            add_compile_definitions(_PYTHON_TARGET_ENABLED)
            add_subdirectory(core)
            set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
            set(LF_MAIN_TARGET <pyModuleName>)
            find_package(Python 3.10.0...<3.11.0 REQUIRED COMPONENTS Interpreter Development)
            Python_add_library(
                ${LF_MAIN_TARGET}
                MODULE
            """
            + cSources.collect(Collectors.joining("\n    ", "    ", "\n"))
            + """
            )
            if (MSVC)
                set_target_properties(${LF_MAIN_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
                set_target_properties(${LF_MAIN_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR})
                set_target_properties(${LF_MAIN_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR})
                set_target_properties(${LF_MAIN_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_SOURCE_DIR})
                set_target_properties(${LF_MAIN_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR})
            endif (MSVC)
            set_target_properties(${LF_MAIN_TARGET} PROPERTIES PREFIX "")
            include_directories(${Python_INCLUDE_DIRS})
            target_link_libraries(${LF_MAIN_TARGET} PRIVATE ${Python_LIBRARIES})
            target_compile_definitions(${LF_MAIN_TARGET} PUBLIC MODULE_NAME=<pyModuleName>)
            """)
        .replace("<pyModuleName>", generatePythonModuleName(executableName));
    // The use of fileConfig.name will break federated execution, but that's fine
  }

  private static String generateCmakeInstall(FileConfig fileConfig) {
    final var pyMainPath =
        fileConfig.getSrcGenPath().resolve(fileConfig.name + ".py").toAbsolutePath();
    // need to replace '\' with '\\' on Windwos for proper escaping in cmake
    final var pyMainName = pyMainPath.toString().replace("\\", "\\\\");
    return """
              if(WIN32)
                file(GENERATE OUTPUT <fileName>.bat CONTENT
                  "@echo off\n\
                  ${Python_EXECUTABLE} <pyMainName> %*"
                )
                install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/<fileName>.bat DESTINATION ${CMAKE_INSTALL_BINDIR})
              else()
                file(GENERATE OUTPUT <fileName> CONTENT
                    "#!/bin/sh\\n\\
                    ${Python_EXECUTABLE} <pyMainName> \\\"$@\\\""
                )
                install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/<fileName> DESTINATION ${CMAKE_INSTALL_BINDIR})
              endif()
            """
        .replace("<fileName>", fileConfig.name)
        .replace("<pyMainName>", pyMainName);
  }

  /**
   * Generate a ({@code key}, {@code val}) tuple pair for the {@code define_macros} field of the
   * Extension class constructor from setuptools.
   *
   * @param key The key of the macro entry
   * @param val The value of the macro entry
   * @return A ({@code key}, {@code val}) tuple pair as String
   */
  private static String generateMacroEntry(String key, String val) {
    return "(" + StringUtil.addDoubleQuotes(key) + ", " + StringUtil.addDoubleQuotes(val) + ")";
  }

  /**
   * Generate the name of the python module.
   *
   * <p>Ideally, this function would belong in a class like {@code PyFileConfig} that specifies all
   * the paths to the generated code.
   *
   * @param lfModuleName The name of the LF module.
   * @return The name of the python module.
   */
  private static String generatePythonModuleName(String lfModuleName) {
    return "LinguaFranca" + lfModuleName;
  }

  /**
   * Generate the python file name given an {@code lfModuleName}.
   *
   * <p>Ideally, this function would belong in a class like {@code PyFileConfig} that specifies all
   * the paths to the generated code.
   *
   * @param lfModuleName The name of the LF module
   * @return The name of the generated python file.
   */
  private static String generatePythonFileName(String lfModuleName) {
    return lfModuleName + ".py";
  }

  /** Copy Python specific target code to the src-gen directory */
  @Override
  protected void copyTargetFiles() throws IOException {
    super.copyTargetFiles();
    FileUtil.copyFromClassPath(
        "/lib/c/reactor-c/python/include", fileConfig.getSrcGenPath(), true, false);
    FileUtil.copyFromClassPath(
        "/lib/c/reactor-c/python/lib", fileConfig.getSrcGenPath(), true, false);
    FileUtil.copyFromClassPath(
        "/lib/py/lf-python-support/LinguaFrancaBase", fileConfig.getSrcGenPath(), true, false);
  }
}
