/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.generator;

import com.google.common.base.Objects;
import com.google.common.collect.Iterables;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.eclipse.core.resources.IMarker;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.AttributeUtils;
import org.lflang.FileConfig;
import org.lflang.MainConflictChecker;
import org.lflang.MessageReporter;
import org.lflang.analyses.uclid.UclidGenerator;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.generator.docker.DockerComposeGenerator;
import org.lflang.generator.docker.DockerGenerator;
import org.lflang.graph.InstantiationGraph;
import org.lflang.lf.Attribute;
import org.lflang.lf.Connection;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.VerifyProperty;
import org.lflang.util.FileUtil;
import org.lflang.validation.AbstractLFValidator;

/**
 * Generator base class for specifying core functionality that all code generators should have.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Matt Weber
 * @author Soroush Bateni
 */
public abstract class GeneratorBase extends AbstractLFValidator {

  /** The main (top-level) reactor instance. */
  public ReactorInstance main;

  /** An error reporter for reporting any errors or warnings during the code generation */
  public MessageReporter messageReporter;

  /** The current target configuration. */
  protected final TargetConfig targetConfig;

  public TargetConfig getTargetConfig() {
    return this.targetConfig;
  }

  public final LFGeneratorContext context;

  /** A factory for compiler commands. */
  protected GeneratorCommandFactory commandFactory;

  public GeneratorCommandFactory getCommandFactory() {
    return commandFactory;
  }

  /**
   * Definition of the main (top-level) reactor. This is an automatically generated AST node for the
   * top-level reactor.
   */
  protected Instantiation mainDef;

  public Instantiation getMainDef() {
    return mainDef;
  }

  /**
   * A list of Reactor definitions in the main resource, including non-main reactors defined in
   * imported resources. These are ordered in the list in such a way that each reactor is preceded
   * by any reactor that it instantiates using a command like {@code foo = new Foo();}
   */
  protected List<Reactor> reactors = new ArrayList<>();

  /**
   * Graph that tracks dependencies between instantiations. This is a graph where each node is a
   * Reactor (not a ReactorInstance) and an arc from Reactor A to Reactor B means that B contains an
   * instance of A, constructed with a statement like {@code a = new A();} After creating the graph,
   * sort the reactors in topological order and assign them to the reactors class variable. Hence,
   * after this method returns, {@code this.reactors} will be a list of Reactors such that any
   * reactor is preceded in the list by reactors that it instantiates.
   */
  protected InstantiationGraph instantiationGraph;

  /** Map from reactions to bank indices */
  protected Map<Reaction, Integer> reactionBankIndices = null;

  /** Indicates whether the current Lingua Franca program contains model reactors. */
  public boolean hasModalReactors = false;

  /** Indicates whether the program has any watchdogs. This is used to check for support. */
  public boolean hasWatchdogs = false;

  /** A list ot AST transformations to apply before code generation */
  private final List<AstTransformation> astTransformations = new ArrayList<>();

  /** Create a new GeneratorBase object. */
  public GeneratorBase(LFGeneratorContext context) {
    this.context = context;
    this.targetConfig = context.getTargetConfig();
    this.messageReporter = context.getErrorReporter();
    this.commandFactory = new GeneratorCommandFactory(messageReporter, context.getFileConfig());
  }

  /**
   * Register an AST transformation to be applied to the AST.
   *
   * <p>The transformations will be applied in the order that they are registered in.
   */
  protected void registerTransformation(AstTransformation transformation) {
    astTransformations.add(transformation);
  }

  /**
   * If the given reactor is defined in another file, process its target properties so that they are
   * reflected in the target configuration.
   */
  private void loadTargetProperties(Resource resource) {
    var mainFileConfig = this.context.getFileConfig();
    if (resource != mainFileConfig.resource) {
      this.context
          .getTargetConfig()
          .mergeImportedConfig(
              LFGenerator.createFileConfig(
                      resource,
                      mainFileConfig.getSrcGenBasePath(),
                      mainFileConfig.useHierarchicalBin)
                  .resource,
              mainFileConfig.resource,
              p -> p.loadFromImport(),
              this.messageReporter);
    }
  }

  /**
   * Generate code from the Lingua Franca model contained by the specified resource.
   *
   * <p>This is the main entry point for code generation. This base class finds all reactor class
   * definitions, including any reactors defined in imported .lf files (except any main reactors in
   * those imported files), and adds them to the {@link GeneratorBase#reactors reactors} list. If
   * errors occur during generation, then a subsequent call to errorsOccurred() will return true.
   *
   * @param resource The resource containing the source code.
   * @param context Context relating to invocation of the code generator. In standalone mode, this
   *     object is also used to relay CLI arguments.
   */
  public void doGenerate(Resource resource, LFGeneratorContext context) {

    printInfo(context);

    // Clear any IDE markers that may have been created by a previous build.
    // Markers mark problems in the Eclipse IDE when running in integrated mode.
    messageReporter.clearHistory();

    // Configure the command factory
    commandFactory.setVerbose();
    if (Objects.equal(context.getMode(), LFGeneratorContext.Mode.STANDALONE)
        && context.getArgs().quiet()) {
      commandFactory.setQuiet();
    }

    // If "-c" or "--clean" is specified, delete any existing generated directories.
    cleanIfNeeded(context);

    // If @property annotations are used, run the LF verifier.
    runVerifierIfPropertiesDetected(resource, context);

    ASTUtils.setMainName(context.getFileConfig().resource, context.getFileConfig().name);

    createMainInstantiation();

    // Check if there are any conflicting main reactors elsewhere in the package.
    if (Objects.equal(context.getMode(), LFGeneratorContext.Mode.STANDALONE) && mainDef != null) {
      for (String conflict : new MainConflictChecker(context.getFileConfig()).conflicts) {
        EObject object = this.mainDef.getReactorClass();
        messageReporter.at(object).error("Conflicting main reactor in " + conflict);
      }
    }

    // Collect reactors and create an instantiation graph.
    // These are needed to figure out which resources we need
    // to validate, which happens in setResources().
    setReactorsAndInstantiationGraph(context.getMode());

    Set<Resource> allResources = GeneratorUtils.getResources(reactors);

    GeneratorUtils.accommodatePhysicalActionsIfPresent(
        allResources,
        getTarget().setsKeepAliveOptionAutomatically(),
        targetConfig,
        messageReporter);

    // Load target properties for all resources.
    allResources.forEach(r -> loadTargetProperties(r));

    for (AstTransformation transformation : astTransformations) {
      transformation.applyTransformation(reactors);
    }

    // Transform connections that reside in mutually exclusive modes and are otherwise conflicting
    // This should be done before creating the instantiation graph
    transformConflictingConnectionsInModalReactors(allResources);

    // Invoke these functions a second time because transformations
    // may have introduced new reactors!
    setReactorsAndInstantiationGraph(context.getMode());

    // Check for existence and support of modes
    hasModalReactors = IterableExtensions.exists(reactors, it -> !it.getModes().isEmpty());
    checkModalReactorSupport(false);

    // Check for the existence and support of watchdogs
    hasWatchdogs = IterableExtensions.exists(reactors, it -> !it.getWatchdogs().isEmpty());

    checkWatchdogSupport(
        getTarget() == Target.C && !targetConfig.get(SingleThreadedProperty.INSTANCE));
    additionalPostProcessingForModes();
  }

  /**
   * If there is a main or federated reactor, then create a synthetic Instantiation for that
   * top-level reactor and set the field mainDef to refer to it.
   */
  protected void createMainInstantiation() {
    // Find the main reactor and create an AST node for its instantiation.
    Iterable<EObject> nodes =
        IteratorExtensions.toIterable(context.getFileConfig().resource.getAllContents());
    for (Reactor reactor : Iterables.filter(nodes, Reactor.class)) {
      if (reactor.isMain()) {
        // Creating a definition for the main reactor because there isn't one.
        this.mainDef = LfFactory.eINSTANCE.createInstantiation();
        this.mainDef.setName(reactor.getName());
        this.mainDef.setReactorClass(reactor);
      }
    }
  }

  /**
   * Create a new instantiation graph. This is a graph where each node is a Reactor (not a
   * ReactorInstance) and an arc from Reactor A to Reactor B means that B contains an instance of A,
   * constructed with a statement like {@code a = new A();} After creating the graph, sort the
   * reactors in topological order and assign them to the reactors class variable. Hence, after this
   * method returns, {@code this.reactors} will be a list of Reactors such that any reactor is
   * preceded in the list by reactors that it instantiates.
   */
  protected void setReactorsAndInstantiationGraph(LFGeneratorContext.Mode mode) {
    // Build the instantiation graph .
    instantiationGraph = new InstantiationGraph(context.getFileConfig().resource, false);

    // Topologically sort the reactors such that all of a reactor's instantiation dependencies occur
    // earlier in
    // the sorted list of reactors. This helps the code generator output code in the correct order.
    // For example if {@code reactor Foo {bar = new Bar()}} then the definition of {@code Bar} has
    // to be generated before
    // the definition of {@code Foo}.
    reactors = instantiationGraph.nodesInTopologicalOrder();

    // If there is no main reactor or if all reactors in the file need to be validated, then make
    // sure the reactors
    // list includes even reactors that are not instantiated anywhere.
    if (mainDef == null || Objects.equal(mode, LFGeneratorContext.Mode.LSP_MEDIUM)) {
      Iterable<EObject> nodes =
          IteratorExtensions.toIterable(context.getFileConfig().resource.getAllContents());
      for (Reactor r : IterableExtensions.filter(nodes, Reactor.class)) {
        if (!reactors.contains(r)) {
          reactors.add(r);
        }
      }
    }
  }

  /**
   * Copy user specific files to the src-gen folder.
   *
   * <p>This should be overridden by the target generators.
   *
   * @param targetConfig The targetConfig to read the {@code files} from.
   * @param fileConfig The fileConfig used to make the copy and resolve paths.
   */
  protected void copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {
    if (targetConfig.isSet(FilesProperty.INSTANCE)) {
      var dst = this.context.getFileConfig().getSrcGenPath();
      FileUtil.copyFilesOrDirectories(
          targetConfig.get(FilesProperty.INSTANCE), dst, fileConfig, messageReporter, false);
    }
  }

  /**
   * Return true if errors occurred in the last call to doGenerate(). This will return true if any
   * of the reportError methods was called.
   *
   * @return True if errors occurred.
   */
  public boolean errorsOccurred() {
    return messageReporter.getErrorsOccurred();
  }

  /*
   * Return the TargetTypes instance associated with this.
   */
  public abstract TargetTypes getTargetTypes();

  /**
   * Mark the specified reaction to belong to only the specified bank index. This is needed because
   * reactions cannot declare a specific bank index as an effect or trigger. Reactions that send
   * messages between federates, including absent messages, need to be specific to a bank member.
   *
   * @param reaction The reaction.
   * @param bankIndex The bank index, or -1 if there is no bank.
   */
  public void setReactionBankIndex(Reaction reaction, int bankIndex) {
    if (bankIndex < 0) {
      return;
    }
    if (reactionBankIndices == null) {
      reactionBankIndices = new LinkedHashMap<>();
    }
    reactionBankIndices.put(reaction, bankIndex);
  }

  /**
   * Return the reaction bank index.
   *
   * @see #setReactionBankIndex(Reaction reaction, int bankIndex)
   * @param reaction The reaction.
   * @return The reaction bank index, if one has been set, and -1 otherwise.
   */
  public int getReactionBankIndex(Reaction reaction) {
    if (reactionBankIndices == null) return -1;
    if (reactionBankIndices.get(reaction) == null) return -1;
    return reactionBankIndices.get(reaction);
  }

  // //////////////////////////////////////////
  // // Protected methods.

  /**
   * Checks whether modal reactors are present and require appropriate code generation. This will
   * set the hasModalReactors variable.
   *
   * @param isSupported indicates if modes are supported by this code generation.
   */
  protected void checkModalReactorSupport(boolean isSupported) {
    if (hasModalReactors && !isSupported) {
      messageReporter
          .nowhere()
          .error(
              "The currently selected code generation or "
                  + "target configuration does not support modal reactors!");
    }
  }

  /**
   * Check whether watchdogs are present and are supported.
   *
   * @param isSupported indicates whether or not this is a supported target and whether or not it is
   *     a threaded runtime.
   */
  protected void checkWatchdogSupport(boolean isSupported) {
    if (hasWatchdogs && !isSupported) {
      messageReporter
          .nowhere()
          .error("Watchdogs are currently only supported for threaded programs in the C target.");
    }
  }

  /**
   * Finds and transforms connections into forwarding reactions iff the connections have the same
   * destination as other connections or reaction in mutually exclusive modes.
   */
  private void transformConflictingConnectionsInModalReactors(Set<Resource> resources) {
    for (Resource r : resources) {
      var transform = ASTUtils.findConflictingConnectionsInModalReactors(r);
      if (!transform.isEmpty()) {
        var factory = LfFactory.eINSTANCE;
        for (Connection connection : transform) {
          // Currently only simple transformations are supported
          if (connection.isPhysical()
              || connection.getDelay() != null
              || connection.isIterated()
              || connection.getLeftPorts().size() > 1
              || connection.getRightPorts().size() > 1) {
            messageReporter
                .at(connection)
                .error(
                    "Cannot transform connection in modal reactor. Connection uses currently not"
                        + " supported features.");
          } else {
            var reaction = factory.createReaction();
            ((Mode) connection.eContainer()).getReactions().add(reaction);

            var sourceRef = connection.getLeftPorts().get(0);
            var destRef = connection.getRightPorts().get(0);
            reaction.getTriggers().add(sourceRef);
            reaction.getEffects().add(destRef);

            var code = factory.createCode();
            var source =
                (sourceRef.getContainer() != null ? sourceRef.getContainer().getName() + "." : "")
                    + sourceRef.getVariable().getName();
            var dest =
                (destRef.getContainer() != null ? destRef.getContainer().getName() + "." : "")
                    + destRef.getVariable().getName();
            code.setBody(getConflictingConnectionsInModalReactorsBody(source, dest));
            reaction.setCode(code);

            EcoreUtil.remove(connection);
          }
        }
      }
    }
  }

  /**
   * Return target code for forwarding reactions iff the connections have the same destination as
   * other connections or reaction in mutually exclusive modes.
   *
   * <p>This method needs to be overridden in target specific code generators that support modal
   * reactors.
   */
  protected String getConflictingConnectionsInModalReactorsBody(String source, String dest) {
    messageReporter
        .nowhere()
        .error(
            "The currently selected code generation "
                + "is missing an implementation for conflicting "
                + "transforming connections in modal reactors.");
    return "MODAL MODELS NOT SUPPORTED";
  }

  /** Hook for additional post-processing of the model. */
  protected void additionalPostProcessingForModes() {
    // Do nothing
  }

  /** Parsed error message from a compiler is returned here. */
  public static class ErrorFileAndLine {
    public String filepath = null;
    public String line = "1";
    public String character = "0";
    public String message = "";
    public boolean isError = true; // false for a warning.

    @Override
    public String toString() {
      return (isError ? "Error" : "Non-error")
          + " at "
          + line
          + ":"
          + character
          + " of file "
          + filepath
          + ": "
          + message;
    }
  }

  /**
   * Given a line of text from the output of a compiler, return an instance of ErrorFileAndLine if
   * the line is recognized as the first line of an error message. Otherwise, return null. This base
   * class simply returns null.
   *
   * @param line A line of output from a compiler or other external tool that might generate errors.
   * @return If the line is recognized as the start of an error message, then return a class
   *     containing the path to the file on which the error occurred (or null if there is none), the
   *     line number (or the string "1" if there is none), the character position (or the string "0"
   *     if there is none), and the message (or an empty string if there is none).
   */
  protected ErrorFileAndLine parseCommandOutput(String line) {
    return null;
  }

  /**
   * Parse the specified string for command errors that can be reported using marks in the Eclipse
   * IDE. In this class, we attempt to parse the messages to look for file and line information,
   * thereby generating marks on the appropriate lines. This should not be called in standalone
   * mode.
   *
   * @param stderr The output on standard error of executing a command.
   */
  public void reportCommandErrors(String stderr) {
    // NOTE: If the VS Code branch passes code review, then this function,
    //  parseCommandOutput, and ErrorFileAndLine will be deleted soon after.
    // First, split the message into lines.
    String[] lines = stderr.split("\\r?\\n");
    StringBuilder message = new StringBuilder();
    Integer lineNumber = null;
    Path path = context.getFileConfig().srcFile;
    // In case errors occur within an imported file, record the original path.
    Path originalPath = path;

    int severity = IMarker.SEVERITY_ERROR;
    for (String line : lines) {
      ErrorFileAndLine parsed = parseCommandOutput(line);
      if (parsed != null) {
        // Found a new line number designator.
        // If there is a previously accumulated message, report it.
        if (message.length() > 0) {
          reportIssue(message, lineNumber, path, severity);

          if (!Objects.equal(originalPath.toFile(), path.toFile())) {
            // Report an error also in the top-level resource.
            // FIXME: It should be possible to descend through the import
            // statements to find which one matches and mark all the
            // import statements down the chain. But what a pain!
            if (severity == IMarker.SEVERITY_ERROR) {
              messageReporter.at(originalPath).error("Error in imported file: " + path);
            } else {
              messageReporter.at(originalPath).warning("Warning in imported file: " + path);
            }
          }
        }
        if (parsed.isError) {
          severity = IMarker.SEVERITY_ERROR;
        } else {
          severity = IMarker.SEVERITY_WARNING;
        }

        // Start accumulating a new message.
        message = new StringBuilder();
        // Append the message on the line number designator line.
        message.append(parsed.message);

        // Set the new line number.
        try {
          lineNumber = Integer.decode(parsed.line);
        } catch (Exception ex) {
          // Set the line number unknown.
          lineNumber = null;
        }
        // FIXME: Ignoring the position within the line.
        // Determine the path within which the error occurred.
        path = Paths.get(parsed.filepath);
      } else {
        // No line designator.
        if (message.length() > 0) {
          message.append("\n");
        } else {
          if (!line.toLowerCase().contains("error:")) {
            severity = IMarker.SEVERITY_WARNING;
          }
        }
        message.append(line);
      }
    }
    if (message.length() > 0) {
      reportIssue(message, lineNumber, path, severity);

      if (originalPath.toFile() != path.toFile()) {
        // Report an error also in the top-level resource.
        // FIXME: It should be possible to descend through the import
        // statements to find which one matches and mark all the
        // import statements down the chain. But what a pain!
        if (severity == IMarker.SEVERITY_ERROR) {
          messageReporter.at(originalPath).error("Error in imported file: " + path);
        } else {
          messageReporter.at(originalPath).warning("Warning in imported file: " + path);
        }
      }
    }
  }

  /** Check if a clean was requested from the standalone compiler and perform the clean step. */
  protected void cleanIfNeeded(LFGeneratorContext context) {
    if (context.isCleanRequested()) {
      try {
        context.getFileConfig().doClean();
      } catch (IOException e) {
        System.err.println("WARNING: IO Error during clean");
      }
    }
  }

  /** Return a {@code DockerGenerator} instance suitable for the target. */
  protected abstract DockerGenerator getDockerGenerator(LFGeneratorContext context);

  /** Create Dockerfiles and docker-compose.yml, build, and create a launcher. */
  protected boolean buildUsingDocker() {
    // Create docker file.
    var dockerCompose = new DockerComposeGenerator(context);
    var dockerData = getDockerGenerator(context).generateDockerData();
    try {
      dockerData.writeDockerFile();
      dockerData.copyScripts(context);
      dockerCompose.writeDockerComposeFile(List.of(dockerData));
    } catch (IOException e) {
      context
          .getErrorReporter()
          .nowhere()
          .error(
              "Error while writing Docker files: "
                  + (e.getMessage() == null ? "No cause given" : e.getMessage()));
      return false;
    }
    return dockerCompose.buildIfRequested();
  }

  /**
   * Check if @property is used. If so, instantiate a UclidGenerator. The verification model needs
   * to be generated before the target code since code generation changes LF program (desugar
   * connections, etc.).
   */
  private void runVerifierIfPropertiesDetected(Resource resource, LFGeneratorContext lfContext) {
    Optional<Reactor> mainOpt = ASTUtils.getMainReactor(resource);
    if (mainOpt.isEmpty()) return;
    Reactor main = mainOpt.get();
    final MessageReporter messageReporter = lfContext.getErrorReporter();
    List<Attribute> properties =
        AttributeUtils.getAttributes(main).stream()
            .filter(attr -> attr.getAttrName().equals("property"))
            .collect(Collectors.toList());
    if (properties.size() > 0) {

      // Provide a warning.
      messageReporter
          .nowhere()
          .warning(
              "Verification using \"@property\" and \"--verify\" is an experimental feature. Use"
                  + " with caution.");

      // Generate uclid files.
      UclidGenerator uclidGenerator = new UclidGenerator(lfContext, properties);
      uclidGenerator.doGenerate(resource, lfContext);

      // Check the generated uclid files.
      if (uclidGenerator.targetConfig.get(VerifyProperty.INSTANCE)) {

        // Check if Uclid5 and Z3 are installed.
        if (commandFactory.createCommand("uclid", List.of()) == null
            || commandFactory.createCommand("z3", List.of()) == null) {
          messageReporter
              .nowhere()
              .error(
                  "Fail to check the generated verification models because Uclid5 or Z3 is not"
                      + " installed.");
        } else {
          // Run the Uclid tool.
          uclidGenerator.runner.run();
        }

      } else {
        messageReporter
            .nowhere()
            .warning(
                "The \"verify\" target property is set to false. Skip checking the verification"
                    + " model. To check the generated verification models, set the \"verify\""
                    + " target property to true or pass \"--verify\" to the lfc command");
      }
    }
  }

  private void reportIssue(StringBuilder message, Integer lineNumber, Path path, int severity) {
    DiagnosticSeverity convertedSeverity =
        severity == IMarker.SEVERITY_ERROR ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
    messageReporter.atNullableLine(path, lineNumber).report(convertedSeverity, message.toString());
  }

  // //////////////////////////////////////////////////
  // // Private functions

  /**
   * Print to stdout information about what source file is being generated, what mode the generator
   * is in, and where the generated sources are to be put.
   */
  public void printInfo(LFGeneratorContext context) {
    messageReporter
        .nowhere()
        .info("Generating code for: " + context.getFileConfig().resource.getURI().toString());
    messageReporter.nowhere().info("Generation mode: " + context.getMode());
    messageReporter
        .nowhere()
        .info("Generating sources into: " + context.getFileConfig().getSrcGenPath());
    messageReporter.nowhere().info(context.getTargetConfig().settings());
  }

  /** Return the Targets enum for the current target */
  public abstract Target getTarget();
}
