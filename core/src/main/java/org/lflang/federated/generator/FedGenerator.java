package org.lflang.federated.generator;

import static org.lflang.generator.DockerGenerator.dockerGeneratorFactory;

import com.google.inject.Injector;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.util.RuntimeIOException;
import org.lflang.FileConfig;
import org.lflang.LFStandaloneSetup;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.launcher.FedLauncherGenerator;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.CodeMap;
import org.lflang.generator.DockerData;
import org.lflang.generator.FedDockerComposeGenerator;
import org.lflang.generator.GeneratorResult.Status;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.MixedRadixInt;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstanceGraph;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.SubContext;
import org.lflang.lf.Expression;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.VarRef;
import org.lflang.util.Averager;

public class FedGenerator {

  /** */
  private final MessageReporter messageReporter;

  /** A list of federate instances. */
  private final List<FederateInstance> federates = new ArrayList<>();

  /**
   * File configuration to be used during the LF code generation stage (not the target code
   * generation stage of individual federates).
   */
  private final FedFileConfig fileConfig;

  /** Configuration of the RTI. */
  final RtiConfig rtiConfig = new RtiConfig();

  /**
   * Target configuration of the federation; drawn from the file in which the federated reactor is
   * defined.
   */
  private final TargetConfig targetConfig;

  /**
   * A map from instantiations to the federate instances for that instantiation. If the
   * instantiation has a width, there may be more than one federate instance.
   */
  private Map<Instantiation, List<FederateInstance>> federatesByInstantiation;

  /**
   * Definition of the main (top-level) reactor. This is an automatically generated AST node for the
   * top-level reactor.
   */
  private Instantiation mainDef;

  /**
   * Create a new generator and initialize a file configuration, target configuration, and error
   * reporter.
   */
  public FedGenerator(LFGeneratorContext context) {
    this.fileConfig = (FedFileConfig) context.getFileConfig();
    this.targetConfig = context.getTargetConfig();
    this.messageReporter = context.getErrorReporter();
  }

  /**
   * Produce LF code for each federate in a separate file, then invoke a target-specific code
   * generator for each of those files.
   *
   * @param resource The resource that has the federated main reactor in it
   * @param context The context in which to carry out the code generation.
   * @return False if no errors have occurred, true otherwise.
   */
  public boolean doGenerate(Resource resource, LFGeneratorContext context) throws IOException {
    if (!federatedExecutionIsSupported(resource)) return true;
    cleanIfNeeded(context);

    // In a federated execution, we need keepalive to be true,
    // otherwise a federate could exit simply because it hasn't received
    // any messages.
    targetConfig.keepalive = true;

    // Process command-line arguments
    processCLIArguments(context);

    // Find the federated reactor
    Reactor federation = FedASTUtils.findFederatedReactor(resource);

    // Extract some useful information about the federation
    analyzeFederates(federation, context);

    // Find all the connections between federates.
    // For each connection between federates, replace it in the
    // AST with an action (which inherits the delay) and three reactions.
    // The action will be physical for physical connections and logical
    // for logical connections.
    replaceFederateConnectionsWithProxies(federation, resource);

    FedEmitter fedEmitter =
        new FedEmitter(
            fileConfig,
            ASTUtils.toDefinition(mainDef.getReactorClass()),
            messageReporter,
            rtiConfig);

    // Generate LF code for each federate.
    Map<Path, CodeMap> lf2lfCodeMapMap = new HashMap<>();
    for (FederateInstance federate : federates) {
      lf2lfCodeMapMap.putAll(fedEmitter.generateFederate(context, federate, federates.size()));
    }

    // Do not invoke target code generators if --no-compile flag is used.
    if (context.getTargetConfig().noCompile) {
      context.finish(Status.GENERATED, lf2lfCodeMapMap);
      return false;
    }

    Map<Path, CodeMap> codeMapMap =
        compileFederates(
            context,
            lf2lfCodeMapMap,
            subContexts -> {
              createDockerFiles(context, subContexts);
              generateLaunchScript();
              // If an error has occurred during codegen of any federate, report it.
              subContexts.forEach(
                  c -> {
                    if (c.getErrorReporter().getErrorsOccurred()) {
                      context
                          .getErrorReporter()
                          .at(c.getFileConfig().srcFile)
                          .error("Failure during code generation of " + c.getFileConfig().srcFile);
                    }
                  });
            });

    context.finish(Status.COMPILED, codeMapMap);
    return false;
  }

  private void generateLaunchScript() {
    new FedLauncherGenerator(this.targetConfig, this.fileConfig, this.messageReporter)
        .doGenerate(federates, rtiConfig);
  }

  /**
   * Generate a Dockerfile for each federate and a docker-compose.yml for the federation.
   *
   * @param context The main context in which the federation has been compiled.
   * @param subContexts The subcontexts in which the federates have been compiled.
   */
  private void createDockerFiles(LFGeneratorContext context, List<SubContext> subContexts) {
    if (context.getTargetConfig().dockerOptions == null) return;
    final List<DockerData> services = new ArrayList<>();
    // 1. create a Dockerfile for each federate
    for (SubContext subContext : subContexts) { // Inherit Docker options from main context
      subContext.getTargetConfig().dockerOptions = context.getTargetConfig().dockerOptions;
      var dockerGenerator = dockerGeneratorFactory(subContext);
      var dockerData = dockerGenerator.generateDockerData();
      try {
        dockerData.writeDockerFile();
      } catch (IOException e) {
        throw new RuntimeIOException(e);
      }
      services.add(dockerData);
    }
    // 2. create a docker-compose.yml for the federation
    try {
      new FedDockerComposeGenerator(context, rtiConfig.getHost()).writeDockerComposeFile(services);
    } catch (IOException e) {
      throw new RuntimeIOException(e);
    }
  }

  /**
   * Check if a clean was requested from the standalone compiler and perform the clean step.
   *
   * @param context Context in which the generator operates
   */
  private void cleanIfNeeded(LFGeneratorContext context) {
    if (context.getArgs().containsKey(BuildParm.CLEAN.getKey())) {
      try {
        fileConfig.doClean();
      } catch (IOException e) {
        System.err.println("WARNING: IO Error during clean");
      }
    }
  }

  /** Return whether federated execution is supported for {@code resource}. */
  private boolean federatedExecutionIsSupported(Resource resource) {
    TargetDecl targetDecl = GeneratorUtils.findTargetDecl(resource);
    var target = Target.fromDecl(targetDecl);
    var targetOK =
        List.of(Target.C, Target.Python, Target.TS, Target.CPP, Target.CCPP).contains(target);
    if (!targetOK) {
      messageReporter
          .at(targetDecl)
          .error("Federated execution is not supported with target " + target + ".");
    }
    if (target.equals(Target.C) && GeneratorUtils.isHostWindows()) {
      messageReporter
          .at(targetDecl)
          .error("Federated LF programs with a C target are currently not supported on Windows.");
      targetOK = false;
    }

    return targetOK;
  }

  private Map<Path, CodeMap> compileFederates(
      LFGeneratorContext context,
      Map<Path, CodeMap> lf2lfCodeMapMap,
      Consumer<List<SubContext>> finalizer) {

    // FIXME: Use the appropriate resource set instead of always using standalone
    Injector inj = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
    XtextResourceSet rs = inj.getInstance(XtextResourceSet.class);
    rs.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    // define output path here
    JavaIoFileSystemAccess fsa = inj.getInstance(JavaIoFileSystemAccess.class);
    fsa.setOutputPath("DEFAULT_OUTPUT", fileConfig.getSrcGenPath().toString());

    var numOfCompileThreads =
        Math.min(
            6, Math.min(Math.max(federates.size(), 1), Runtime.getRuntime().availableProcessors()));
    var compileThreadPool = Executors.newFixedThreadPool(numOfCompileThreads);
    messageReporter
        .nowhere()
        .info("******** Using " + numOfCompileThreads + " threads to compile the program.");
    Map<Path, CodeMap> codeMapMap = new ConcurrentHashMap<>();
    List<SubContext> subContexts = Collections.synchronizedList(new ArrayList<>());
    Averager averager = new Averager(federates.size());
    final var threadSafeErrorReporter = new SynchronizedMessageReporter(messageReporter);
    for (int i = 0; i < federates.size(); i++) {
      FederateInstance fed = federates.get(i);
      final int id = i;
      compileThreadPool.execute(
          () -> {
            Resource res =
                rs.getResource(
                    URI.createFileURI(
                        FedEmitter.lfFilePath(fileConfig, fed).toAbsolutePath().toString()),
                    true);
            FileConfig subFileConfig =
                LFGenerator.createFileConfig(res, fileConfig.getSrcGenPath(), true);
            MessageReporter subContextMessageReporter =
                new LineAdjustingMessageReporter(threadSafeErrorReporter, lf2lfCodeMapMap);

            var props = new Properties();
            if (targetConfig.dockerOptions != null && targetConfig.target.buildsUsingDocker()) {
              props.put("no-compile", "true");
            }
            props.put("docker", "false");

            TargetConfig subConfig =
                new TargetConfig(
                    props,
                    GeneratorUtils.findTargetDecl(subFileConfig.resource),
                    subContextMessageReporter);
            SubContext subContext =
                new SubContext(context, IntegratedBuilder.VALIDATED_PERCENT_PROGRESS, 100) {
                  @Override
                  public MessageReporter getErrorReporter() {
                    return subContextMessageReporter;
                  }

                  @Override
                  public void reportProgress(String message, int percentage) {
                    averager.report(
                        id,
                        percentage,
                        meanPercentage -> super.reportProgress(message, meanPercentage));
                  }

                  @Override
                  public FileConfig getFileConfig() {
                    return subFileConfig;
                  }

                  @Override
                  public TargetConfig getTargetConfig() {
                    return subConfig;
                  }
                };

            inj.getInstance(LFGenerator.class).doGenerate(res, fsa, subContext);
            codeMapMap.putAll(subContext.getResult().getCodeMaps());
            subContexts.add(subContext);
          });
    }
    // Initiate an orderly shutdown in which previously submitted tasks are
    // executed, but no new tasks will be accepted.
    compileThreadPool.shutdown();

    // Wait for all compile threads to finish (NOTE: Can block forever)
    try {
      if (!compileThreadPool.awaitTermination(Long.MAX_VALUE, TimeUnit.NANOSECONDS)) {
        context.getErrorReporter().nowhere().error("Timed out while compiling.");
      }
    } catch (Exception e) {
      context
          .getErrorReporter()
          .nowhere()
          .error("Failure during code generation: " + e.getMessage());
      e.printStackTrace();
    } finally {
      finalizer.accept(subContexts);
    }
    return codeMapMap;
  }

  /**
   * Process command-line arguments passed on to the generator.
   *
   * @param context Context of the build process.
   */
  private void processCLIArguments(LFGeneratorContext context) {
    if (context.getArgs().containsKey("rti")) {
      setFederationRTIProperties(context);
    }
  }

  /**
   * Set the RTI hostname, port and username if given as compiler arguments
   *
   * @param context Context of the build process.
   */
  private void setFederationRTIProperties(LFGeneratorContext context) {
    String rtiAddr = context.getArgs().getProperty("rti");
    Pattern pattern =
        Pattern.compile(
            "([a-zA-Z\\d]+@)?([a-zA-Z\\d]+\\.?[a-z]{2,}|\\d+\\.\\d+\\.\\d+\\.\\d+):?(\\d+)?");
    Matcher matcher = pattern.matcher(rtiAddr);

    if (!matcher.find()) {
      return;
    }

    // the user match group contains a trailing "@" which needs to be removed.
    String userWithAt = matcher.group(1);
    String user = (userWithAt == null) ? null : userWithAt.substring(0, userWithAt.length() - 1);
    String host = matcher.group(2);
    String port = matcher.group(3);

    if (host != null) {
      rtiConfig.setHost(host);
    }
    if (port != null) {
      rtiConfig.setPort(Integer.parseInt(port));
    }
    if (user != null) {
      rtiConfig.setUser(user);
    }
  }

  /**
   * Analyze the federation and record various properties of it.
   *
   * @param federation The federated reactor that contains all federates' instances.
   */
  private void analyzeFederates(Reactor federation, LFGeneratorContext context) {
    // Create an instantiation for the fed reactor because there isn't one.
    // Creating a definition for the main reactor because there isn't one.
    mainDef = LfFactory.eINSTANCE.createInstantiation();
    mainDef.setName(federation.getName());
    mainDef.setReactorClass(federation);

    // Make sure that if no federation RTI properties were given in the
    // cmdline, then those specified in the lf file are not lost
    if (rtiConfig.getHost().equals("localhost")
        && federation.getHost() != null
        && !federation.getHost().getAddr().equals("localhost")) {
      rtiConfig.setHost(federation.getHost().getAddr());
    }

    // Since federates are always within the main (federated) reactor,
    // create a list containing just that one containing instantiation.
    // This will be used to look up parameter values.
    List<Instantiation> mainReactorContext = new ArrayList<>();
    mainReactorContext.add(mainDef);

    // Create a FederateInstance for each instance in the top-level reactor.
    for (Instantiation instantiation : ASTUtils.allInstantiations(federation)) {
      int bankWidth = ASTUtils.width(instantiation.getWidthSpec(), mainReactorContext);
      if (bankWidth < 0) {
        messageReporter
            .at(instantiation)
            .error("Cannot determine bank width! Assuming width of 1.");
        // Continue with a bank width of 1.
        bankWidth = 1;
      }
      List<FederateInstance> federateInstances =
          getFederateInstances(instantiation, bankWidth, context);
      if (federatesByInstantiation == null) {
        federatesByInstantiation = new LinkedHashMap<>();
      }
      federatesByInstantiation.put(instantiation, federateInstances);
    }
  }

  /**
   * Get federate instances for a given {@code instantiation}. A bank will result in the creation of
   * multiple federate instances (one for each member of the bank).
   *
   * @param instantiation An instantiation that corresponds to a federate.
   * @param bankWidth The width specified for the instantiation.
   * @return A list of federate instance (of type @see FederateInstance).
   */
  private List<FederateInstance> getFederateInstances(
      Instantiation instantiation, int bankWidth, LFGeneratorContext context) {
    // Create one federate instance for each instance in a bank of reactors.
    List<FederateInstance> federateInstances = new ArrayList<>(bankWidth);

    for (int i = 0; i < bankWidth; i++) {
      // Assign an integer ID to the federate.
      int federateID = federates.size();
      var resource = instantiation.getReactorClass().eResource();
      var federateTargetConfig = new FedTargetConfig(context, resource);
      FederateInstance federateInstance =
          new FederateInstance(
              instantiation, federateID, i, bankWidth, federateTargetConfig, messageReporter);
      federates.add(federateInstance);
      federateInstances.add(federateInstance);

      if (instantiation.getHost() != null) {
        federateInstance.host = instantiation.getHost().getAddr();
        // The following could be 0.
        federateInstance.port = instantiation.getHost().getPort();
        // The following could be null.
        federateInstance.user = instantiation.getHost().getUser();
        /* FIXME: The at keyword should support a directory component.
         * federateInstance.dir = instantiation.getHost().dir
         */
        if (federateInstance.host != null
            && !federateInstance.host.equals("localhost")
            && !federateInstance.host.equals("0.0.0.0")) {
          federateInstance.isRemote = true;
        }
      }
    }
    return federateInstances;
  }

  /**
   * Replace connections between federates in the AST with proxies that handle sending and receiving
   * data.
   *
   * @param federation Reactor class of the federation.
   * @param resource The file system resource from which the original program is derived.
   */
  private void replaceFederateConnectionsWithProxies(Reactor federation, Resource resource) {
    // Each connection in the AST may represent more than one connection between
    // federation instances because of banks and multiports. We need to generate communication
    // for each of these. To do this, we create a ReactorInstance so that we don't have
    // to duplicate the rather complicated logic in that class. We specify a depth of 1,
    // so it only creates the reactors immediately within the top level, not reactors
    // that those contain.
    ReactorInstance mainInstance = new ReactorInstance(federation, messageReporter);

    new ReactionInstanceGraph(mainInstance); // Constructor has side effects; its result is ignored

    insertIndexers(mainInstance, resource);

    for (ReactorInstance child : mainInstance.children) {
      for (PortInstance output : child.outputs) {
        replaceConnectionFromOutputPort(output, resource);
      }
    }

    // Remove the connections at the top level
    federation.getConnections().clear();

    // There will be AST transformations that invalidate some info
    // cached in ReactorInstance. FIXME: most likely not needed anymore
    mainInstance.clearCaches(false);
  }

  /**
   * Insert reactors that split a multiport into many ports. This is necessary in order to index
   * into specific entries of a multiport.
   */
  private void insertIndexers(ReactorInstance mainInstance, Resource resource) {
    for (ReactorInstance child : mainInstance.children) {
      for (PortInstance input : child.inputs) {
        var indexer = indexer(child, input, resource);
        var count = 0;
        for (FederateInstance federate : federatesByInstantiation.get(child.getDefinition())) {
          federate.networkReactors.add(indexer);
          var outerConnection = LfFactory.eINSTANCE.createConnection();
          var instantiation = LfFactory.eINSTANCE.createInstantiation();
          instantiation.setReactorClass(indexer);
          instantiation.setName(indexer.getName() + count++);
          federate.networkPortToIndexer.put(input, instantiation);
          federate.networkHelperInstantiations.add(instantiation);
          outerConnection.getLeftPorts().add(varRefOf(instantiation, "port"));
          outerConnection.getRightPorts().add(varRefOf(child.getDefinition(), input.getName()));
          federate.networkConnections.add(outerConnection);
        }
      }
    }
  }

  /**
   * Add an {@code indexer} to the model and return it. An indexer is a reactor that is an adapter
   * from many ports to just one port
   */
  private Reactor indexer(ReactorInstance reactorInstance, PortInstance input, Resource resource) {
    var indexer =
        FedASTUtils.addReactorDefinition(
            "_" + reactorInstance.getName() + input.getName(), resource);
    var output = LfFactory.eINSTANCE.createOutput();
    output.setWidthSpec(EcoreUtil.copy(input.getDefinition().getWidthSpec()));
    output.setType(EcoreUtil.copy(input.getDefinition().getType()));
    output.setName("port");
    indexer.getOutputs().add(output);
    for (int i = 0; i < (input.isMultiport() ? input.getWidth() : 1); i++) {
      var splitInput = LfFactory.eINSTANCE.createInput();
      splitInput.setName("port" + i);
      splitInput.setType(EcoreUtil.copy(input.getDefinition().getType()));
      indexer.getInputs().add(splitInput);
    }
    var innerConnection = LfFactory.eINSTANCE.createConnection();
    indexer.getInputs().stream()
        .map(Input::getName)
        .map(it -> FedGenerator.varRefOf(null, it))
        .forEach(innerConnection.getLeftPorts()::add);
    innerConnection.getRightPorts().add(varRefOf(null, output.getName()));
    indexer.getConnections().add(innerConnection);
    return indexer;
  }

  /** Return a {@code VarRef} with the given name. */
  private static VarRef varRefOf(Instantiation container, String name) {
    var varRef = LfFactory.eINSTANCE.createVarRef();
    var variable = LfFactory.eINSTANCE.createVariable();
    variable.setName(name);
    varRef.setVariable(variable);
    varRef.setContainer(container);
    return varRef;
  }

  /**
   * Replace the connections from the specified output port.
   *
   * @param output The output port instance.
   * @param resource The file system resource from which the original program is derived.
   */
  private void replaceConnectionFromOutputPort(PortInstance output, Resource resource) {
    // Iterate through ranges of the output port
    for (SendRange srcRange : output.getDependentPorts()) {
      if (srcRange.connection == null) {
        // This should not happen.
        messageReporter.at(output.getDefinition()).error("Cannot find output connection for port");
        continue;
      }
      // Iterate through destinations
      for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
        replaceOneToManyConnection(srcRange, dstRange, resource);
      }
    }
  }

  /**
   * Replace (potentially multiple) connection(s) that originate from an output port to multiple
   * destinations.
   *
   * @param srcRange A range of an output port that sources data for this connection.
   * @param dstRange A range of input ports that receive the data.
   * @param resource The file system resource from which the original program is derived.
   */
  private void replaceOneToManyConnection(
      SendRange srcRange, RuntimeRange<PortInstance> dstRange, Resource resource) {
    MixedRadixInt srcID = srcRange.startMR();
    MixedRadixInt dstID = dstRange.startMR();
    int dstCount = 0;
    int srcCount = 0;

    while (dstCount++ < dstRange.width) {
      int srcChannel = srcID.getDigits().get(0);
      int srcBank = srcID.get(1);
      int dstChannel = dstID.getDigits().get(0);
      int dstBank = dstID.get(1);

      FederateInstance srcFederate =
          federatesByInstantiation.get(srcRange.instance.getParent().getDefinition()).get(srcBank);
      FederateInstance dstFederate =
          federatesByInstantiation.get(dstRange.instance.getParent().getDefinition()).get(dstBank);

      // Clear banks
      srcFederate.instantiation.setWidthSpec(null);
      dstFederate.instantiation.setWidthSpec(null);

      FedConnectionInstance fedConnection =
          new FedConnectionInstance(
              srcRange,
              dstRange,
              srcChannel,
              srcBank,
              dstChannel,
              dstBank,
              srcFederate,
              dstFederate,
              FedUtils.getSerializer(srcRange.connection, srcFederate, dstFederate));

      replaceFedConnection(fedConnection, resource);

      dstID.increment();
      srcID.increment();
      srcCount++;
      if (srcCount == srcRange.width) {
        srcID = srcRange.startMR(); // Multicast. Start over.
      }
    }
  }

  /**
   * Replace a one-to-one federated connection with proxies.
   *
   * @param connection A connection between two federates.
   * @param resource The file system resource from which the original program is derived.
   */
  private void replaceFedConnection(FedConnectionInstance connection, Resource resource) {
    if (!connection.getDefinition().isPhysical()
        && targetConfig.coordination != CoordinationType.DECENTRALIZED) {
      // Map the delays on connections between federates.
      Set<Expression> dependsOnDelays =
          connection.dstFederate.dependsOn.computeIfAbsent(
              connection.srcFederate, k -> new LinkedHashSet<>());
      // Put the delay on the cache.
      if (connection.getDefinition().getDelay() != null) {
        dependsOnDelays.add(connection.getDefinition().getDelay());
      } else {
        // To indicate that at least one connection has no delay, add a null entry.
        dependsOnDelays.add(null);
      }
      // Map the connections between federates.
      Set<Expression> sendsToDelays =
          connection.srcFederate.sendsTo.computeIfAbsent(
              connection.dstFederate, k -> new LinkedHashSet<>());
      if (connection.getDefinition().getDelay() != null) {
        sendsToDelays.add(connection.getDefinition().getDelay());
      } else {
        // To indicate that at least one connection has no delay, add a null entry.
        sendsToDelays.add(null);
      }
    }

    FedASTUtils.makeCommunication(connection, resource, targetConfig.coordination, messageReporter);
  }
}
