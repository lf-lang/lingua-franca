package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure1;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.launcher.FedLauncher;
import org.lflang.federated.launcher.FedLauncherFactory;
import org.lflang.generator.CodeMap;
import org.lflang.generator.GeneratorResult.Status;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.MixedRadixInt;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.SubContext;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;

import com.google.inject.Injector;

public class FedGenerator {

    /** Average asynchronously reported numbers and do something with them. */
    private static class Averager {
        private final int n;
        private final int[] sum;

        /** Create an averager of reports from {@code n} processes. */
        public Averager(int n) {
            this.n = n;
            sum = new int[n];
        }

        /**
         * Receive {@code x} from process {@code id} and invoke {@code callback}
         * on the mean of the numbers most recently reported by the processes.
         */
        public synchronized void report(int id, int x, Procedure1<Integer> callback) {
            assert 0 < id && id < n;
            sum[id] = x;
            callback.apply(Arrays.stream(sum).sum() / n);
        }
    }

    private final FedFileConfig fileConfig;
    private final ErrorReporter errorReporter;
    /**
     * The current target configuration.
     */
    private final TargetConfig targetConfig = new TargetConfig();
    /**
     * A list of federate instances.
     */
    private final List<FederateInstance> federates = new ArrayList<>();
    /**
     * A map from federate IDs to federate instances.
     */
    private final Map<Integer, FederateInstance> federateByID = new LinkedHashMap<>();
    /**
     * The federation RTI properties, which defaults to 'localhost: 15045'.
     */
    final LinkedHashMap<String, Object> federationRTIProperties = CollectionLiterals.newLinkedHashMap(
        Pair.of("host", "localhost"),
        Pair.of("port", 0) // Indicator to use the default port, typically 15045.
    );
    /**
     * A map from instantiations to the federate instances for that
     * instantiation.
     * If the instantiation has a width, there may be more than one federate
     * instance.
     */
    private Map<Instantiation, List<FederateInstance>> federatesByInstantiation;

    /**
     * Definition of the main (top-level) reactor.
     * This is an automatically generated AST node for the top-level
     * reactor.
     */
    private Instantiation mainDef;

    public FedGenerator(FedFileConfig fileConfig, ErrorReporter errorReporter) {
        this.fileConfig = fileConfig;
        this.errorReporter = errorReporter;
    }

    public boolean doGenerate(Resource resource, LFGeneratorContext context) throws IOException {
        initializeTargetConfig(context);


        // In a federated execution, we need keepalive to be true,
        // otherwise a federate could exit simply because it hasn't received
        // any messages.
        targetConfig.keepalive = true;

        // Process command-line arguments
        processCLIArguments(context);

        // Find the federated reactor
        Reactor fedReactor = FedASTUtils.findFederatedReactor(resource);

        // Extract some useful information about the federation
        analyzeFederates(fedReactor);

        // Find all the connections between federates.
        // For each connection between federates, replace it in the
        // AST with an action (which inherits the delay) and four reactions.
        // The action will be physical for physical connections and logical
        // for logical connections.
        replaceFederateConnectionsWithProxies(fedReactor);

        createLauncher(fileConfig, errorReporter, federationRTIProperties);

        FedEmitter fedEmitter = new FedEmitter(
            fileConfig,
            ASTUtils.toDefinition(mainDef.getReactorClass()),
            errorReporter,
            federationRTIProperties
        );
        // Generate code for each federate
        Map<Path, CodeMap> lf2lfCodeMapMap = new HashMap<>();
        for (FederateInstance federate : federates) {
            lf2lfCodeMapMap.putAll(fedEmitter.generateFederate(
                context, federate, federates.size()
            ));
        }

        Map<Path, CodeMap> codeMapMap = compileFederates(context, lf2lfCodeMapMap);
        context.finish(Status.COMPILED, fileConfig.name, fileConfig, codeMapMap);
        return false;
    }

    /**
     * Create a launcher for the federation.
     * @param fileConfig
     * @param errorReporter
     * @param federationRTIProperties
     */
    public void createLauncher(
        FedFileConfig fileConfig,
        ErrorReporter errorReporter,
        LinkedHashMap<String, Object> federationRTIProperties
    ) {
        FedLauncher launcher;
        if (federates.size() == 0) {
            // no federates, use target properties of main file
            TargetDecl targetDecl = GeneratorUtils.findTarget(fileConfig.resource);
            launcher = FedLauncherFactory.getLauncher(Target.fromDecl(targetDecl),
                                                      targetConfig,
                                                      fileConfig,
                                                      errorReporter);
        } else {
            launcher = FedLauncherFactory.getLauncher(
                federates.get(0), // FIXME: This architecture only works for one target.
                fileConfig,
                errorReporter
            );
        }
        try {
            launcher.createLauncher(
                federates,
                federationRTIProperties
            );
        } catch (IOException e) {
            errorReporter.reportError(e.getMessage());
        }

        // System.out.println(PythonInfoGenerator.generateFedRunInfo(fileConfig));
    }

    private Map<Path, CodeMap> compileFederates(LFGeneratorContext context, Map<Path, CodeMap> lf2lfCodeMapMap) {
        // FIXME: Use the appropriate resource set instead of always using standalone
        Injector inj = new LFStandaloneSetup()
            .createInjectorAndDoEMFRegistration();
        XtextResourceSet rs = inj.getInstance(XtextResourceSet.class);
        rs.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
        LFGenerator gen = inj.getInstance(LFGenerator.class);
        // define output path here
        JavaIoFileSystemAccess fsa = inj.getInstance(JavaIoFileSystemAccess.class);
        fsa.setOutputPath("DEFAULT_OUTPUT", fileConfig.getFedSrcGenPath().toString());

        var numOfCompileThreads = Math.min(6,
                                           Math.min(
                                               Math.max(federates.size(), 1),
                                               Runtime.getRuntime().availableProcessors()
                                           )
        );
        var compileThreadPool = Executors.newFixedThreadPool(numOfCompileThreads);
        System.out.println("******** Using "+numOfCompileThreads+" threads to compile the program.");
        Map<Path, CodeMap> codeMapMap = new HashMap<>();

        Averager averager = new Averager(federates.size());
        for (int i = 0; i < federates.size(); i++) {
            FederateInstance fed = federates.get(i);
            final int id = i;
            compileThreadPool.execute(() -> {
                SubContext cont = new SubContext(context, 0, 0) {
                    @Override
                    public ErrorReporter constructErrorReporter(FileConfig fileConfig) {
                        return new LineAdjustingErrorReporter(errorReporter, lf2lfCodeMapMap);
                    }

                    @Override
                    public void reportProgress(String message, int percentage) {
                        averager.report(id, percentage, meanPercentage -> super.reportProgress(message, meanPercentage));
                    }
                };
                Resource res = rs.getResource(URI.createFileURI(
                    fileConfig.getFedSrcPath().resolve(fed.name + ".lf").toAbsolutePath().toString()
                ), true);
                gen.doGenerate(res, fsa, cont);
                codeMapMap.putAll(cont.getResult().getCodeMaps());
            });
        }

        // Initiate an orderly shutdown in which previously submitted tasks are
        // executed, but no new tasks will be accepted.
        compileThreadPool.shutdown();

        // Wait for all compile threads to finish (NOTE: Can block forever)
        try {
            compileThreadPool.awaitTermination(Long.MAX_VALUE, TimeUnit.NANOSECONDS);
        } catch (Exception e) {
            Exceptions.sneakyThrow(e);
        }
        return codeMapMap;
    }

    /**
     * Initialize the target config.
     * @param context
     * @throws IOException
     */
    private void initializeTargetConfig(LFGeneratorContext context) throws IOException {
        GeneratorUtils.setTargetConfig(
            context,
            GeneratorUtils.findTarget(fileConfig.resource),
            targetConfig,
            errorReporter
        );
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
        Pattern pattern = Pattern.compile("([a-zA-Z0-9]+@)?([a-zA-Z0-9]+\\.?[a-z]{2,}|[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+):?([0-9]+)?");
        Matcher matcher = pattern.matcher(rtiAddr);

        if (!matcher.find()) {
            return;
        }

        // the user match group contains a trailing "@" which needs to be removed.
        String userWithAt = matcher.group(1);
        String user = (userWithAt == null) ? null : userWithAt.substring(0,
                                                                       userWithAt.length()
                                                                           - 1);
        String host = matcher.group(2);
        String port = matcher.group(3);

        if (host != null) {
            federationRTIProperties.put("host", host);
        }
        if (port != null) {
            federationRTIProperties.put("port", port);
        }
        if (user != null) {
            federationRTIProperties.put("user", user);
        }
    }

    /**
     * Analyze the federation and record various properties of it.
     *
     * @param fedReactor The federated reactor that contains all federates' instances.
     */
    private void analyzeFederates(Reactor fedReactor) {
        // Create an instantiation for the fed reactor because there isn't one.
        // Creating a definition for the main reactor because there isn't one.
        mainDef = LfFactory.eINSTANCE.createInstantiation();
        mainDef.setName(fedReactor.getName());
        mainDef.setReactorClass(fedReactor);

        // Since federates are always within the main (federated) reactor,
        // create a list containing just that one containing instantiation.
        // This will be used to look up parameter values.
        List<Instantiation> mainReactorContext = new ArrayList<>();
        mainReactorContext.add(mainDef);

        // Create a FederateInstance for each top-level reactor.
        for (Instantiation instantiation : ASTUtils.allInstantiations(fedReactor)) {
            int bankWidth = ASTUtils.width(instantiation.getWidthSpec(), mainReactorContext);
            if (bankWidth < 0) {
                errorReporter.reportError(instantiation, "Cannot determine bank width! Assuming width of 1.");
                // Continue with a bank width of 1.
                bankWidth = 1;
            }
            List<FederateInstance> federateInstances = getFederateInstances(instantiation, bankWidth);
            if (federatesByInstantiation == null) {
                federatesByInstantiation = new LinkedHashMap<>();
            }
            federatesByInstantiation.put(instantiation, federateInstances);
        }
    }

    /**
     * Get federate instances for a given {@code instantiation}. A bank will
     * result in the creation of multiple federate instances (one for each
     * member of the bank).
     *
     * @param instantiation An instantiation that corresponds to a federate.
     * @param bankWidth     The width specified for the instantiation.
     * @return A list of federate instance (of type @see FederateInstance).
     */
    private List<FederateInstance> getFederateInstances(Instantiation instantiation, int bankWidth) {
        // Create one federate instance for each instance in a bank of reactors.
        List<FederateInstance> federateInstances = new ArrayList<>(bankWidth);
        for (int i = 0; i < bankWidth; i++) {
            // Assign an integer ID to the federate.
            int federateID = federates.size();
            FederateInstance federateInstance = new FederateInstance(instantiation, federateID, i, errorReporter);
            federateInstance.bankIndex = i;
            federates.add(federateInstance);
            federateInstances.add(federateInstance);
            federateByID.put(federateID, federateInstance);

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
     * Replace connections between federates in the AST with proxies that
     * handle sending and receiving data.
     *
     * @param fedReactor
     */
    private void replaceFederateConnectionsWithProxies(Reactor fedReactor) {
        // Each connection in the AST may represent more than one connection between
        // federate instances because of banks and multiports. We need to generate communication
        // for each of these. To do this, we create a ReactorInstance so that we don't have
        // to duplicate the rather complicated logic in that class. We specify a depth of 1,
        // so it only creates the reactors immediately within the top level, not reactors
        // that those contain.
        ReactorInstance mainInstance = new ReactorInstance(fedReactor, errorReporter);

        for (ReactorInstance child : mainInstance.children) {
            for (PortInstance output : child.outputs) {
                replaceConnectionFromOutputPort(output);
            }
        }

        // Remove the connections at the top level
        fedReactor.getConnections().clear();

        // There will be AST transformations that invalidate some info
        // cached in ReactorInstance. FIXME: most likely not needed anymore
        mainInstance.clearCaches(false);
    }

    /**
     * Replace the connections from the specified output port.
     *
     * @param output The output port instance.
     */
    private void replaceConnectionFromOutputPort(PortInstance output) {
        // Iterate through ranges of the output port
        for (SendRange srcRange : output.getDependentPorts()) {
            if (srcRange.connection == null) {
                // This should not happen.
                errorReporter.reportError(
                    output.getDefinition(),
                    "Unexpected error. Cannot find output connection for port"
                );
                continue;
            }
            // Iterate through destinations
            for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                replaceOneToManyConnection(
                    srcRange,
                    dstRange
                );
            }
        }
    }

    /**
     * Replace (potentially multiple) connection(s) that originate from an
     * output port to multiple destinations.
     *
     * @param srcRange A range of an output port that sources data for this
     *                 connection.
     * @param dstRange A range of input ports that receive the data.
     */
    private void replaceOneToManyConnection(
        SendRange srcRange,
        RuntimeRange<PortInstance> dstRange
    ) {
        MixedRadixInt srcID = srcRange.startMR();
        MixedRadixInt dstID = dstRange.startMR();
        int dstCount = 0;
        int srcCount = 0;

        while (dstCount++ < dstRange.width) {
            int srcChannel = srcID.getDigits().get(0);
            int srcBank = srcID.get(1);
            int dstChannel = dstID.getDigits().get(0);
            int dstBank = dstID.get(1);

            FederateInstance srcFederate = federatesByInstantiation.get(
                srcRange.instance.getParent().getDefinition()
            ).get(srcBank);
            FederateInstance dstFederate = federatesByInstantiation.get(
                dstRange.instance.getParent().getDefinition()
            ).get(dstBank);

            // Clear banks
            srcFederate.instantiation.setWidthSpec(null);
            dstFederate.instantiation.setWidthSpec(null);

            FedConnectionInstance fedConnection = new FedConnectionInstance(
                srcRange,
                dstRange,
                srcChannel,
                srcBank,
                dstChannel,
                dstBank,
                srcFederate,
                dstFederate,
                FedUtils.getSerializer(srcRange.connection, srcFederate, dstFederate)
            );

            replaceFedConnection(fedConnection);

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
     */
    private void replaceFedConnection(FedConnectionInstance connection) {
        if (!connection.getDefinition().isPhysical()
            && targetConfig.coordination != CoordinationType.DECENTRALIZED) {
            // Map the delays on connections between federates.
            Set<Expression> dependsOnDelays =
                connection.dstFederate.dependsOn.computeIfAbsent(
                    connection.srcFederate,
                    k -> new LinkedHashSet<>()
                );
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
                    connection.dstFederate,
                    k -> new LinkedHashSet<>()
                );
            if (connection.getDefinition().getDelay() != null) {
                sendsToDelays.add(connection.getDefinition().getDelay());
            } else {
                // To indicate that at least one connection has no delay, add a null entry.
                sendsToDelays.add(null);
            }
        }

        FedASTUtils.makeCommunication(connection, targetConfig.coordination, errorReporter);
    }
}
