package org.lflang.federated.generator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.MixedRadixInt;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;

public class FedGenerator {

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

        FedEmitter fedEmitter = new FedEmitter(
            fileConfig,
            ASTUtils.toDefinition(mainDef.getReactorClass()),
            errorReporter,
            federationRTIProperties
        );
        // Generate code for each federate
        for (FederateInstance federate : federates) {
            fedEmitter.generateFederate(
                federate
            );
        }

        compileFederate();

        return false;
    }

    private void compileFederate() {
        var numOfCompileThreads = Math.min(6,
                                           Math.min(
                                               Math.max(federates.size(), 1),
                                               Runtime.getRuntime().availableProcessors()
                                           )
        );
        var compileThreadPool = Executors.newFixedThreadPool(numOfCompileThreads);
        System.out.println("******** Using "+numOfCompileThreads+" threads to compile the program.");
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
        // get and parse the target declaration (once)
        TargetDecl target = GeneratorUtils.findTarget(fileConfig.resource);
        TargetConfig fedTargetConfig = new TargetConfig();
        List<KeyValuePair> pairs = target.getConfig().getPairs();
        TargetProperty.set(fedTargetConfig, pairs != null ? pairs : List.of(), errorReporter);

        // Create one federate instance for each instance in a bank of reactors.
        List<FederateInstance> federateInstances = new ArrayList<>(bankWidth);
        for (int i = 0; i < bankWidth; i++) {
            // Assign an integer ID to the federate.
            int federateID = federates.size();
            FederateInstance federateInstance = new FederateInstance(instantiation, federateID, i, errorReporter);
            federateInstance.bankIndex = i;
            federateInstance.target = target;
            federateInstance.targetConfig = targetConfig;
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
        ReactorInstance mainInstance = new ReactorInstance(fedReactor, errorReporter, 1);

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
