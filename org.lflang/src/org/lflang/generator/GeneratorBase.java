/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

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

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import org.eclipse.core.resources.IMarker;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.MainConflictChecker;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.federated.FedASTUtils;
import org.lflang.federated.FederateInstance;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.graph.InstantiationGraph;
import org.lflang.lf.Action;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;
import org.lflang.validation.AbstractLFValidator;

import com.google.common.base.Objects;
import com.google.common.collect.Iterables;

/**
 * Generator base class for specifying core functionality
 * that all code generators should have.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
public abstract class GeneratorBase extends AbstractLFValidator {

    ////////////////////////////////////////////
    //// Public fields.

    /**
     * Constant that specifies how to name generated delay reactors.
     */
    public static String GEN_DELAY_CLASS_NAME = "_lf_GenDelay";

    /**
     * The main (top-level) reactor instance.
     */
    public ReactorInstance main;

    /** A error reporter for reporting any errors or warnings during the code generation */
    public ErrorReporter errorReporter;

    ////////////////////////////////////////////
    //// Protected fields.

    /**
     * The current target configuration.
     */
    protected TargetConfig targetConfig = new TargetConfig();

    public TargetConfig getTargetConfig() { return this.targetConfig;}
    
    /**
     * The current file configuration.
     */
    protected FileConfig fileConfig;
    
    /**
     * A factory for compiler commands.
     */
    protected GeneratorCommandFactory commandFactory;

    public GeneratorCommandFactory getCommandFactory() { return commandFactory; }

    /**
     * Collection of generated delay classes.
     */
    private LinkedHashSet<Reactor> delayClasses = new LinkedHashSet<>();

    /**
     * Definition of the main (top-level) reactor.
     * This is an automatically generated AST node for the top-level
     * reactor.
     */
    protected Instantiation mainDef;
    public Instantiation getMainDef() { return mainDef; }

    /**
     * A list of Reactor definitions in the main resource, including non-main 
     * reactors defined in imported resources. These are ordered in the list in
     * such a way that each reactor is preceded by any reactor that it instantiates
     * using a command like `foo = new Foo();`
     */
    protected List<Reactor> reactors = new ArrayList<>();
    
    /**
     * The set of resources referenced reactor classes reside in.
     */
    protected Set<LFResource> resources = new LinkedHashSet<>();
    
    /**
     * Graph that tracks dependencies between instantiations. 
     * This is a graph where each node is a Reactor (not a ReactorInstance)
     * and an arc from Reactor A to Reactor B means that B contains an instance of A, constructed with a statement
     * like `a = new A();`  After creating the graph, 
     * sort the reactors in topological order and assign them to the reactors class variable. 
     * Hence, after this method returns, `this.reactors` will be a list of Reactors such that any
     * reactor is preceded in the list by reactors that it instantiates.
     */
    protected InstantiationGraph instantiationGraph;

    /**
     * The set of unordered reactions. An unordered reaction is one that does
     * not have any dependency on other reactions in the containing reactor, 
     * and where no other reaction in the containing reactor depends on it.
     * There is currently no way in the syntax of LF to make a reaction
     * unordered, deliberately, because it can introduce unexpected
     * nondeterminacy. However, certain automatically generated reactions are
     * known to be safe to be unordered because they do not interact with the
     * state of the containing reactor. To make a reaction unordered, when
     * the Reaction instance is created, add that instance to this set.
     */
    protected Set<Reaction> unorderedReactions = null;

    /**
     * Map from reactions to bank indices
     */
    protected Map<Reaction,Integer> reactionBankIndices = null;

    /**
     * Keep a unique list of enabled serializers
     */
    public HashSet<SupportedSerializers> enabledSerializers = new HashSet<>();

    /**
     * Indicates whether or not the current Lingua Franca program
     * contains a federation.
     */
    public boolean isFederated = false;

    /**
     * Indicates whether or not the current Lingua Franca program
     * contains model reactors.
     */
    public boolean hasModalReactors = false;

    // //////////////////////////////////////////
    // // Target properties, if they are included.
    /**
     * A list of federate instances or a list with a single empty string
     * if there are no federates specified. FIXME: Why put a single empty string there? It should be just empty...
     */
    public List<FederateInstance> federates = new ArrayList<>();

    /**
     * A map from federate IDs to federate instances.
     */
    protected Map<Integer, FederateInstance> federateByID = new LinkedHashMap<>();

    /**
     * A map from instantiations to the federate instances for that instantiation.
     * If the instantiation has a width, there may be more than one federate instance.
     */
    protected Map<Instantiation, List<FederateInstance>> federatesByInstantiation;

    /**
     * The federation RTI properties, which defaults to 'localhost: 15045'.
     */
    protected LinkedHashMap<String, Object> federationRTIProperties = CollectionLiterals.newLinkedHashMap(
        Pair.of("host", "localhost"),
        Pair.of("port", 0) // Indicator to use the default port, typically 15045.
    );

    /**
     * Contents of $LF_CLASSPATH, if it was set.
     */
    protected String classpathLF;

    /**
     * The name of the top-level reactor.
     */
    protected String topLevelName; // FIXME: remove and use fileConfig.name instead

    // //////////////////////////////////////////
    // // Private fields.
    
    /**
     * Create a new GeneratorBase object.
     */
    public GeneratorBase(FileConfig fileConfig, ErrorReporter errorReporter) {
        this.fileConfig = fileConfig;
        this.topLevelName = fileConfig.name;
        this.errorReporter = errorReporter;
        this.commandFactory = new GeneratorCommandFactory(errorReporter, fileConfig);
    }

    // //////////////////////////////////////////
    // // Code generation functions to override for a concrete code generator.

    /**
     * Store the given reactor in the collection of generated delay classes
     * and insert it in the AST under the top-level reactors node.
     */
    public void addDelayClass(Reactor generatedDelay) {
        // Record this class, so it can be reused.
        delayClasses.add(generatedDelay);
        // And hook it into the AST.
        EObject node = IteratorExtensions.findFirst(fileConfig.resource.getAllContents(), Model.class::isInstance);
        ((Model) node).getReactors().add(generatedDelay);
    }

    /**
     * Return the generated delay reactor that corresponds to the given class
     * name if it had been created already, `null` otherwise.
     */
    public Reactor findDelayClass(String className) {
        return IterableExtensions.findFirst(delayClasses, it -> it.getName().equals(className));
    }

    /**
     * If there is a main or federated reactor, then create a synthetic Instantiation
     * for that top-level reactor and set the field mainDef to refer to it.
     */
    private void createMainInstantiation() {
        // Find the main reactor and create an AST node for its instantiation.
        Iterable<EObject> nodes = IteratorExtensions.toIterable(fileConfig.resource.getAllContents());
        for (Reactor reactor : Iterables.filter(nodes, Reactor.class)) {
            if (reactor.isMain() || reactor.isFederated()) {
                // Creating an definition for the main reactor because there isn't one.
                this.mainDef = LfFactory.eINSTANCE.createInstantiation();
                this.mainDef.setName(reactor.getName());
                this.mainDef.setReactorClass(reactor);
            }
        }
    }

    /**
     * Generate code from the Lingua Franca model contained by the specified resource.
     * 
     * This is the main entry point for code generation. This base class finds all
     * reactor class definitions, including any reactors defined in imported .lf files
     * (except any main reactors in those imported files), and adds them to the 
     * {@link #GeneratorBase.reactors reactors} list. If errors occur during
     * generation, then a subsequent call to errorsOccurred() will return true.
     * @param resource The resource containing the source code.
     * @param context Context relating to invocation of the code generator.
     * In stand alone mode, this object is also used to relay CLI arguments.
     */
    public void doGenerate(Resource resource, LFGeneratorContext context) {
        
        GeneratorUtils.setTargetConfig(
            context, GeneratorUtils.findTarget(fileConfig.resource), targetConfig, errorReporter
        );

        cleanIfNeeded(context);

        printInfo(context.getMode());

        // Clear any IDE markers that may have been created by a previous build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        if (errorReporter instanceof EclipseErrorReporter) {
            ((EclipseErrorReporter) errorReporter).clearMarkers();
        }
        
        ASTUtils.setMainName(fileConfig.resource, fileConfig.name);
        
        createMainInstantiation();

        // Check if there are any conflicting main reactors elsewhere in the package.
        if (Objects.equal(context.getMode(), LFGeneratorContext.Mode.STANDALONE) && mainDef != null) {
            for (String conflict : new MainConflictChecker(fileConfig).conflicts) {
                errorReporter.reportError(this.mainDef.getReactorClass(), "Conflicting main reactor in " + conflict);
            }
        }

        // Configure the command factory
        commandFactory.setVerbose();
        if (Objects.equal(context.getMode(), LFGeneratorContext.Mode.STANDALONE) && context.getArgs().containsKey("quiet")) {
            commandFactory.setQuiet();
        }

        // This must be done before desugaring delays below.
        analyzeFederates(context);
        
        // Process target files. Copy each of them into the src-gen dir.
        // FIXME: Should we do this here? This doesn't make sense for federates the way it is
        // done here.
        copyUserFiles(this.targetConfig, this.fileConfig);

        // Collect reactors and create an instantiation graph. 
        // These are needed to figure out which resources we need
        // to validate, which happens in setResources().
        setReactorsAndInstantiationGraph(context.getMode());

        GeneratorUtils.validate(context, fileConfig, instantiationGraph, errorReporter);
        List<Resource> allResources = GeneratorUtils.getResources(reactors);
        resources.addAll(allResources.stream()  // FIXME: This filter reproduces the behavior of the method it replaces. But why must it be so complicated? Why are we worried about weird corner cases like this?
            .filter(it -> !Objects.equal(it, fileConfig.resource) || mainDef != null && it == mainDef.getReactorClass().eResource())
            .map(it -> GeneratorUtils.getLFResource(it, fileConfig.getSrcGenBasePath(), context, errorReporter))
            .collect(Collectors.toList())
        );
        GeneratorUtils.accommodatePhysicalActionsIfPresent(
            allResources, 
            getTarget().setsKeepAliveOptionAutomatically(), 
            targetConfig, 
            errorReporter
        );
        // FIXME: Should the GeneratorBase pull in `files` from imported
        // resources?

        // Reroute connections that have delays associated with them via 
        // generated delay reactors.
        transformDelays();
        
        // Transform connections that reside in mutually exclusive modes and are otherwise conflicting
        // This should be done before creating the instantiation graph
        transformConflictingConnectionsInModalReactors();

        // Invoke these functions a second time because transformations 
        // may have introduced new reactors!
        setReactorsAndInstantiationGraph(context.getMode());

        // Check for existence and support of modes
        hasModalReactors = IterableExtensions.exists(reactors, it -> !it.getModes().isEmpty());
        checkModalReactorSupport(false);
        generateStartupReactionsInModesIfNeeded();
        
        enableSupportForSerializationIfApplicable(context.getCancelIndicator());
    }

    /**
     * Check if a clean was requested from the standalone compiler and perform
     * the clean step.
     */
    protected void cleanIfNeeded(LFGeneratorContext context) {
        if (context.getArgs().containsKey("clean")) {
            try {
                fileConfig.doClean();
            } catch (IOException e) {
                System.err.println("WARNING: IO Error during clean");
            }
        }
    }

    /**
     * Create a new instantiation graph. This is a graph where each node is a Reactor (not a ReactorInstance)
     * and an arc from Reactor A to Reactor B means that B contains an instance of A, constructed with a statement
     * like `a = new A();`  After creating the graph, 
     * sort the reactors in topological order and assign them to the reactors class variable. 
     * Hence, after this method returns, `this.reactors` will be a list of Reactors such that any
     * reactor is preceded in the list by reactors that it instantiates.
     */
    protected void setReactorsAndInstantiationGraph(LFGeneratorContext.Mode mode) {
        // Build the instantiation graph . 
        instantiationGraph = new InstantiationGraph(fileConfig.resource, false);

        // Topologically sort the reactors such that all of a reactor's instantiation dependencies occur earlier in 
        // the sorted list of reactors. This helps the code generator output code in the correct order.
        // For example if `reactor Foo {bar = new Bar()}` then the definition of `Bar` has to be generated before
        // the definition of `Foo`.
        reactors = instantiationGraph.nodesInTopologicalOrder();

        // If there is no main reactor or if all reactors in the file need to be validated, then make sure the reactors
        // list includes even reactors that are not instantiated anywhere.
        if (mainDef == null || Objects.equal(mode, LFGeneratorContext.Mode.LSP_MEDIUM)) {
            Iterable<EObject> nodes = IteratorExtensions.toIterable(fileConfig.resource.getAllContents());
            for (Reactor r : IterableExtensions.filter(nodes, Reactor.class)) {
                if (!reactors.contains(r)) {
                    reactors.add(r);
                }
            }
        }
    }

    /**
     * For each involved resource, replace connections with delays with generated delay reactors.
     */
    private void transformDelays() {
        for (LFResource r : resources) {
            ASTUtils.insertGeneratedDelays(r.eResource, this);
        }
    }

    /**
     * Copy user specific files to the src-gen folder.
     *
     * This should be overridden by the target generators.
     *
     * @param targetConfig The targetConfig to read the `files` from.
     * @param fileConfig The fileConfig used to make the copy and resolve paths.
     */
    protected void copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {}

    /**
     * Return true if errors occurred in the last call to doGenerate().
     * This will return true if any of the reportError methods was called.
     * @return True if errors occurred.
     */
    public boolean errorsOccurred() {
        return errorReporter.getErrorsOccurred();
    }

    /*
     * Return the TargetTypes instance associated with this.
     */
    public abstract TargetTypes getTargetTypes();
    
    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param the action to schedule
     * @param the port to read from
     */
    public abstract String generateDelayBody(Action action, VarRef port);

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param the action that triggers the reaction
     * @param the port to write to
     */
    public abstract String generateForwardBody(Action action, VarRef port);

    /**
     * Generate code for the generic type to be used in the class definition
     * of a generated delay reactor.
     */
    public abstract String generateDelayGeneric();

    /**
     * Return true if the reaction is unordered. An unordered reaction is one
     * that does not have any dependency on other reactions in the containing
     * reactor, and where no other reaction in the containing reactor depends
     * on it. There is currently no way in the syntax of LF to make a reaction
     * unordered, deliberately, because it can introduce unexpected 
     * nondeterminacy. However, certain automatically generated reactions are
     * known to be safe to be unordered because they do not interact with the
     * state of the containing reactor. To make a reaction unordered, when
     * the Reaction instance is created, add that instance to this set.
     * @return True if the reaction has been marked unordered.
     */
    public boolean isUnordered(Reaction reaction) {
        return unorderedReactions != null ? unorderedReactions.contains(reaction) : false;
    }

    /**
     * Mark the reaction unordered. An unordered reaction is one that does not
     * have any dependency on other reactions in the containing reactor, and
     * where no other reaction in the containing reactor depends on it. There
     * is currently no way in the syntax of LF to make a reaction unordered,
     * deliberately, because it can introduce unexpected nondeterminacy. 
     * However, certain automatically generated reactions are known to be safe
     * to be unordered because they do not interact with the state of the 
     * containing reactor. To make a reaction unordered, when the Reaction
     * instance is created, add that instance to this set.
     * @param reaction The reaction to make unordered.
     */
    public void makeUnordered(Reaction reaction) {
        if (unorderedReactions == null) {
            unorderedReactions = new LinkedHashSet<Reaction>();
        }
        unorderedReactions.add(reaction);
    }

    /**
     * Mark the specified reaction to belong to only the specified
     * bank index. This is needed because reactions cannot declare
     * a specific bank index as an effect or trigger. Reactions that
     * send messages between federates, including absent messages,
     * need to be specific to a bank member.
     * @param The reaction.
     * @param bankIndex The bank index, or -1 if there is no bank.
     */
    public void setReactionBankIndex(Reaction reaction, int bankIndex) {
        if (bankIndex < 0) {
            return;
        }
        if (reactionBankIndices == null) {
            reactionBankIndices = new LinkedHashMap<Reaction,Integer>();
        }  
        reactionBankIndices.put(reaction, bankIndex);
    }

    /**
     * Return the reaction bank index.
     * @see setReactionBankIndex(Reaction reaction, int bankIndex)
     * @param The reaction.
     * @return The reaction bank index, if one has been set, and -1 otherwise.
     */
    public int getReactionBankIndex(Reaction reaction) {
        if (reactionBankIndices == null) return -1;
        if (reactionBankIndices.get(reaction) == null) return -1;
        return reactionBankIndices.get(reaction);
    }
    
    /**
     * Given a representation of time that may possibly include units, return
     * a string that the target language can recognize as a value. In this base
     * class, if units are given, e.g. "msec", then we convert the units to upper
     * case and return an expression of the form "MSEC(value)". Particular target
     * generators will need to either define functions or macros for each possible
     * time unit or override this method to return something acceptable to the
     * target language.
     * @param time A TimeValue that represents a time.
     * @return A string, such as "MSEC(100)" for 100 milliseconds.
     */
    public static String timeInTargetLanguage(TimeValue time) {
        if (time != null) {
            if (time.unit != null) {
                return cMacroName(time.unit) + "(" + time.getMagnitude() + ")";
            } else {
                return Long.valueOf(time.getMagnitude()).toString();
            }
        }
        return "0"; // FIXME: do this or throw exception?
    }

    // note that this is moved out by #544
    public static final String cMacroName(TimeUnit unit) {
        return unit.getCanonicalName().toUpperCase();
    }

    // //////////////////////////////////////////
    // // Protected methods.
    
    /**
     * Checks whether modal reactors are present and require appropriate code generation.
     * This will set the hasModalReactors variable.
     * @param isSupported indicates if modes are supported by this code generation.
     */
    protected void checkModalReactorSupport(boolean isSupported) {
        if (hasModalReactors && !isSupported) {
            errorReporter.reportError("The currently selected code generation or " +
                                      "target configuration does not support modal reactors!");
        }
    }
    
    /**
     * Finds and transforms connections into forwarding reactions iff the connections have the same destination as other
     * connections or reaction in mutually exclusive modes.
     */
    private void transformConflictingConnectionsInModalReactors() {
        for (LFResource r : resources) {
            var transform = ASTUtils.findConflictingConnectionsInModalReactors(r.eResource);
            if (!transform.isEmpty()) {
                var factory = LfFactory.eINSTANCE;
                for (Connection connection : transform) {
                    // Currently only simple transformations are supported
                    if (connection.isPhysical() || connection.getDelay() != null || connection.isIterated() || 
                        connection.getLeftPorts().size() > 1 || connection.getRightPorts().size() > 1
                    ) {
                        errorReporter.reportError(connection, "Cannot transform connection in modal reactor. Connection uses currently not supported features.");
                    } else {
                        var reaction = factory.createReaction();
                        ((Mode)connection.eContainer()).getReactions().add(reaction);
                        
                        var sourceRef = connection.getLeftPorts().get(0);
                        var destRef = connection.getRightPorts().get(0);
                        reaction.getTriggers().add(sourceRef);
                        reaction.getEffects().add(destRef);
                        
                        var code = factory.createCode();
                        var source = (sourceRef.getContainer() != null ?
                                sourceRef.getContainer().getName() + "." : "") + sourceRef.getVariable().getName();
                        var dest = (destRef.getContainer() != null ? 
                                destRef.getContainer().getName() + "." : "") + destRef.getVariable().getName();
                        code.setBody(getConflictingConnectionsInModalReactorsBody(source, dest));
                        reaction.setCode(code);
                        
                        EcoreUtil.remove(connection);
                    }
                }
            }
        }
    }
    /**
     * Return target code for forwarding reactions iff the connections have the
     * same destination as other connections or reaction in mutually exclusive modes.
     * 
     * This methods needs to be overridden in target specific code generators that
     * support modal reactors.
     */
    protected String getConflictingConnectionsInModalReactorsBody(String source, String dest) {
        errorReporter.reportError("The currently selected code generation " +
                                  "is missing an implementation for conflicting " +
                                  "transforming connections in modal reactors.");
        return "MODAL MODELS NOT SUPPORTED";
    }
    
    /**
     * Generate startup reactions in modes.
     * 
     * Startup reactions (reactions that have startup in their list of triggers)
     * will be triggered when the mode is entered for the first time and on each subsequent
     * reset transition to that mode. These reactions could be useful for targets
     * to perform cleanups, for example, to reset state variables.
     */
    protected void generateStartupReactionsInModesIfNeeded() {
        // Do nothing
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
    public String generateNetworkReceiverBody(
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
        throw new UnsupportedOperationException("This target does not support network connections between federates.");
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network. This base class throws an exception.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @throws UnsupportedOperationException If the target does not support this operation.
     * @param serializer The serializer used on the connection.
     */
    public String generateNetworkSenderBody(
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
        throw new UnsupportedOperationException("This target does not support network connections between federates.");
    }
    
    /**
     * Generate code for the body of a reaction that waits long enough so that the status
     * of the trigger for the given port becomes known for the current logical time.
     * 
     * @param port The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     */
    public String generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.");
    }    
    
    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     * 
     * @param port The port to generate the control reaction for
     * @param portID The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel if a multiport
     * @param delay The delay value imposed on the connection using after
     */
    public String generateNetworkOutputControlReactionBody(
        VarRef port,
        int portID,
        int receivingFederateID,
        int sendingBankIndex,
        int sendingChannelIndex,
        Delay delay
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.");
    }

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    public void enableSupportForSerializationIfApplicable(CancelIndicator cancelIndicator) {
        if (!IterableExtensions.isNullOrEmpty(enabledSerializers)) {
            throw new UnsupportedOperationException(
                "Serialization is target-specific "+
                " and is not implemented for the "+getTarget().toString()+" target."
            );
        }
    }
    
    /**
     * Returns true if the program is federated and uses the decentralized
     * coordination mechanism.
     */
    public boolean isFederatedAndDecentralized() {
        return isFederated && targetConfig.coordination == CoordinationType.DECENTRALIZED;
    }
    
    /**
     * Returns true if the program is federated and uses the centralized
     * coordination mechanism.
     */
    public boolean isFederatedAndCentralized() {
        return isFederated && targetConfig.coordination == CoordinationType.CENTRALIZED;
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    public void writeDockerFile(File dockerComposeDir, String dockerFileName, String federateName) throws IOException {
        throw new UnsupportedOperationException("This target does not support docker file generation.");
    }
    
    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    public String getDockerComposeCommand() {
        String OS = System.getProperty("os.name").toLowerCase();
        return (OS.indexOf("nux") >= 0) ? "docker-compose" : "docker compose";
    }
    
    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    public String getDockerBuildCommand(String dockerFile, File dockerComposeDir, String federateName) {
        return String.join("\n", 
            "Dockerfile for "+topLevelName+" written to "+dockerFile,
            "#####################################",
            "To build the docker image, go to "+dockerComposeDir+" and run:",
            "",
            "    "+getDockerComposeCommand()+" build "+federateName,
            "",
            "#####################################"
        );
    }

    /**
     * Parsed error message from a compiler is returned here.
     */
    public static class ErrorFileAndLine {
        public String filepath = null;
        public String line = "1";
        public String character = "0";
        public String message = "";
        public boolean isError = true; // false for a warning.
        
        @Override 
        public String toString() {
          return (isError ? "Error" : "Non-error") + " at " + line + ":" + character + " of file " + filepath + ": " + message;
        }
    }

    /**
     * Given a line of text from the output of a compiler, return
     * an instance of ErrorFileAndLine if the line is recognized as
     * the first line of an error message. Otherwise, return null.
     * This base class simply returns null.
     * @param line A line of output from a compiler or other external
     * tool that might generate errors.
     * @return If the line is recognized as the start of an error message,
     * then return a class containing the path to the file on which the
     * error occurred (or null if there is none), the line number (or the
     * string "1" if there is none), the character position (or the string
     * "0" if there is none), and the message (or an empty string if there
     * is none).
     */
    protected ErrorFileAndLine parseCommandOutput(String line) {
        return null;
    }

    /**
     * Parse the specified string for command errors that can be reported
     * using marks in the Eclipse IDE. In this class, we attempt to parse
     * the messages to look for file and line information, thereby generating
     * marks on the appropriate lines. This should not be called in standalone
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
        Path path = fileConfig.srcFile;
        // In case errors occur within an imported file, record the original path.
        Path originalPath = path;
        
        int severity = IMarker.SEVERITY_ERROR;
        for (String line : lines) {
            ErrorFileAndLine parsed = parseCommandOutput(line);
            if (parsed != null) {
                // Found a new line number designator.
                // If there is a previously accumulated message, report it.
                if (message.length() > 0) {
                    if (severity == IMarker.SEVERITY_ERROR)
                        errorReporter.reportError(path, lineNumber, message.toString());
                    else
                        errorReporter.reportWarning(path, lineNumber, message.toString());

                    if (!Objects.equal(originalPath.toFile(), path.toFile())) {
                        // Report an error also in the top-level resource.
                        // FIXME: It should be possible to descend through the import
                        // statements to find which one matches and mark all the
                        // import statements down the chain. But what a pain!
                        if (severity == IMarker.SEVERITY_ERROR) {
                            errorReporter.reportError(originalPath, 1, "Error in imported file: " + path);
                        } else {
                            errorReporter.reportWarning(originalPath, 1, "Warning in imported file: " + path);
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
            if (severity == IMarker.SEVERITY_ERROR) {
                errorReporter.reportError(path, lineNumber, message.toString());
            } else {
                errorReporter.reportWarning(path, lineNumber, message.toString());
            }

            if (originalPath.toFile() != path.toFile()) {
                // Report an error also in the top-level resource.
                // FIXME: It should be possible to descend through the import
                // statements to find which one matches and mark all the
                // import statements down the chain. But what a pain!
                if (severity == IMarker.SEVERITY_ERROR) {
                    errorReporter.reportError(originalPath, 1, "Error in imported file: " + path);
                } else {
                    errorReporter.reportWarning(originalPath, 1, "Warning in imported file: " + path);
                }
            }
        }
    }

    /**
     * Generate target code for a parameter reference.
     * 
     * @param param The parameter to generate code for
     * @return Parameter reference in target code
     */
    protected String getTargetReference(Parameter param) {
        return param.getName();
    }

    // //////////////////////////////////////////////////
    // // Private functions

    /**
     * Remove triggers in each federates' network reactions that are defined 
     * in remote federates.
     *
     * This must be done in code generators after the dependency graphs
     * are built and levels are assigned. Otherwise, these disconnected ports
     * might reference data structures in remote federates and cause 
     * compile/runtime errors.
     *
     * @param instance The reactor instance to remove these ports from if any.
     *  Can be null.
     */
    protected void removeRemoteFederateConnectionPorts(ReactorInstance instance) {
        if (!isFederated) {
            return;
        }
        for (FederateInstance federate : federates) {
            // Remove disconnected network triggers from the AST
            federate.removeRemoteFederateConnectionPorts();
            if (instance == null) {
                continue;
            }
            // If passed a reactor instance, also purge the disconnected network triggers
            // from the reactor instance graph
            for (Reaction reaction : federate.networkReactions) {
                ReactionInstance networkReaction = instance.lookupReactionInstance(reaction);
                if (networkReaction == null) {
                    continue;
                }
                for (VarRef port : federate.remoteNetworkReactionTriggers) {
                    PortInstance disconnectedPortInstance = instance.lookupPortInstance(port);
                    if (disconnectedPortInstance != null) {
                        networkReaction.removePortInstance(disconnectedPortInstance);
                    }
                }
            }
        }
    }

    /** 
     * Set the RTI hostname, port and username if given as compiler arguments
     */
    private void setFederationRTIProperties(LFGeneratorContext context) {
        String rtiAddr = context.getArgs().getProperty("rti").toString();
        Pattern pattern = Pattern.compile("([a-zA-Z0-9]+@)?([a-zA-Z0-9]+\\.?[a-z]{2,}|[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+):?([0-9]+)?");
        Matcher matcher = pattern.matcher(rtiAddr);

        if (!matcher.find()) {
            return;
        }

        // the user match group contains a trailing "@" which needs to be removed.
        String userWithAt = matcher.group(1);
        String user = userWithAt == null ? null : userWithAt.substring(0, userWithAt.length() - 1);
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
     * Analyze the AST to determine whether code is being mapped to
     * single or to multiple target machines. If it is being mapped
     * to multiple machines, then set the {@link #isFederated} field to true,
     * create a FederateInstance for each federate, and record various
     * properties of the federation
     * 
     * In addition, for each top-level connection, add top-level reactions to the AST
     * that send and receive messages over the network.
     * 
     * This class is target independent, so the target code
     * generator still has quite a bit of work to do.
     * It needs to provide the body of the sending and
     * receiving reactions. It also needs to provide the
     * runtime infrastructure that uses the dependency
     * information between federates. See the C target
     * for a reference implementation.
     */
    private void analyzeFederates(LFGeneratorContext context) {
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        Reactor mainReactor = mainDef != null ? ASTUtils.toDefinition(mainDef.getReactorClass()) : null;

        if (mainDef == null || !mainReactor.isFederated()) {
            // The program is not federated.
            // Ensure federates is never empty.
            FederateInstance federateInstance = new FederateInstance(null, 0, 0, this, errorReporter);
            federates.add(federateInstance);
            federateByID.put(0, federateInstance);
        } else {
            // The Lingua Franca program is federated
            isFederated = true;
            
            // If the "--rti" flag is given to the compiler, use the argument from the flag.
            if (context.getArgs().containsKey("rti")) {
                setFederationRTIProperties(context);
            } else if (mainReactor.getHost() != null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainReactor.getHost().getAddr() != null) {
                    federationRTIProperties.put("host", mainReactor.getHost().getAddr());
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainReactor.getHost().getPort() != 0) {
                    federationRTIProperties.put("port", mainReactor.getHost().getPort());
                }
                // Get the user information, if specified.
                if (mainReactor.getHost().getUser() != null) {
                    federationRTIProperties.put("user", mainReactor.getHost().getUser());
                }
            }

            // Since federates are always within the main (federated) reactor,
            // create a list containing just that one containing instantiation.
            // This will be used to look up parameter values.
            List<Instantiation> mainReactorContext = new ArrayList<>();
            mainReactorContext.add(mainDef);

            // Create a FederateInstance for each top-level reactor.
            for (Instantiation instantiation :  ASTUtils.allInstantiations(mainReactor)) {
                int bankWidth = ASTUtils.width(instantiation.getWidthSpec(), mainReactorContext);
                if (bankWidth < 0) {
                    errorReporter.reportError(instantiation, "Cannot determine bank width! Assuming width of 1.");
                    // Continue with a bank width of 1.
                    bankWidth = 1;
                }
                // Create one federate instance for each instance in a bank of reactors.
                List<FederateInstance> federateInstances = new ArrayList<>(bankWidth);
                for (int i = 0; i < bankWidth; i++) {
                    // Assign an integer ID to the federate.
                    int federateID = federates.size();
                    FederateInstance federateInstance = new FederateInstance(instantiation, federateID, i, this, errorReporter);
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
                        if (federateInstance.host != null &&
                            federateInstance.host != "localhost" &&
                            federateInstance.host != "0.0.0.0"
                        ) {
                            federateInstance.isRemote = true;
                        }
                    }
                }
                if (federatesByInstantiation == null) {
                    federatesByInstantiation = new LinkedHashMap<Instantiation, List<FederateInstance>>();
                }
                federatesByInstantiation.put(instantiation, federateInstances);
            }

            // In a federated execution, we need keepalive to be true,
            // otherwise a federate could exit simply because it hasn't received
            // any messages.
            targetConfig.keepalive = true;

            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical for physical connections and logical
            // for logical connections.
            replaceFederateConnectionsWithActions();

            // Remove the connections at the top level
            mainReactor.getConnections().clear();
        }
    }
    
    /**
     * Replace connections between federates in the AST with actions that
     * handle sending and receiving data.
     */
    private void replaceFederateConnectionsWithActions() {
        Reactor mainReactor = mainDef != null ? ASTUtils.toDefinition(mainDef.getReactorClass()) : null;

        // Each connection in the AST may represent more than one connection between
        // federate instances because of banks and multiports. We need to generate communication
        // for each of these. To do this, we create a ReactorInstance so that we don't have
        // to duplicate the rather complicated logic in that class. We specify a depth of 1,
        // so it only creates the reactors immediately within the top level, not reactors
        // that those contain.
        ReactorInstance mainInstance = new ReactorInstance(mainReactor, errorReporter, 1);

        for (ReactorInstance child : mainInstance.children) {
            for (PortInstance output : child.outputs) {
                replaceConnectionFromFederate(output, child, mainInstance);
            }
        }
    }
    
    /**
     * Replace the connections from the specified output port for the specified federate reactor.
     * @param output The output port instance.
     * @param srcFederate The federate for which this port is an output.
     * @param federateReactor The reactor instance for that federate.
     * @param mainInstance The main reactor instance.
     */
    private void replaceConnectionFromFederate(
        PortInstance output,
        ReactorInstance federateReactor,
        ReactorInstance mainInstance
    ) {
        for (SendRange srcRange : output.dependentPorts) {
            for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                
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
                        srcRange.instance.parent.definition
                    ).get(srcBank);
                    FederateInstance dstFederate = federatesByInstantiation.get(
                        dstRange.instance.parent.definition
                    ).get(dstBank);
                    
                    Connection connection = srcRange.connection;
                
                    if (connection == null) {
                        // This should not happen.
                        errorReporter.reportError(output.definition, 
                                "Unexpected error. Cannot find output connection for port");
                    } else {
                        if (srcFederate != dstFederate
                                && !connection.isPhysical() 
                                && targetConfig.coordination != CoordinationType.DECENTRALIZED) {
                            // Map the delays on connections between federates.
                            // First see if the cache has been created.
                            Set<Delay> dependsOnDelays = dstFederate.dependsOn.get(srcFederate);
                            if (dependsOnDelays == null) {
                                // If not, create it.
                                dependsOnDelays = new LinkedHashSet<Delay>();
                                dstFederate.dependsOn.put(srcFederate, dependsOnDelays);
                            }
                            // Put the delay on the cache.
                            if (connection.getDelay() != null) {
                                dependsOnDelays.add(connection.getDelay());
                            } else {
                                // To indicate that at least one connection has no delay, add a null entry.
                                dependsOnDelays.add(null);
                            }
                            // Map the connections between federates.
                            Set<Delay> sendsToDelays = srcFederate.sendsTo.get(dstFederate);
                            if (sendsToDelays == null) {
                                sendsToDelays = new LinkedHashSet<Delay>();
                                srcFederate.sendsTo.put(dstFederate, sendsToDelays);
                            }
                            if (connection.getDelay() != null) {
                                sendsToDelays.add(connection.getDelay());
                            } else {
                                // To indicate that at least one connection has no delay, add a null entry.
                                sendsToDelays.add(null);
                            }
                        }
    
                        FedASTUtils.makeCommunication(
                            srcRange.instance,
                            dstRange.instance,
                            connection,
                            srcFederate,
                            srcBank,
                            srcChannel,
                            dstFederate,
                            dstBank,
                            dstChannel,
                            this,
                            targetConfig.coordination
                        );
                    }                    
                    dstID.increment();
                    srcID.increment();
                    srcCount++;
                    if (srcCount == srcRange.width) {
                        srcID = srcRange.startMR(); // Multicast. Start over.
                    }
                }
            }
        }
    }

    /**
     * Print to stdout information about what source file is being generated,
     * what mode the generator is in, and where the generated sources are to be put.
     */
    public void printInfo(LFGeneratorContext.Mode mode) {
        System.out.println("Generating code for: " + fileConfig.resource.getURI().toString());
        System.out.println("******** mode: " + mode);
        System.out.println("******** generated sources: " + fileConfig.getSrcGenPath());
    }

    /**
     * Indicates whether delay banks generated from after delays should have a variable length width.
     * 
     * If this is true, any delay reactors that are inserted for after delays on multiport connections 
     * will have a unspecified variable length width. The code generator is then responsible for inferring the
     * correct width of the delay bank, which is only possible if the precise connection width is known at compile time.
     * 
     * If this is false, the width specification of the generated bank will list all the ports listed on the right
     * side of the connection. This gives the code generator the information needed to infer the correct width at 
     * runtime.
     */
    public boolean generateAfterDelaysWithVariableWidth() { return true; }

    /**
     * Get the buffer type used for network messages
     */
    public String getNetworkBufferType() { return ""; }

    /**
     * Return the Targets enum for the current target
     */
    public abstract Target getTarget();

    /**
     * Get textual representation of a time in the target language.
     * 
     * @param t A time AST node
     * @return A time string in the target language
     */
    public static String getTargetTime(Time t) {
        TimeValue value = new TimeValue(t.getInterval(), TimeUnit.fromName(t.getUnit()));
        return timeInTargetLanguage(value);
    }

    /**
     * Get textual representation of a value in the target language.
     * 
     * If the value evaluates to 0, it is interpreted as a normal value.
     * 
     * @param v A time AST node
     * @return A time string in the target language
     */
    public static String getTargetValue(Value v) {
        if (v.getTime() != null) {
            return getTargetTime(v.getTime());
        }
        return ASTUtils.toText(v);
    }

    /**
     * Get textual representation of a value in the target language.
     * 
     * If the value evaluates to 0, it is interpreted as a time.
     * 
     * @param v A time AST node
     * @return A time string in the target language
     */
    public static String getTargetTime(Value v) {
        if (v.getTime() != null) {
            return getTargetTime(v.getTime());
        } else if (ASTUtils.isZero(v)) {
            TimeValue value = TimeValue.ZERO;
            return timeInTargetLanguage(value);
        }
        return ASTUtils.toText(v);
    }

    public static String getTargetTime(Delay d) {
        if (d.getParameter() != null) {
            return ASTUtils.toText(d);
        } else {
            return getTargetTime(d.getTime());
        }
    }
}
