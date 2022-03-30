/*
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.lflang;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.TerminalRule;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.impl.CompositeNode;
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.util.Pair;
import org.eclipse.xtext.util.Tuples;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.StringExtensions;

import org.lflang.generator.CodeMap;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.InvalidSourceException;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.ArraySpec;
import org.lflang.lf.Assignment;
import org.lflang.lf.Code;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Element;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.Time;
import org.lflang.lf.Timer;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.util.StringUtil;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterators;

/**
 * A helper class for modifying and analyzing the AST.
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 */
public class ASTUtils {
    
    /**
     * The Lingua Franca factory for creating new AST nodes.
     */
    public static final LfFactory factory = LfFactory.eINSTANCE;
    
    /**
     * The Lingua Franca feature package.
     */
    public static final LfPackage featurePackage = LfPackage.eINSTANCE;

    /* Match an abbreviated form of a float literal. */
    private static final Pattern ABBREVIATED_FLOAT = Pattern.compile("[+\\-]?\\.\\d+[\\deE+\\-]*");
    
    /**
     * A mapping from Reactor features to corresponding Mode features for collecting contained elements.
     */
    private static final Map<EStructuralFeature, EStructuralFeature> reactorModeFeatureMap = Map.of(
            featurePackage.getReactor_Actions(),        featurePackage.getMode_Actions(),
            featurePackage.getReactor_Connections(),    featurePackage.getMode_Connections(),
            featurePackage.getReactor_Instantiations(), featurePackage.getMode_Instantiations(),
            featurePackage.getReactor_Reactions(),      featurePackage.getMode_Reactions(),
            featurePackage.getReactor_StateVars(),      featurePackage.getMode_StateVars(),
            featurePackage.getReactor_Timers(),         featurePackage.getMode_Timers()
            );


    /**
     * Get all reactors defined in the given resource.
     * @param resource the resource to extract reactors from
     * @return An iterable over all reactors found in the resource
     */
    public static Iterable<Reactor> getAllReactors(Resource resource) {
        return StreamSupport.stream(IteratorExtensions.toIterable(resource.getAllContents()).spliterator(), false)
                     .filter(Reactor.class::isInstance)
                     .map(Reactor.class::cast)
                     .collect(Collectors.toList());
    }

    /**
     * Find connections in the given resource that have a delay associated with them, 
     * and reroute them via a generated delay reactor.
     * @param resource The AST.
     * @param generator A code generator.
     */
    public static void insertGeneratedDelays(Resource resource, GeneratorBase generator) {
        // The resulting changes to the AST are performed _after_ iterating 
        // in order to avoid concurrent modification problems.
        List<Connection> oldConnections = new ArrayList<>();
        Map<EObject, List<Connection>> newConnections = new LinkedHashMap<>();
        Map<EObject, List<Instantiation>> delayInstances = new LinkedHashMap<>();

        // Iterate over the connections in the tree.
        for (Reactor container : getAllReactors(resource)) {
            for (Connection connection : allConnections(container)) {
                if (connection.getDelay() != null) { 
                    EObject parent = connection.eContainer();
                    // Assume all the types are the same, so just use the first on the right.
                    Type type = ((Port) connection.getRightPorts().get(0).getVariable()).getType();
                    Reactor delayClass = getDelayClass(type, generator);
                    String generic = generator.getTargetTypes().supportsGenerics() ? generator.getTargetTypes().getTargetType(InferredType.fromAST(type)) : "";
                    Instantiation delayInstance = getDelayInstance(delayClass, connection, generic, 
                        !generator.generateAfterDelaysWithVariableWidth());

                    // Stage the new connections for insertion into the tree.
                    List<Connection> connections = convertToEmptyListIfNull(newConnections.get(parent));
                    connections.addAll(rerouteViaDelay(connection, delayInstance));
                    newConnections.put(parent, connections);
                    // Stage the original connection for deletion from the tree.
                    oldConnections.add(connection);

                    // Stage the newly created delay reactor instance for insertion
                    List<Instantiation> instances = convertToEmptyListIfNull(delayInstances.get(parent));
                    instances.add(delayInstance);
                    delayInstances.put(parent, instances);
                }
            }
        }

        // Remove old connections; insert new ones.
        oldConnections.forEach(connection -> {
            var container = connection.eContainer();
            if (container instanceof Reactor) {
                ((Reactor) container).getConnections().remove(connection);
            } else if (container instanceof Mode) {
                ((Mode) container).getConnections().remove(connection);
            }
        });
        newConnections.forEach((container, connections) -> {
            if (container instanceof Reactor) {
                ((Reactor) container).getConnections().addAll(connections);
            } else if (container instanceof Mode) {
                ((Mode) container).getConnections().addAll(connections);
            }
        });
        // Finally, insert the instances and, before doing so, assign them a unique name.
        delayInstances.forEach((container, instantiations) -> 
            instantiations.forEach(instantiation -> {
                if (container instanceof Reactor) {
                    instantiation.setName(getUniqueIdentifier((Reactor) container, "delay"));
                    ((Reactor) container).getInstantiations().add(instantiation);
                } else if (container instanceof Mode) {
                    instantiation.setName(getUniqueIdentifier((Reactor) container.eContainer(), "delay"));
                    ((Mode) container).getInstantiations().add(instantiation);
                }
            })
        );
    }
    
    /**
     * Find connections in the given resource that would be conflicting writes if they were not located in mutually
     * exclusive modes.
     * 
     * @param resource The AST.
     * @return a list of connections being able to be transformed
     */
    public static Collection<Connection> findConflictingConnectionsInModalReactors(Resource resource) {
        var transform = new HashSet<Connection>();

        for (Reactor reactor : getAllReactors(resource)) {
            if (!reactor.getModes().isEmpty()) { // Only for modal reactors
                var allWriters = HashMultimap.<Pair<Instantiation, Variable>, EObject>create();
                
                // Collect destinations
                for (var rea : allReactions(reactor)) {
                    for (var eff : rea.getEffects()) {
                        if (eff.getVariable() instanceof Port) {
                            allWriters.put(Tuples.pair(eff.getContainer(), eff.getVariable()), rea);
                        }
                    }
                }
                for (var con : ASTUtils.<Connection>collectElements(reactor, featurePackage.getReactor_Connections(), false, true)) {
                    for (var port : con.getRightPorts()) {
                        allWriters.put(Tuples.pair(port.getContainer(), port.getVariable()), con);
                    }
                }
                
                // Handle conflicting writers
                for (var key : allWriters.keySet()) {
                    var writers = allWriters.get(key);
                    if (writers.size() > 1) { // has multiple sources
                        var writerModes = HashMultimap.<Mode, EObject>create();
                        // find modes
                        for (var writer : writers) {
                            if (writer.eContainer() instanceof Mode) {
                                writerModes.put((Mode) writer.eContainer(), writer);
                            } else {
                                writerModes.put(null, writer);
                            }
                        }
                        // Conflicting connection can only be handled if..
                        if (!writerModes.containsKey(null) && // no writer is on root level (outside of modes) and...
                            writerModes.keySet().stream().map(writerModes::get).allMatch(writersInMode -> // all writers in a mode are either...
                                writersInMode.size() == 1 || // the only writer or...
                                writersInMode.stream().allMatch(w -> w instanceof Reaction) // all are reactions and hence ordered
                            )) {
                            // Add connections to transform list
                            writers.stream().filter(w -> w instanceof Connection).forEach(c -> transform.add((Connection) c));
                        }
                    }
                }
            }
        }
        
        return transform;
    }
    
    /**
     * Return the enclosing reactor of an LF EObject in a reactor or mode. 
     * @param obj the LF model element
     * @return the reactor or null
     */
    public static Reactor getEnclosingReactor(EObject obj) {
        if (obj.eContainer() instanceof Reactor) {
            return (Reactor) obj.eContainer();
        } else if (obj.eContainer() instanceof Mode) {
            return (Reactor) obj.eContainer().eContainer();
        }
        return null;
    }
    
    /**
     * Find the main reactor and change it to a federated reactor.
     * Return true if the transformation was successful (or the given resource
     * already had a federated reactor); return false otherwise.
     */
    public static boolean makeFederated(Resource resource) {
        // Find the main reactor
        Reactor r = IteratorExtensions.findFirst(
            Iterators.filter(resource.getAllContents(), Reactor.class),
            Reactor::isMain
        );
        if (r == null) {
            return false;
        }
        r.setMain(false);
        r.setFederated(true);
        return true;
    }
    
    /**
     * Change the target name to 'newTargetName'.
     * For example, change C to CCpp.
     */
    public static boolean changeTargetName(Resource resource, String newTargetName) {
        targetDecl(resource).setName(newTargetName);
        return true;
    }

    /**
     * Add a new target property to the given resource.
     *
     * This also creates a config object if the resource does not yey have one.
     *
     * @param resource The resource to modify
     * @param name Name of the property to add
     * @param value Value to be assigned to the property
     */
    public static boolean addTargetProperty(final Resource resource, final String name, final Element value) {
        var config = targetDecl(resource).getConfig();
        if (config == null) {
            config = LfFactory.eINSTANCE.createKeyValuePairs();
            targetDecl(resource).setConfig(config);
        }
        final var newProperty = LfFactory.eINSTANCE.createKeyValuePair();
        newProperty.setName(name);
        newProperty.setValue(value);
        config.getPairs().add(newProperty);
        return true;
    }
    
    /**
     * Return true if the connection involves multiple ports on the left or right side of the connection, or
     * if the port on the left or right of the connection involves a bank of reactors or a multiport.
     * @param connection The connection.
     */
    public static boolean hasMultipleConnections(Connection connection) {
        if (connection.getLeftPorts().size() > 1 || connection.getRightPorts().size() > 1) {
            return true;
        }
        VarRef leftPort  = connection.getLeftPorts().get(0);
        VarRef rightPort = connection.getRightPorts().get(0);
        Instantiation leftContainer  = leftPort.getContainer();
        Instantiation rightContainer = rightPort.getContainer();
        Port leftPortAsPort  = (Port) leftPort.getVariable();
        Port rightPortAsPort = (Port) rightPort.getVariable();
        return leftPortAsPort.getWidthSpec() != null
            || leftContainer != null && leftContainer.getWidthSpec() != null
            || rightPortAsPort.getWidthSpec() != null
            || rightContainer != null && rightContainer.getWidthSpec() != null;
    }
    
    /**
     * Take a connection and reroute it via an instance of a generated delay
     * reactor. This method returns a list to new connections to substitute
     * the original one.
     * @param connection The connection to reroute.
     * @param delayInstance The delay instance to route the connection through.
     */
    private static List<Connection> rerouteViaDelay(Connection connection, 
            Instantiation delayInstance) {
        List<Connection> connections = new ArrayList<>();    
        Connection upstream = factory.createConnection();
        Connection downstream = factory.createConnection();
        VarRef input = factory.createVarRef();
        VarRef output = factory.createVarRef();

        Reactor delayClass = toDefinition(delayInstance.getReactorClass());
        
        // Establish references to the involved ports.
        input.setContainer(delayInstance);
        input.setVariable(delayClass.getInputs().get(0));
        output.setContainer(delayInstance);
        output.setVariable(delayClass.getOutputs().get(0));
        upstream.getLeftPorts().addAll(connection.getLeftPorts());
        upstream.getRightPorts().add(input);
        downstream.getLeftPorts().add(output);
        downstream.getRightPorts().addAll(connection.getRightPorts());
        downstream.setIterated(connection.isIterated());
        connections.add(upstream);
        connections.add(downstream);
        return connections;
    }
    
    /**
     * Create a new instance delay instances using the given reactor class.
     * The supplied time value is used to override the default interval (which
     * is zero).
     * If the target supports parametric polymorphism, then a single class may
     * be used for each instantiation, in which case a non-empty string must
     * be supplied to parameterize the instance.
     * A default name ("delay") is assigned to the instantiation, but this
     * name must be overridden at the call site, where checks can be done to
     * avoid name collisions in the container in which the instantiation is
     * to be placed. Such checks (or modifications of the AST) are not
     * performed in this method in order to avoid causing concurrent
     * modification exceptions. 
     * @param delayClass The class to create an instantiation for
     * @param connection The connection to create a delay instantiation foe
     * @param generic A string that denotes the appropriate type parameter, 
     *  which should be null or empty if the target does not support generics.
     * @param defineWidthFromConnection If this is true and if the connection 
     *  is a wide connection, then instantiate a bank of delays where the width
     *  is given by ports involved in the connection. Otherwise, the width will
     *  be  unspecified indicating a variable length.
     */
    private static Instantiation getDelayInstance(Reactor delayClass, 
            Connection connection, String generic, Boolean defineWidthFromConnection) {
        Delay delay = connection.getDelay();
        Instantiation delayInstance = factory.createInstantiation();
        delayInstance.setReactorClass(delayClass);
        if (!StringExtensions.isNullOrEmpty(generic)) {
            TypeParm typeParm = factory.createTypeParm();
            typeParm.setLiteral(generic);
            delayInstance.getTypeParms().add(typeParm);
        }
        if (hasMultipleConnections(connection)) {
            WidthSpec widthSpec = factory.createWidthSpec();
            if (defineWidthFromConnection) {
                // Add all left ports of the connection to the WidthSpec of the generated delay instance.
                // This allows the code generator to later infer the width from the involved ports.
                // We only consider the left ports here, as they could be part of a broadcast. In this case, we want
                // to delay the ports first, and then broadcast the output of the delays.
                for (VarRef port : connection.getLeftPorts()) {
                    WidthTerm term = factory.createWidthTerm();
                    term.setPort(EcoreUtil.copy(port));
                    widthSpec.getTerms().add(term);
                }   
            } else {
                widthSpec.setOfVariableLength(true);
            }
            delayInstance.setWidthSpec(widthSpec);
        }
        Assignment assignment = factory.createAssignment();
        assignment.setLhs(delayClass.getParameters().get(0));
        Value value = factory.createValue();
        if (delay.getParameter() != null) {
            value.setParameter(delay.getParameter());
        } else {
            value.setTime(delay.getTime());
        }
        assignment.getRhs().add(value);
        delayInstance.getParameters().add(assignment);
        delayInstance.setName("delay");  // This has to be overridden.
        return delayInstance;
    }
    
    /**
     * Return a synthesized AST node that represents the definition of a delay
     * reactor. Depending on whether the target supports generics, either this
     * method will synthesize a generic definition and keep returning it upon
     * subsequent calls, or otherwise, it will synthesize a new definition for 
     * each new type it hasn't yet created a compatible delay reactor for.
     * @param type The type the delay class must be compatible with.
     * @param generator A code generator.
     */
    private static Reactor getDelayClass(Type type, GeneratorBase generator) {
        String className;
        if (generator.getTargetTypes().supportsGenerics()) {
            className = GeneratorBase.GEN_DELAY_CLASS_NAME;
        } else {
            String id = Integer.toHexString(InferredType.fromAST(type).toText().hashCode());
            className = String.format("%s_%s", GeneratorBase.GEN_DELAY_CLASS_NAME, id);
        }

        // Only add class definition if it is not already there.
        Reactor classDef = generator.findDelayClass(className);
        if (classDef != null) {
            return classDef;
        }
        
        Reactor delayClass = factory.createReactor();
        Parameter delayParameter = factory.createParameter();
        Action action = factory.createAction();
        VarRef triggerRef = factory.createVarRef();
        VarRef effectRef = factory.createVarRef();
        Input input = factory.createInput();
        Output output = factory.createOutput();
        VarRef inRef = factory.createVarRef();
        VarRef outRef = factory.createVarRef();

        Reaction r1 = factory.createReaction();
        Reaction r2 = factory.createReaction();
        
        delayParameter.setName("delay");
        delayParameter.setType(factory.createType());
        delayParameter.getType().setId("time");
        delayParameter.getType().setTime(true);
        Time defaultTime = factory.createTime();
        defaultTime.setUnit(null);
        defaultTime.setInterval(0);
        Value defaultValue = factory.createValue();
        defaultValue.setTime(defaultTime);
        delayParameter.getInit().add(defaultValue);

        // Name the newly created action; set its delay and type.
        action.setName("act");
        action.setMinDelay(factory.createValue());
        action.getMinDelay().setParameter(delayParameter);
        action.setOrigin(ActionOrigin.LOGICAL);

        if (generator.getTargetTypes().supportsGenerics()) {
            action.setType(factory.createType());
            action.getType().setId("T");
        } else {
            action.setType(EcoreUtil.copy(type));
        }

        input.setName("inp");
        input.setType(EcoreUtil.copy(action.getType()));

        output.setName("out");
        output.setType(EcoreUtil.copy(action.getType()));

        // Establish references to the involved ports.
        inRef.setVariable(input);
        outRef.setVariable(output);

        // Establish references to the action.
        triggerRef.setVariable(action);
        effectRef.setVariable(action);

        // Add the action to the reactor.
        delayClass.setName(className);
        delayClass.getActions().add(action);

        // Configure the second reaction, which reads the input.
        r1.getTriggers().add(inRef);
        r1.getEffects().add(effectRef);
        r1.setCode(factory.createCode());
        r1.getCode().setBody(generator.generateDelayBody(action, inRef));
        
        // Configure the first reaction, which produces the output.
        r2.getTriggers().add(triggerRef);
        r2.getEffects().add(outRef);
        r2.setCode(factory.createCode());
        r2.getCode().setBody(generator.generateForwardBody(action, outRef));

        // Add the reactions to the newly created reactor class.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        delayClass.getReactions().add(r2);
        delayClass.getReactions().add(r1);

        // Add a type parameter if the target supports it.
        if (generator.getTargetTypes().supportsGenerics()) {
            TypeParm parm = factory.createTypeParm();
            parm.setLiteral(generator.generateDelayGeneric());
            delayClass.getTypeParms().add(parm);
        }
        delayClass.getInputs().add(input);
        delayClass.getOutputs().add(output);
        delayClass.getParameters().add(delayParameter);
        generator.addDelayClass(delayClass);
        return delayClass;
    }
    
    /**
     * Produce a unique identifier within a reactor based on a
     * given based name. If the name does not exists, it is returned;
     * if does exist, an index is appended that makes the name unique.
     * @param reactor The reactor to find a unique identifier within.
     * @param name The name to base the returned identifier on.
     */
    public static String getUniqueIdentifier(Reactor reactor, String name) {
        LinkedHashSet<String> vars = new LinkedHashSet<>();
        allActions(reactor).forEach(it -> vars.add(it.getName()));
        allTimers(reactor).forEach(it -> vars.add(it.getName()));
        allParameters(reactor).forEach(it -> vars.add(it.getName()));
        allInputs(reactor).forEach(it -> vars.add(it.getName()));
        allOutputs(reactor).forEach(it -> vars.add(it.getName()));
        allStateVars(reactor).forEach(it -> vars.add(it.getName()));
        allInstantiations(reactor).forEach(it -> vars.add(it.getName()));

        int index = 0;
        String suffix = "";
        boolean exists = true; 
        while (exists) {
            String id = name + suffix;
            if (IterableExtensions.exists(vars, it -> it.equals(id))) {
                suffix = "_" + index;
                index++;
            } else {
                exists = false;
            }
        }
        return name + suffix;
    }
   
    ////////////////////////////////
    //// Utility functions for supporting inheritance and modes
    
    /**
     * Given a reactor class, return a list of all its actions,
     * which includes actions of base classes that it extends.
     * This also includes actions in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<Action> allActions(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Actions());
    }
    
    /**
     * Given a reactor class, return a list of all its connections,
     * which includes connections of base classes that it extends.
     * This also includes connections in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<Connection> allConnections(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Connections());
    }
    
    /**
     * Given a reactor class, return a list of all its inputs,
     * which includes inputs of base classes that it extends.
     * If the base classes include a cycle, where X extends Y and Y extends X,
     * then return only the input defined in the base class.
     * The returned list may be empty.
     * @param definition Reactor class definition.
     */
    public static List<Input> allInputs(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Inputs());
    }
    
    /**
     * Given a reactor class, return a list of all its instantiations,
     * which includes instantiations of base classes that it extends.
     * This also includes instantiations in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<Instantiation> allInstantiations(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Instantiations());
    }
    
    /**
     * Given a reactor class, return a list of all its outputs,
     * which includes outputs of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Output> allOutputs(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Outputs());
    }

    /**
     * Given a reactor class, return a list of all its parameters,
     * which includes parameters of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Parameter> allParameters(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Parameters());
    }
    
    /**
     * Given a reactor class, return a list of all its reactions,
     * which includes reactions of base classes that it extends.
     * This also includes reactions in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<Reaction> allReactions(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Reactions());
    }
    
    /**
     * Given a reactor class, return a list of all its state variables,
     * which includes state variables of base classes that it extends.
     * This also includes reactions in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<StateVar> allStateVars(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_StateVars());
    }
    
    /**
     * Given a reactor class, return a list of all its timers,
     * which includes timers of base classes that it extends.
     * This also includes reactions in modes, returning a flattened
     * view over all modes.
     * @param definition Reactor class definition.
     */
    public static List<Timer> allTimers(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Timers());
    }

    /**
     * Given a reactor class, returns a list of all its modes,
     * which includes modes of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Mode> allModes(Reactor definition) {
        return ASTUtils.collectElements(definition, featurePackage.getReactor_Modes());
    }
    
    /**
     * Return all the superclasses of the specified reactor
     * in deepest-first order. For example, if A extends B and C, and
     * B and C both extend D, this will return the list [D, B, C, A].
     * Duplicates are removed. If the specified reactor does not extend
     * any other reactor, then return an empty list.
     * If a cycle is found, where X extends Y and Y extends X, or if
     * a superclass is declared that is not found, then return null.
     * @param reactor The specified reactor.
     */
    public static LinkedHashSet<Reactor> superClasses(Reactor reactor) {
        return superClasses(reactor, new LinkedHashSet<>());
    }

    /**
     * Collect elements of type T from the class hierarchy and modes
     * defined by a given reactor definition.
     * @param definition The reactor definition.
     * @param <T> The type of elements to collect (e.g., Port, Timer, etc.)
     * @return A list of all elements of type T found
     */
    public static <T> List<T> collectElements(Reactor definition, EStructuralFeature feature) {
        return ASTUtils.collectElements(definition, feature, true, true);
    }
    
    /**
     * Collect elements of type T contained in given reactor definition, including
     * modes and the class hierarchy defined depending on configuration.
     * @param definition The reactor definition.
     * @param feature The structual model elements to collect.
     * @param includeSuperClasses Whether to include elements in super classes.
     * @param includeModes Whether to include elements in modes.
     * @param <T> The type of elements to collect (e.g., Port, Timer, etc.)
     * @return A list of all elements of type T found
     */
    @SuppressWarnings("unchecked")
    public static <T> List<T> collectElements(Reactor definition, EStructuralFeature feature, boolean includeSuperClasses, boolean includeModes) {
        List<T> result = new ArrayList<>();
        
        if (includeSuperClasses) {
            // Add elements of elements defined in superclasses.
            LinkedHashSet<Reactor> s = superClasses(definition);
            if (s != null) {
                for (Reactor superClass : s) {
                    result.addAll((EList<T>) superClass.eGet(feature));
                }
            }
        }
        
        // Add elements of the current reactor.
        result.addAll((EList<T>) definition.eGet(feature));
        
        if (includeModes && reactorModeFeatureMap.containsKey(feature)) {
            var modeFeature = reactorModeFeatureMap.get(feature);
            // Add elements of elements defined in modes.
            for (Mode mode : includeSuperClasses ? allModes(definition) : definition.getModes()) {
                result.addAll((EList<T>) mode.eGet(modeFeature));
            }
        }
        
        return result;
    }

    ////////////////////////////////
    //// Utility functions for translating AST nodes into text

    /**
     * Translate the given code into its textual representation.
     * @param code AST node to render as string.
     * @return Textual representation of the given argument.
     */
    public static String toText(Code code) {
        return CodeMap.Correspondence.tag(code, toUntaggedText(code), true);
    }

    /**
     * Translate the given code into its textual representation
     * without any {@code CodeMap.Correspondence} tags inserted.
     * @param code AST node to render as string.
     * @return Textual representation of the given argument.
     */
    private static String toUntaggedText(Code code) {
        // FIXME: This function should not be necessary, but it is because we currently inspect the
        //  content of code blocks in the validator and generator (using regexes, etc.). See #810, #657.
        String text = "";
        if (code != null) {
            ICompositeNode node = NodeModelUtils.getNode(code);
            if (node != null) {
                StringBuilder builder = new StringBuilder(Math.max(node.getTotalLength(), 1));
                for (ILeafNode leaf : node.getLeafNodes()) {
                    builder.append(leaf.getText());
                }
                String str = builder.toString().trim();
                // Remove the code delimiters (and any surrounding comments).
                // This assumes any comment before {= does not include {=.
                int start = str.indexOf("{=");
                int end = str.indexOf("=}", start);
                if (start == -1 || end == -1) {
                    // Silent failure is needed here because toText is needed to create the intermediate representation,
                    // which the validator uses.
                    return str;
                }
                str = str.substring(start + 2, end);
                if (str.split("\n").length > 1) {
                    // multi line code
                    text = StringUtil.trimCodeBlock(str);
                } else {
                    // single line code
                    text = str.trim();
                }
            } else if (code.getBody() != null) {
                // Code must have been added as a simple string.
                text = code.getBody();
            }
        }
        return text;
    }
    
    public static String toText(TypeParm t) {
        return !StringExtensions.isNullOrEmpty(t.getLiteral()) ? t.getLiteral() : toText(t.getCode());
    }
    
    /**
     * Return a textual representation of the given element, 
     * without quotes if there are any. Leading or trailing 
     * whitespace is removed.
     * 
     * @param e The element to be rendered as a string.
     */
    public static String toText(Element e) {
        String str = "";
        if (e.getLiteral() != null) {
            str = withoutQuotes(e.getLiteral()).trim();
        }
        if (e.getId() != null) {
            str = e.getId();
        }
        return str;
    }
    
    /**
     * Return an integer representation of the given element.
     * 
     * Internally, this method uses Integer.decode, so it will
     * also understand hexadecimal, binary, etc.
     * 
     * @param e The element to be rendered as an integer.
     */
    public static Integer toInteger(Element e) {
        return Integer.decode(e.getLiteral());
    }
    
    /**
     * Return a time value based on the given element.
     * 
     * @param e The element to be rendered as a time value.
     */
    public static TimeValue toTimeValue(Element e) {
        return new TimeValue(e.getTime(), TimeUnit.fromName(e.getUnit()));
    }
    
    /**
     * Returns the time value represented by the given AST node.
     */
    public static TimeValue toTimeValue(Time e) {
        if (!isValidTime(e)) {
            // invalid unit, will have been reported by validator
            throw new IllegalArgumentException();
        }
        return new TimeValue(e.getInterval(), TimeUnit.fromName(e.getUnit()));
    }

    /**
     * Return a boolean based on the given element.
     * 
     * @param e The element to be rendered as a boolean.
     */
    public static boolean toBoolean(Element e) {
        return toText(e).equalsIgnoreCase("true");
    }
    
    /**
     * Convert a time to its textual representation as it would
     * appear in LF code.
     * 
     * @param t The time to be converted
     * @return A textual representation
     */
    public static String toText(Time t) {
        return toTimeValue(t).toString();
    }
        
    /**
     * Convert a value to its textual representation as it would
     * appear in LF code.
     * 
     * @param v The value to be converted
     * @return A textual representation
     */
    public static String toText(Value v) {
        if (v.getParameter() != null) {
            return v.getParameter().getName();
        }
        if (v.getTime()!= null) {
            return toText(v.getTime());
        }
        if (v.getLiteral() != null) {
            return v.getLiteral();
        }
        if (v.getCode() != null) {
            return toText(v.getCode());
        }
        return "";
    }
    
    public static String toText(Delay d) {
        if (d.getParameter() != null) {
            return d.getParameter().getName();
        }
        return toText(d.getTime());
    }
    
    /**
     * Return a string of the form either "name" or "container.name" depending
     * on in which form the variable reference was given.
     * @param v The variable reference.
     */
    public static String toText(VarRef v) {
        if (v.getContainer() != null) {
            return String.format("%s.%s", v.getClass().getName(), v.getVariable().getName());
        } else {
            return v.getVariable().getName();
        }
    }
    
    /**
     * Convert an array specification to its textual representation as it would
     * appear in LF code.
     * 
     * @param spec The array spec to be converted
     * @return A textual representation
     */
    public static String toText(ArraySpec spec) {
        if (spec != null) {
            return (spec.isOfVariableLength()) ? "[]" : "[" + spec.getLength() + "]";
        }
        return "";
    }
    
    /**
     * Translate the given type into its textual representation, including
     * any array specifications.
     * @param type AST node to render as string.
     * @return Textual representation of the given argument.
     */
    public static String toText(Type type) {
        if (type != null) {
            String base = baseType(type);
            String arr = (type.getArraySpec() != null) ? toText(type.getArraySpec()) : "";
            return base + arr;
        }
        return "";
    }
    
    /**
     * Given the right-hand side of a target property, return a list with all
     * the strings that the property lists.
     * 
     * Arrays are traversed, so strings are collected recursively. Empty strings
     * are ignored; they are not added to the list.
     * @param value The right-hand side of a target property.
     */
    public static List<String> toListOfStrings(Element value) {
        List<String> elements = new ArrayList<>();
        if (value.getArray() != null) {
            for (Element element : value.getArray().getElements()) {
                elements.addAll(toListOfStrings(element));
            }
            return elements;
        } else {
            String v = toText(value);
            if (!v.isEmpty()) {
                elements.add(toText(value));
            }
        }
        return elements;
    }
    
    /**
     * Translate the given type into its textual representation, but
     * do not append any array specifications.
     * @param type AST node to render as string.
     * @return Textual representation of the given argument.
     */
    public static String baseType(Type type) {
        if (type != null) {
            if (type.getCode() != null) {
                return toText(type.getCode());
            } else {
                if (type.isTime()) {
                    return "time";
                } else {
                    StringBuilder stars = new StringBuilder();
                    List<String> iterList = convertToEmptyListIfNull(type.getStars());
                    for (String s : iterList) {
                        stars.append(s);
                    }
                    if (!IterableExtensions.isNullOrEmpty(type.getTypeParms())) {
                        List<String> typeParamsStr = new ArrayList<>();
                        type.getTypeParms().forEach(it -> typeParamsStr.add(toText(it)));
                        return String.format("%s<%s>", type.getId(), String.join(", ", typeParamsStr));
                    } else {
                        return type.getId() + stars;
                    }
                }
            }
        }
        return "";
    }
        
    /**
     * Report whether the given literal is zero or not.
     * @param literal AST node to inspect.
     * @return True if the given literal denotes the constant `0`, false
     * otherwise.
     */
    public static boolean isZero(String literal) {
        try {
            if (literal != null &&
                Integer.parseInt(literal) == 0) {
                return true;
            }
        } catch (NumberFormatException e) {
            // Not an int.
        }
        return false;
    }
    
    public static boolean isZero(Code code) {
        return code != null && isZero(toUntaggedText(code));
    }
    
    /**
     * Report whether the given value is zero or not.
     * @param value AST node to inspect.
     * @return True if the given value denotes the constant `0`, false otherwise.
     */
    public static boolean isZero(Value value) {
        if (value.getLiteral() != null) {
            return isZero(value.getLiteral());
        } else if (value.getCode() != null) {
            return isZero(value.getCode());
        }
        return false;
    }
    
    
    /**
     * Report whether the given string literal is an integer number or not.
     * @param literal AST node to inspect.
     * @return True if the given value is an integer, false otherwise.
     */
    public static boolean isInteger(String literal) {
        try {
            //noinspection ResultOfMethodCallIgnored
            Integer.decode(literal);
        } catch (NumberFormatException e) {
            return false;
        }
        return true;
    }

	/**
     * Report whether the given code is an integer number or not.
     * @param code AST node to inspect.
     * @return True if the given code is an integer, false otherwise.
     */
	public static boolean isInteger(Code code) {
        return isInteger(toUntaggedText(code));
    }
    
    /**
     * Report whether the given value is an integer number or not.
     * @param value AST node to inspect.
     * @return True if the given value is an integer, false otherwise.
     */
    public static boolean isInteger(Value value) {
        if (value.getLiteral() != null) {
            return isInteger(value.getLiteral());
        } else if (value.getCode() != null) {
            return isInteger(value.getCode());
        }
        return false;
    }
    
    /**
     * Report whether the given value denotes a valid time or not.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    public static boolean isValidTime(Value value) {
        if (value != null) {
            if (value.getParameter() != null) {
                return isOfTimeType(value.getParameter());
            } else if (value.getTime() != null) {
                return isValidTime(value.getTime());
            } else if (value.getLiteral() != null) {
                return isZero(value.getLiteral());
            } else if (value.getCode() != null) {
                return isZero(value.getCode());
            }
        }
        return false;
    }

    /**
     * Report whether the given time denotes a valid time or not.
     * @param t AST node to inspect.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    public static boolean isValidTime(Time t) {
        if (t == null) return false;
        String unit = t.getUnit();
        return t.getInterval() == 0 ||
               TimeUnit.isValidUnit(unit);
    }

    /**
     * Return the type of a declaration with the given
     * (nullable) explicit type, and the given (nullable)
     * initializer. If the explicit type is null, then the
     * type is inferred from the initializer. Only two types
     * can be inferred: "time" and "timeList". Return the
     * "undefined" type if neither can be inferred.
     *
     * @param type     Explicit type declared on the declaration
     * @param initList A list of values used to initialize a parameter or
     *                 state variable.
     * @return The inferred type, or "undefined" if none could be inferred.
     */
    public static InferredType getInferredType(Type type, List<Value> initList) {
        if (type != null) {
            return InferredType.fromAST(type);
        } else if (initList == null) {
            return InferredType.undefined();
        }

        if (initList.size() == 1) {
            // If there is a single element in the list, and it is a proper
            // time value with units, we infer the type "time".
            Value init = initList.get(0);
            if (init.getParameter() != null) {
                return getInferredType(init.getParameter());
            } else if (ASTUtils.isValidTime(init) && !ASTUtils.isZero(init)) {
                return InferredType.time();
            }
        } else if (initList.size() > 1) {
            // If there are multiple elements in the list, and there is at
            // least one proper time value with units, and all other elements
            // are valid times (including zero without units), we infer the
            // type "time list".
            var allValidTime = true;
            var foundNonZero = false;

            for (var init : initList) {
                if (!ASTUtils.isValidTime(init)) {
                    allValidTime = false;
                }
                if (!ASTUtils.isZero(init)) {
                    foundNonZero = true;
                }
            }

            if (allValidTime && foundNonZero) {
                // Conservatively, no bounds are inferred; the returned type
                // is a variable-size list.
                return InferredType.timeList();
            }
        }
        return InferredType.undefined();
    }

    /**
     * Given a parameter, return an inferred type. Only two types can be
     * inferred: "time" and "timeList". Return the "undefined" type if
     * neither can be inferred.
     *
     * @param p A parameter to infer the type of.
     * @return The inferred type, or "undefined" if none could be inferred.
     */
    public static InferredType getInferredType(Parameter p) {
        return getInferredType(p.getType(), p.getInit());
    }

    /**
     * Given a state variable, return an inferred type. Only two types can be
     * inferred: "time" and "timeList". Return the "undefined" type if
     * neither can be inferred.
     *
     * @param s A state variable to infer the type of.
     * @return The inferred type, or "undefined" if none could be inferred.
     */
    public static InferredType getInferredType(StateVar s) {
        return getInferredType(s.getType(), s.getInit());
    }

    /**
     * Construct an inferred type from an "action" AST node based
     * on its declared type. If no type is declared, return the "undefined"
     * type.
     *
     * @param a An action to construct an inferred type object for.
     * @return The inferred type, or "undefined" if none was declared.
     */
    public static InferredType getInferredType(Action a) {
        return getInferredType(a.getType(), null);
    }

    /**
     * Construct an inferred type from a "port" AST node based on its declared
     * type. If no type is declared, return the "undefined" type.
     *
     * @param p A port to construct an inferred type object for.
     * @return The inferred type, or "undefined" if none was declared.
     */
    public static InferredType getInferredType(Port p) {
        return getInferredType(p.getType(), null);
    }

    

    /**
     * If the given string can be recognized as a floating-point number that has a leading decimal point,
     * prepend the string with a zero and return it. Otherwise, return the original string.
     *
     * @param literal A string might be recognizable as a floating point number with a leading decimal point.
     * @return an equivalent representation of <code>literal
     * </code>
     */
    public static String addZeroToLeadingDot(String literal) {
        Matcher m = ABBREVIATED_FLOAT.matcher(literal);
        if (m.matches()) {
            return literal.replace(".", "0.");
        }
        return literal;
    }

    /**
     * Return true if the specified port is a multiport.
     * @param port The port.
     * @return True if the port is a multiport.
     */
    public static boolean isMultiport(Port port) {
        return port.getWidthSpec() != null;
    }

    ////////////////////////////////
    //// Utility functions for translating AST nodes into text
    // This is a continuation of a large section of ASTUtils.xtend
    // with the same name.

    /**
     * Generate code for referencing a port, action, or timer.
     * @param reference The reference to the variable.
     */
    public static String generateVarRef(VarRef reference) {
        var prefix = "";
        if (reference.getContainer() != null) {
            prefix = reference.getContainer().getName() + ".";
        }
        return prefix + reference.getVariable().getName();
    }

    /**
     * Assuming that the given value denotes a valid time literal,
     * return a time value.
     */
    public static TimeValue getLiteralTimeValue(Value v) {
        if (v.getTime() != null) {
            return toTimeValue(v.getTime());
        } else if (v.getLiteral() != null && v.getLiteral().equals("0")) {
            return TimeValue.ZERO;
        } else {
            return null;
        }
    }

    /**
     * If the parameter is of time type, return its default value.
     * Otherwise, return null.
     */
    public static TimeValue getDefaultAsTimeValue(Parameter p) {
        if (isOfTimeType(p)) {
            var init = p.getInit().get(0);
            if (init != null) {
                return getLiteralTimeValue(init);
            }
        }
        return null;
    }

    /**
     * Return whether the given state variable is inferred
     * to a time type.
     */
    public static boolean isOfTimeType(StateVar state) {
        InferredType t = getInferredType(state);
        return t.isTime && !t.isList;
    }

    /**
     * Return whether the given parameter is inferred
     * to a time type.
     */
    public static boolean isOfTimeType(Parameter param) {
        InferredType t = getInferredType(param);
        return t.isTime && !t.isList;
    }

    

        
    /**
     * Given a parameter, return its initial value.
     * The initial value is a list of instances of Value, where each
     * Value is either an instance of Time, Literal, or Code.
     * 
     * If the instantiations argument is null or an empty list, then the
     * value returned is simply the default value given when the parameter
     * is defined.
     * 
     * If a list of instantiations is given, then the first instantiation
     * is required to be an instantiation of the reactor class that is 
     * parameterized by the parameter. I.e.,
     * ```
     *     parameter.eContainer == instantiations.get(0).reactorClass
     * ```
     * If a second instantiation is given, then it is required to be an instantiation of a
     * reactor class that contains the first instantiation.  That is,
     * ```
     *     instantiations.get(0).eContainer == instantiations.get(1).reactorClass
     * ```
     * More generally, for all 0 <= i < instantiations.size - 1,
     * ```
     *     instantiations.get(i).eContainer == instantiations.get(i + 1).reactorClass
     * ```
     * If any of these conditions is not satisfied, then an IllegalArgumentException
     * will be thrown.
     * 
     * Note that this chain of reactions cannot be inferred from the parameter because
     * in each of the predicates above, there may be more than one instantiation that
     * can appear on the right hand side of the predicate.
     * 
     * For example, consider the following program:
     * ```
     *     reactor A(x:int(1)) {}
     *     reactor B(y:int(2)) {
     *         a1 = new A(x = y);
     *         a2 = new A(x = -1);
     *     }
     *     reactor C(z:int(3)) {
     *         b1 = new B(y = z);
     *         b2 = new B(y = -2);
     *     }
     * ```
     * Notice that there are a total of four instances of reactor class A.
     * Then
     * ```
     *     initialValue(x, null) returns 1
     *     initialValue(x, [a1]) returns 2
     *     initialValue(x, [a2]) returns -1
     *     initialValue(x, [a1, b1]) returns 3
     *     initialValue(x, [a2, b1]) returns -1
     *     initialValue(x, [a1, b2]) returns -2
     *     initialValue(x, [a2, b2]) returns -1
     * ```
     * (Actually, in each of the above cases, the returned value is a list with
     * one entry, a Literal, e.g. ["1"]).
     * 
     * There are two instances of reactor class B.
     * ```
     *     initialValue(y, null) returns 2
     *     initialValue(y, [a1]) throws an IllegalArgumentException
     *     initialValue(y, [b1]) returns 3
     *     initialValue(y, [b2]) returns -2
     * ```
     * 
     * @param parameter The parameter.
     * @param instantiations The (optional) list of instantiations.
     * 
     * @return The value of the parameter.
     * 
     * @throws IllegalArgumentException If an instantiation provided is not an
     *  instantiation of the reactor class that is parameterized by the
     *  respective parameter or if the chain of instantiations is not nested.
     */
    public static List<Value> initialValue(Parameter parameter, List<Instantiation> instantiations) {
        // If instantiations are given, then check to see whether this parameter gets overridden in
        // the first of those instantiations.
        if (instantiations != null && instantiations.size() > 0) {
            // Check to be sure that the instantiation is in fact an instantiation
            // of the reactor class for which this is a parameter.
            Instantiation instantiation = instantiations.get(0);

            if (!belongsTo(parameter, instantiation)) {
                throw new IllegalArgumentException("Parameter "
                    + parameter.getName()
                    + " is not a parameter of reactor instance "
                    + instantiation.getName()
                    + "."
                );
            }
            // In case there is more than one assignment to this parameter, we need to
            // find the last one.
            Assignment lastAssignment = null;
            for (Assignment assignment: instantiation.getParameters()) {
                if (assignment.getLhs().equals(parameter)) {
                    lastAssignment = assignment;
                }
            }
            if (lastAssignment != null) {
                // Right hand side can be a list. Collect the entries.
                List<Value> result = new ArrayList<>();
                for (Value value: lastAssignment.getRhs()) {
                    if (value.getParameter() != null) {
                        if (instantiations.size() > 1
                            && instantiation.eContainer() != instantiations.get(1).getReactorClass()
                        ) {
                            throw new IllegalArgumentException("Reactor instance "
                                    + instantiation.getName()
                                    + " is not contained by instance "
                                    + instantiations.get(1).getName()
                                    + "."
                            );
                        }
                        result.addAll(initialValue(value.getParameter(), 
                                instantiations.subList(1, instantiations.size())));
                    } else {
                        result.add(value);
                    }
                }
                return result;
            }
        }
        // If we reach here, then either no instantiation was supplied or
        // there was no assignment in the instantiation. So just use the
        // parameter's initial value.
        return parameter.getInit();
    }
    
    /**
     * Return true if the specified object (a Parameter, Port, Action, or Timer)
     * belongs to the specified instantiation, meaning that it is defined in
     * the reactor class being instantiated or one of its base classes.
     * @param eobject The object.
     * @param instantiation The instantiation.
     */
    public static boolean belongsTo(EObject eobject, Instantiation instantiation) {
        Reactor reactor = toDefinition(instantiation.getReactorClass());
        return belongsTo(eobject, reactor);
    }
    
    /**
     * Return true if the specified object (a Parameter, Port, Action, or Timer)
     * belongs to the specified reactor, meaning that it is defined in
     * reactor class or one of its base classes.
     * @param eobject The object.
     * @param reactor The reactor.
     */
    public static boolean belongsTo(EObject eobject, Reactor reactor) {
        if (eobject.eContainer() == reactor) return true;
        for (ReactorDecl baseClass : reactor.getSuperClasses()) {
            if (belongsTo(eobject, toDefinition(baseClass))) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Given a parameter return its integer value or null
     * if it does not have an integer value.
     * If the value of the parameter is a list of integers,
     * return the sum of value in the list.
     * The instantiations parameter is as in
     * {@link #initialValue(Parameter, List)}.
     * 
     * @param parameter The parameter.
     * @param instantiations The (optional) list of instantiations.
     * 
     * @return The integer value of the parameter, or null if does not have an integer value.
     *
     * @throws IllegalArgumentException If an instantiation provided is not an
     *  instantiation of the reactor class that is parameterized by the
     *  respective parameter or if the chain of instantiations is not nested.
     */
    public static Integer initialValueInt(Parameter parameter, List<Instantiation> instantiations) {
        List<Value> values = initialValue(parameter, instantiations);
        int result = 0;
        for (Value value: values) {
            if (value.getLiteral() == null) { 
                return null;
            }
            try {
                result += Integer.decode(value.getLiteral());
            } catch (NumberFormatException ex) {
                return null;
            }
        }
        return result;
    }
    
    /**
     * Given the width specification of port or instantiation
     * and an (optional) list of nested instantiations, return
     * the width if it can be determined and -1 if not.
     * It will not be able to be determined if either the
     * width is variable (in which case you should use
     * {@link #inferPortWidth(VarRef, Connection, List)} )
     * or the list of instantiations is incomplete or missing.
     * If there are parameter references in the width, they are
     * evaluated to the extent possible given the instantiations list.
     * 
     * The instantiations list is as in 
     * {@link #initialValue(Parameter, List)}.
     * If the spec belongs to an instantiation (for a bank of reactors),
     * then the first element on this list should be the instantiation
     * that contains this instantiation. If the spec belongs to a port,
     * then the first element on the list should be the instantiation
     * of the reactor that contains the port.
     *
     * @param spec The width specification or null (to return 1).
     * @param instantiations The (optional) list of instantiations.
     * 
     * @return The width, or -1 if the width could not be determined.
     *
     * @throws IllegalArgumentException If an instantiation provided is not as
     *  given above or if the chain of instantiations is not nested.
     */
    public static int width(WidthSpec spec, List<Instantiation> instantiations) {
        if (spec == null) {
            return 1;
        }
        if (spec.isOfVariableLength() && spec.eContainer() instanceof Instantiation) {
            // We may be able to infer the width by examining the connections of
            // the enclosing reactor definition. This works, for example, with
            // delays between multiports or banks of reactors.
            // Attempt to infer the width.
            for (Connection c : ((Reactor) spec.eContainer().eContainer()).getConnections()) {
                int leftWidth = 0;
                int rightWidth = 0;
                int leftOrRight = 0;
                for (VarRef leftPort : c.getLeftPorts()) {
                    if (leftPort.getContainer() == spec.eContainer()) {
                        if (leftOrRight != 0) {
                            throw new InvalidSourceException("Multiple ports with variable width on a connection.");
                        }
                        // Indicate that the port is on the left.
                        leftOrRight = -1;
                    } else {
                        leftWidth += inferPortWidth(leftPort, c, instantiations);
                    }
                }
                for (VarRef rightPort : c.getRightPorts()) {
                    if (rightPort.getContainer() == spec.eContainer()) {
                        if (leftOrRight != 0) {
                            throw new InvalidSourceException("Multiple ports with variable width on a connection.");
                        }
                        // Indicate that the port is on the right.
                        leftOrRight = 1;
                    } else {
                        rightWidth += inferPortWidth(rightPort, c, instantiations);
                    }
                }
                if (leftOrRight < 0) {
                    return rightWidth - leftWidth;
                } else if (leftOrRight > 0) {
                    return leftWidth - rightWidth;
                }
            }
            // A connection was not found with the instantiation.
            return -1;
        }
        var result = 0;
        for (WidthTerm term: spec.getTerms()) {
            if (term.getParameter() != null) {
                Integer termWidth = initialValueInt(term.getParameter(), instantiations);
                if (termWidth != null) {
                    result += termWidth;
                } else {
                    return -1;
                }
            } else if (term.getWidth() > 0) {
                result += term.getWidth();
            } else {
                return -1;
            }
        }
        return result;
    }

    /**
     * Infer the width of a port reference in a connection.
     * The port reference one or two parts, a port and an (optional) container
     * which is an Instantiation that may refer to a bank of reactors.
     * The width will be the product of the bank width and the port width.
     * The returned value will be 1 if the port is not in a bank and is not a multiport.
     * 
     * If the width cannot be determined, this will return -1.
     * The width cannot be determined if the list of instantiations is
     * missing or incomplete.
     * 
     * The instantiations list is as in 
     * {@link #initialValue(Parameter, List)}.
     * The first element on this list should be the instantiation
     * that contains the specified connection.
     *
     * @param reference A port reference.
     * @param connection A connection, or null if not in the context of a connection.
     * @param instantiations The (optional) list of instantiations.
     * 
     * @return The width or -1 if it could not be determined.
     *
     * @throws IllegalArgumentException If an instantiation provided is not as
     *  given above or if the chain of instantiations is not nested.
     */
    public static int inferPortWidth(
        VarRef reference, Connection connection, List<Instantiation> instantiations
    ) {
        if (reference.getVariable() instanceof Port) {
            // If the port is given as a.b, then we want to prepend a to
            // the list of instantiations to determine the width of this port.
            List<Instantiation> extended = instantiations;
            if (reference.getContainer() != null) {
                extended = new ArrayList<>();
                extended.add(reference.getContainer());
                if (instantiations != null) {
                    extended.addAll(instantiations);
                }
            }

            int portWidth = width(((Port) reference.getVariable()).getWidthSpec(), extended);
            if (portWidth < 0) { 
                // Could not determine port width.
                return -1; 
            }
            
            // Next determine the bank width. This may be unspecified, in which
            // case it has to be inferred using the connection.
            int bankWidth = 1;
            if (reference.getContainer() != null) {
                bankWidth = width(reference.getContainer().getWidthSpec(), instantiations);
                if (bankWidth < 0 && connection != null) {
                    // Try to infer the bank width from the connection.
                    if (reference.getContainer().getWidthSpec().isOfVariableLength()) {
                        // This occurs for a bank of delays.
                        int leftWidth = 0;
                        int rightWidth = 0;
                        int leftOrRight = 0;
                        for (VarRef leftPort : connection.getLeftPorts()) {
                            if (leftPort == reference) {
                                if (leftOrRight != 0) {
                                    throw new InvalidSourceException("Multiple ports with variable width on a connection.");
                                }
                                // Indicate that this port is on the left.
                                leftOrRight = -1;
                            } else {
                                // The left port is not the same as this reference.
                                int otherWidth = inferPortWidth(leftPort, connection, instantiations);
                                if (otherWidth < 0) {
                                    // Cannot determine width.
                                    return -1;
                                }
                                leftWidth += otherWidth;
                            }
                        }
                        for (VarRef rightPort : connection.getRightPorts()) {
                            if (rightPort == reference) {
                                if (leftOrRight != 0) {
                                    throw new InvalidSourceException("Multiple ports with variable width on a connection.");
                                }
                                // Indicate that this port is on the right.
                                leftOrRight = 1;
                            } else {
                                int otherWidth = inferPortWidth(rightPort, connection, instantiations);
                                if (otherWidth < 0) {
                                    // Cannot determine width.
                                    return -1;
                                }
                                rightWidth += otherWidth;
                            }
                        }
                        int discrepancy = 0;
                        if (leftOrRight < 0) {
                            // This port is on the left.
                            discrepancy = rightWidth - leftWidth;
                        } else if (leftOrRight > 0) {
                            // This port is on the right.
                            discrepancy = leftWidth - rightWidth;
                        }
                        // Check that portWidth divides the discrepancy.
                        if (discrepancy % portWidth != 0) {
                            // This is an error.
                            return -1; 
                        }
                        bankWidth = discrepancy / portWidth;
                    } else {
                        // Could not determine the bank width.
                        return -1; 
                    }
                }
            }
            return portWidth * bankWidth;
        }
        // Argument is not a port.
        return -1;
    }
    
    /**
     * Given an instantiation of a reactor or bank of reactors, return
     * the width. This will be 1 if this is not a reactor bank. Otherwise,
     * this will attempt to determine the width. If the width is declared
     * as a literal constant, it will return that constant. If the width
     * is specified as a reference to a parameter, this will throw an
     * exception. If the width is variable, this will find
     * connections in the enclosing reactor and attempt to infer the
     * width. If the width cannot be determined, it will throw an exception.
     *
     * IMPORTANT: This method should not be used you really need to
     * determine the width! It will not evaluate parameter values.
     * @see #width(WidthSpec, List)
     *
     * @param instantiation A reactor instantiation.
     * 
     * @return The width, if it can be determined.
     * @deprecated
     */
    @Deprecated
    public static int widthSpecification(Instantiation instantiation) {
        int result = width(instantiation.getWidthSpec(), null);
        if (result < 0) {
            throw new InvalidSourceException("Cannot determine width for the instance "
                    + instantiation.getName());
        }
        return result;
    }

    /**
     * Report whether a state variable has been initialized or not.
     * @param v The state variable to be checked.
     * @return True if the variable was initialized, false otherwise.
     */
    public static boolean isInitialized(StateVar v) {
        return v != null && (v.getParens().size() == 2 || v.getBraces().size() == 2);
    }

    /**
     * Report whether the given time state variable is initialized using a 
     * parameter or not.
     * @param s A state variable.
     * @return True if the argument is initialized using a parameter, false 
     * otherwise.
     */
    public static boolean isParameterized(StateVar s) {
        return s.getInit() != null && 
               IterableExtensions.exists(s.getInit(), it -> it.getParameter() != null);
    }

    /**
     * Check if the reactor class uses generics
     * @param r the reactor to check 
     * @return true if the reactor uses generics
     */
    public static boolean isGeneric(Reactor r) {
        if (r == null) {
            return false;
        }
        return r.getTypeParms().size() != 0;
    }
    
    /**
     * If the specified reactor declaration is an import, then
     * return the imported reactor class definition. Otherwise,
     * just return the argument.
     * @param r A Reactor or an ImportedReactor.
     * @return The Reactor class definition or null if no definition is found.
     */
    public static Reactor toDefinition(ReactorDecl r) {
        if (r == null)
            return null;
        if (r instanceof Reactor) {
            return (Reactor) r;
        } else if (r instanceof ImportedReactor) {
            return ((ImportedReactor) r).getReactorClass();
        }
        return null;
    }
    
    /**
     * Retrieve a specific annotation in a JavaDoc style comment associated with the given model element in the AST.
     * 
     * This will look for a JavaDoc style comment. If one is found, it searches for the given annotation `key`.
     * and extracts any string that follows the annotation marker.  
     * 
     * @param object the AST model element to search a comment for
     * @param key the specific annotation key to be extracted
     * @return `null` if no JavaDoc style comment was found or if it does not contain the given key.
     *     The string immediately following the annotation marker otherwise.
     */
    public static String findAnnotationInComments(EObject object, String key) {
        if (object.eResource() instanceof XtextResource) {
            ICompositeNode compNode = NodeModelUtils.findActualNodeFor(object);
            if (compNode != null) {
                // Find comment node in AST
                // For reactions/timers/action/etc., it is usually the lowermost first child node
                INode node = compNode.getFirstChild();
                while (node instanceof CompositeNode) {
                    node = ((CompositeNode) node).getFirstChild();
                }
                // For reactors, it seems to be the next sibling of the first child node
                if (node == null && compNode.getFirstChild() != null) {
                    node = compNode.getFirstChild().getNextSibling();
                }
                while (node instanceof HiddenLeafNode) { // Only comments preceding start of element
                    HiddenLeafNode hlNode = (HiddenLeafNode) node;
                    EObject rule = hlNode.getGrammarElement();
                    if (rule instanceof TerminalRule) {
                        String line = null;
                        TerminalRule tRule = (TerminalRule) rule;
                        if ("SL_COMMENT".equals(tRule.getName())) {
                            if (hlNode.getText().contains(key)) {
                                line = hlNode.getText();
                            }
                        } else if ("ML_COMMENT".equals(tRule.getName())) {
                            boolean found = false;
                            for (String str : hlNode.getText().split("\n")) {
                                if (!found && str.contains(key)) {
                                    line = str;
                                }
                            }
                            // This is shorter but causes a warning:
                            //line = node.text.split("\n").filterNull.findFirst[it.contains(key)]
                        }
                        if (line != null) {
                            var value = line.substring(line.indexOf(key) + key.length()).trim();
                            if (value.contains("*")) { // in case of single line block comment (e.g. /** @anno 1503 */)
                                value = value.substring(0, value.indexOf("*")).trim();
                            }
                            return value;
                        }
                    }
                    node = node.getNextSibling();
                }
            }
        }
        return null;
    }
    
    /**
     * Remove quotation marks surrounding the specified string.
     */
    public static String withoutQuotes(String s) {
        String result = s;
        if (s.startsWith("\"") || s.startsWith("'")) {
            result = s.substring(1);
        }
        if (result.endsWith("\"") || result.endsWith("'")) {
            result = result.substring(0, result.length() - 1);
        }
        return result;
    }
    
    /**
     * Search for an `@label` annotation for a given reaction.
     * 
     * @param n the reaction for which the label should be searched
     * @return The annotated string if an `@label` annotation was found. `null` otherwise.
     */
    public static String label(Reaction n) {
        return findAnnotationInComments(n, "@label");
    }
    
    /**
     * Find the main reactor and set its name if none was defined.
     * @param resource The resource to find the main reactor in.
     */
    public static void setMainName(Resource resource, String name) {
        Reactor main = IteratorExtensions.findFirst(Iterators.filter(resource.getAllContents(), Reactor.class), 
            it -> it.isMain() || it.isFederated()
        );
        if (main != null && StringExtensions.isNullOrEmpty(main.getName())) {
            main.setName(name);
        }
    }
    
    /**
     * Create a new instantiation node with the given reactor as its defining class.
     * @param reactor The reactor class to create an instantiation of.
     */
    public static Instantiation createInstantiation(Reactor reactor) {
        Instantiation inst = LfFactory.eINSTANCE.createInstantiation();
        inst.setReactorClass(reactor);
        // If the reactor is federated or at the top level, then it
        // may not have a name. In the generator's doGenerate()
        // method, the name gets set using setMainName().
        // But this may be called before that, e.g. during
        // diagram synthesis.  We assign a temporary name here.
        if (reactor.getName() == null) {
            if (reactor.isFederated() || reactor.isMain()) {
                inst.setName("main");
            } else {
                inst.setName("");
            }
            
        } else {
            inst.setName(reactor.getName());
        }
        return inst;
    }

    /**
     * Returns the target declaration in the given model.
     * Non-null because it would cause a parse error.
     */
    public static TargetDecl targetDecl(Model model) {
        return IteratorExtensions.head(Iterators.filter(model.eAllContents(), TargetDecl.class));
    }

    /**
     * Returns the target declaration in the given resource.
     * Non-null because it would cause a parse error.
     */
    public static TargetDecl targetDecl(Resource model) {
        return IteratorExtensions.head(Iterators.filter(model.getAllContents(), TargetDecl.class));
    }
    
    /////////////////////////////////////////////////////////
    //// Private methods
    
    /**
     * Returns the list if it is not null. Otherwise return an empty list.
     */
    public static <T> List<T> convertToEmptyListIfNull(List<T> list) {
        return list != null ? list : new ArrayList<>();
    }

    /**
     * Return all the superclasses of the specified reactor
     * in deepest-first order. For example, if A extends B and C, and
     * B and C both extend D, this will return the list [D, B, C, A].
     * Duplicates are removed. If the specified reactor does not extend
     * any other reactor, then return an empty list.
     * If a cycle is found, where X extends Y and Y extends X, or if
     * a superclass is declared that is not found, then return null.
     * @param reactor The specified reactor.
     * @param extensions A set of reactors extending the specified reactor
     *  (used to detect circular extensions).
     */
    private static LinkedHashSet<Reactor> superClasses(Reactor reactor, Set<Reactor> extensions) {
        LinkedHashSet<Reactor> result = new LinkedHashSet<>();
        for (ReactorDecl superDecl : convertToEmptyListIfNull(reactor.getSuperClasses())) {
            Reactor r = toDefinition(superDecl);
            if (r == reactor || r == null) return null;
            // If r is in the extensions, then we have a circular inheritance structure.
            if (extensions.contains(r)) return null;
            extensions.add(r);
            LinkedHashSet<Reactor> baseExtends = superClasses(r, extensions);
            extensions.remove(r);
            if (baseExtends == null) return null;
            result.addAll(baseExtends);
            result.add(r);
        }
        return result;
    }
}
