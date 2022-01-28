/* A helper class for analyzing the AST. */

/*************
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
***************/

package org.lflang;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;

import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.TerminalRule;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.impl.CompositeNode;
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.xbase.lib.CollectionExtensions;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.CodeMap;
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
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;

import static org.eclipse.emf.ecore.util.EcoreUtil.*;
import static org.lflang.JavaAstUtils.*;

/**
 * A helper class for modifying and analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
public class ASTUtils {
    
    /**
     * The Lingua Franca factory for creating new AST nodes.
     */
    public static final LfFactory factory = LfFactory.eINSTANCE;
    
    /**
     * Find connections in the given resource that have a delay associated with them, 
     * and reroute them via a generated delay reactor.
     * @param resource The AST.
     * @param generator A code generator.
     */
    public static void insertGeneratedDelays(Resource resource, GeneratorBase generator) {
        // The resulting changes to the AST are performed _after_ iterating 
        // in order to avoid concurrent modification problems.
        ArrayList<Connection> oldConnections = new ArrayList<Connection>();
        LinkedHashMap<Reactor, List<Connection>> newConnections = new LinkedHashMap<Reactor, List<Connection>>();
        LinkedHashMap<Reactor, List<Instantiation>> delayInstances = new LinkedHashMap<Reactor, List<Instantiation>>();
        Iterable<Reactor> containers = Iterables.<Reactor>filter(IteratorExtensions.<EObject>toIterable(resource.getAllContents()), Reactor.class);
        
        // Iterate over the connections in the tree.
        for (Reactor container : containers) {
            for (Connection connection : container.getConnections()) {
                if (connection.getDelay() != null) { 
                    Reactor parent = (Reactor) connection.eContainer();
                    // Assume all the types are the same, so just use the first on the right.
                    Type type = ((Port) connection.getRightPorts().get(0).getVariable()).getType();
                    Reactor delayClass = getDelayClass(type, generator);
                    String generic = generator.getTargetTypes().supportsGenerics() ? generator.getTargetTypes().getTargetType(InferredType.fromAST(type)) : "";
                    Instantiation delayInstance = ASTUtils.getDelayInstance(delayClass, connection, generic, 
                        !generator.generateAfterDelaysWithVariableWidth());

                    // Stage the new connections for insertion into the tree.
                    List<Connection> connections = newConnections.get(parent) != null ? newConnections.get(parent) : new ArrayList<>();
                    connections.addAll(rerouteViaDelay(connection, delayInstance));
                    newConnections.put(parent, connections);
                    // Stage the original connection for deletion from the tree.
                    oldConnections.add(connection);

                    // Stage the newly created delay reactor instance for insertion
                    List<Instantiation> instances = delayInstances.get(parent) != null ? delayInstances.get(parent) : new ArrayList<>();
                    CollectionExtensions.<Instantiation>addAll(instances, delayInstance);
                    delayInstances.put(parent, instances);
                }
            }
        }

        // Remove old connections; insert new ones.
        oldConnections.forEach(connection -> ((Reactor) connection.eContainer()).getConnections().remove(connection));
        newConnections.forEach((reactor, connections) -> reactor.getConnections().addAll(connections));
        // Finally, insert the instances and, before doing so, assign them a unique name.
        delayInstances.forEach((reactor, instantiations) -> 
            instantiations.forEach(instantiation -> {
                instantiation.setName(getUniqueIdentifier(reactor, "delay"));
                reactor.getInstantiations().add(instantiation);
            })
        );
    }
    
    /**
     * Find the main reactor and change it to a federated reactor.
     * Return true if the transformation was successful (or the given resource
     * already had a federated reactor); return false otherwise.
     */
    public static boolean makeFederated(Resource resource) {
        Reactor r = IteratorExtensions.<Reactor>findFirst(
            Iterators.<Reactor>filter(resource.getAllContents(), Reactor.class), 
            it -> { return it.isMain(); }
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
        TargetDecl r = ASTUtils.targetDecl(resource);
        r.setName(newTargetName);
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
        VarRef leftPort = connection.getLeftPorts().get(0);
        VarRef rightPort = connection.getRightPorts().get(0);
        Instantiation leftContainer = leftPort.getContainer();
        Instantiation rightContainer = rightPort.getContainer();
        if (((Port) leftPort.getVariable()).getWidthSpec()  != null || (leftContainer  != null && leftContainer.getWidthSpec()  != null) ||
            ((Port) rightPort.getVariable()).getWidthSpec() != null || (rightContainer != null && rightContainer.getWidthSpec() != null)) {
            return true;
        }
        return false;
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
        List<Connection> connections = new ArrayList<Connection>();    
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
                    term.setPort(EcoreUtil.<VarRef>copy(port));
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
        if ((classDef != null)) {
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
            action.setType(EcoreUtil.<Type>copy(type));
        }

        input.setName("inp");
        input.setType(EcoreUtil.<Type>copy(action.getType()));

        output.setName("out");
        output.setType(EcoreUtil.<Type>copy(action.getType()));

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
            if (IterableExtensions.<String>exists(vars, it -> { return it.equals(id); })) {
                suffix = ("_" + index);
                index++;
            } else {
                exists = false;
            }
        }
        return name + suffix;
    }
   
    ////////////////////////////////
    //// Utility functions for supporting inheritance
    
    /**
     * Given a reactor class, return a list of all its actions,
     * which includes actions of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Action> allActions(Reactor definition) {
        List<Action> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allActions(toDefinition(base)));
        }
        result.addAll(definition.getActions());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its connections,
     * which includes connections of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Connection> allConnections(Reactor definition) {
        List<Connection> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allConnections(toDefinition(base)));
        }
        result.addAll(definition.getConnections());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its inputs,
     * which includes inputs of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Input> allInputs(Reactor definition) {
        List<Input> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allInputs(toDefinition(base)));
        }
        result.addAll(definition.getInputs());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its instantiations,
     * which includes instantiations of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Instantiation> allInstantiations(Reactor definition) {
        List<Instantiation> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allInstantiations(toDefinition(base)));
        }
        result.addAll(definition.getInstantiations());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its outputs,
     * which includes outputs of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Output> allOutputs(Reactor definition) {
        List<Output> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allOutputs(toDefinition(base)));
        }
        result.addAll(definition.getOutputs());
        return result;
    }

    /**
     * Given a reactor class, return a list of all its parameters,
     * which includes parameters of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Parameter> allParameters(Reactor definition) {
        List<Parameter> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allParameters(toDefinition(base)));
        }
        result.addAll(definition.getParameters());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its reactions,
     * which includes reactions of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Reaction> allReactions(Reactor definition) {
        List<Reaction> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allReactions(toDefinition(base)));
        }
        result.addAll(definition.getReactions());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its state variables,
     * which includes state variables of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<StateVar> allStateVars(Reactor definition) {
        List<StateVar> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allStateVars(toDefinition(base)));
        }
        result.addAll(definition.getStateVars());
        return result;
    }
    
    /**
     * Given a reactor class, return a list of all its timers,
     * which includes timers of base classes that it extends.
     * @param definition Reactor class definition.
     */
    public static List<Timer> allTimers(Reactor definition) {
        List<Timer> result = new ArrayList<>();
        List<ReactorDecl> superClasses = definition.getSuperClasses() != null ? definition.getSuperClasses() : new ArrayList<>();
        for (ReactorDecl base : superClasses) {
            result.addAll(allTimers(toDefinition(base)));
        }
        result.addAll(definition.getTimers());
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
                str = str.substring((start + 2), end);
                if (str.split("\n").length > 1) {
                    // multi line code
                    text = trimCodeBlock(str);
                } else {
                    // single line code
                    text = str.trim();
                }
            } else if (code.getBody() != null) {
                // Code must have been added as a simple string.
                text = code.getBody().toString();
            }
        }
        return text;
    }
    
    public static String toText(TypeParm t) {
        return !StringExtensions.isNullOrEmpty(t.getLiteral()) ? t.getLiteral() : toText(t.getCode());
    }
    
    /**
     * Intelligently trim the white space in a code block.
	 * 
	 * The leading whitespaces of the first non-empty
	 * code line is considered as a common prefix across all code lines. If the
	 * remaining code lines indeed start with this prefix, it removes the prefix
	 * from the code line.
	 * 
     * For examples, this code
     * <pre>{@code 
     *        int test = 4;
     *        if (test == 42) {
     *            printf("Hello\n");
     *        }
     * }</pre>
     * will be trimmed to this:
     * <pre>{@code 
     * int test = 4;
     * if (test == 42) {
     *     printf("Hello\n");
     * }
     * }</pre>
     * 
     * In addition, if the very first line has whitespace only, then
     * that line is removed. This just means that the {= delimiter
     * is followed by a newline.
     * 
     * @param code the code block to be trimmed
     * @return trimmed code block 
     */
    public static String trimCodeBlock(String code) {
        String[] codeLines = code.split("\n");
        String prefix = null;
        StringBuilder buffer = new StringBuilder();
        for (String line : codeLines) {
            if (prefix == null) {
                if (line.trim().length() > 0) {
                    // this is the first code line
                    // find the index of the first code line
                    char[] characters = line.toCharArray();
                    boolean foundFirstCharacter = false;
                    int firstCharacter = 0;
                    for (var i = 0; i < characters.length; i++) {
                        if (!foundFirstCharacter && !Character.isWhitespace(characters[i])) {
                            foundFirstCharacter = true;
                            firstCharacter = i;
                        }
                    }
                    // extract the whitespace prefix
                    prefix = line.substring(0, firstCharacter);
                }
            }

            // try to remove the prefix from all subsequent lines
            if (prefix != null) {
                if (line.startsWith(prefix)) {
                    buffer.append(line.substring(prefix.length()));
                    buffer.append("\n");
                } else {
                    buffer.append(line);
                    buffer.append("\n");
                }
            }
        }
        if (buffer.length() > 1) {
        	buffer.deleteCharAt(buffer.length() - 1); // remove the last newline
        } 
        return buffer.toString();
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
        return JavaAstUtils.toTimeValue(t).toString();
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
                    String stars = "";
                    List<String> iterList = type.getStars() != null ? type.getStars() : new ArrayList<>();
                    for (String s : iterList) {
                        stars += s;
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
     * @param literalOrCode AST node to inspect.
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
        if (code != null && isZero(toUntaggedText(code))) {
            return true;
        }
        return false;
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
                if (JavaAstUtils.isOfTimeType(value.getParameter())) {
                    return true;
                }
            } else if (value.getTime() != null) {
                return isValidTime(value.getTime());
            } else if (value.getLiteral() != null) {
                if (isZero(value.getLiteral())) {
                    return true;
                }
            } else if (value.getCode() != null) {
                if (isZero(value.getCode())) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Report whether the given time denotes a valid time or not.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    public static boolean isValidTime(Time t) {
        if (t != null && t.getUnit() != null) {
            return true;
        }
        return false;
    }

	/**
     * Report whether the given parameter denotes time list, meaning it is a list
     * of which all elements are valid times.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time list, false otherwise.
     */
    // TODO: why does this function always return true ???
	public static boolean isValidTimeList(Parameter p) {
        if (p != null) {
            if (p.getType() != null && p.getType().isTime() && p.getType().getArraySpec() != null) {
                return true;
            } else if (p.getInit() != null && 
                       p.getInit().size() > 1 && 
                       IterableExtensions.<Value>forall(p.getInit(), it -> { return isValidTime(it); })) {
                return true;
            }
        }
        return true;
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
     * @param instantiation The (optional) instantiation.
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
            Assignment lastAssignment = (Assignment) null;
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
     * @param instnatiation The instantiation.
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
     * @param instnatiation The instantiation.
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
     * {@link initialValue(Parameter, List<Instantiation>)}.
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
     * {@link inferPortWidth(VarRef, Connection, List<Instantiation>})
     * or the list of instantiations is incomplete or missing.
     * If there are parameter references in the width, they are
     * evaluated to the extent possible given the instantiations list.
     * 
     * The instantiations list is as in 
     * {@link initialValue(Parameter, List<Instantiation>)}.
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
     * {@link initialValue(Parameter, List<Instantiation>}.
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
    //     if (reference.variable instanceof Port) {
    //         // If the port is given as a.b, then we want to prepend a to
    //         // the list of instantiations to determine the width of this port.
    //         var extended = instantiations;
    //         if (reference.container !== null) {
    //             extended = new ArrayList<Instantiation>();
    //             extended.add(reference.container);
    //             if (instantiations !== null) {
    //                 extended.addAll(instantiations);
    //             }
    //         }

    //         val portWidth = width((reference.variable as Port).widthSpec, extended)
    //         if (portWidth < 0) return -1; // Could not determine port width.
            
    //         // Next determine the bank width. This may be unspecified, in which
    //         // case it has to be inferred using the connection.
    //         var bankWidth = 1
    //         if (reference.container !== null) {
    //             bankWidth = width(reference.container.widthSpec, instantiations)
    //             if (bankWidth < 0 && connection !== null) {
    //                 // Try to infer the bank width from the connection.
    //                 if (reference.container.widthSpec.isOfVariableLength) {
    //                     // This occurs for a bank of delays.
    //                     var leftWidth = 0
    //                     var rightWidth = 0
    //                     var leftOrRight = 0
    //                     for (leftPort : connection.leftPorts) {
    //                         if (leftPort === reference) {
    //                             if (leftOrRight !== 0) {
    //                                 throw new InvalidSourceException("Multiple ports with variable width on a connection.")
    //                             }
    //                             // Indicate that this port is on the left.
    //                             leftOrRight = -1
    //                         } else {
    //                             // The left port is not the same as this reference.
    //                             val otherWidth = inferPortWidth(leftPort, connection, instantiations)
    //                             if (otherWidth < 0) return -1; // Cannot determine width.
    //                             leftWidth += otherWidth;
    //                         }
    //                     }
    //                     for (rightPort : connection.rightPorts) {
    //                         if (rightPort === reference) {
    //                             if (leftOrRight !== 0) {
    //                                 throw new InvalidSourceException("Multiple ports with variable width on a connection.")
    //                             }
    //                             // Indicate that this port is on the right.
    //                             leftOrRight = 1
    //                         } else {
    //                             val otherWidth = inferPortWidth(rightPort, connection, instantiations)
    //                             if (otherWidth < 0) return -1; // Cannot determine width.
    //                             rightWidth += otherWidth
    //                         }
    //                     }
    //                     var discrepancy = 0;
    //                     if (leftOrRight < 0) {
    //                         // This port is on the left.
    //                         discrepancy = rightWidth - leftWidth
    //                     } else if (leftOrRight > 0) {
    //                         // This port is on the right.
    //                         discrepancy = leftWidth - rightWidth
    //                     }
    //                     // Check that portWidth divides the discrepancy.
    //                     if (discrepancy % portWidth != 0) {
    //                         return -1; // This is an error.
    //                     }
    //                     bankWidth = discrepancy / portWidth;
    //                 } else {
    //                     return -1; // Could not determine the bank width.
    //                 }
    //             }
    //         }
    //         return portWidth * bankWidth
    //     }
    //     // Argument is not a port.
    //     return -1;
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
     * @see width(WidthSpec, List<Instantiation> instantiations)
     *
     * @param instantiation A reactor instantiation.
     * 
     * @return The width, if it can be determined.
     * @deprecated
     */
    public static int widthSpecification(Instantiation instantiation) {
    //     val result = width(instantiation.widthSpec, null);
    //     if (result < 0) {
    //         throw new InvalidSourceException("Cannot determine width for the instance "
    //                 + instantiation.name);
    //     }
    //     return result
    }

    /**
     * Report whether a state variable has been initialized or not.
     * @param v The state variable to be checked.
     * @return True if the variable was initialized, false otherwise.
     */
    public static boolean isInitialized(StateVar v) {
    //     if (v !== null && (v.parens.size == 2 || v.braces.size == 2)) {
    //         return true
    //     }
    //     return false
    }
        
    /**
     * Report whether the given time state variable is initialized using a 
     * parameter or not.
     * @param s A state variable.
     * @return True if the argument is initialized using a parameter, false 
     * otherwise.
     */
    public static boolean isParameterized(StateVar s) {
    //     if (s.init !== null && s.init.exists[it.parameter !== null]) {
    //         return true
    //     }
    //     return false
    }

    /**
     * Check if the reactor class uses generics
     * @param r the reactor to check 
     * @true true if the reactor uses generics
     */
    public static boolean isGeneric(Reactor r) {
    //     var Reactor defn = r
        
    //     return defn?.typeParms.length != 0;
    }
    
    /**
     * If the specified reactor declaration is an import, then
     * return the imported reactor class definition. Otherwise,
     * just return the argument.
     * @param r A Reactor or an ImportedReactor.
     * @return The Reactor class definition.
     */
    public static Reactor toDefinition(ReactorDecl r) {
    //     if (r === null)
    //         return null
    //     if (r instanceof Reactor) {
    //         return r
    //     } else if (r instanceof ImportedReactor) {
    //         return r.reactorClass
    //     }
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
    //     if (object.eResource instanceof XtextResource) {
    //         val compNode = NodeModelUtils.findActualNodeFor(object)
    //         if (compNode !== null) {
    //             // Find comment node in AST
    //             // For reactions/timers/action/etc., it is usually the lowermost first child node
    //             var node = compNode.firstChild
    //             while (node instanceof CompositeNode) {
    //                 node = node.firstChild
    //             }
    //             // For reactors, it seems to be the next sibling of the first child node
    //             if (node === null && compNode.firstChild !== null) {
    //                 node = compNode.firstChild.nextSibling
    //             }
    //             while (node instanceof HiddenLeafNode) { // Only comments preceding start of element
    //                 val rule = node.grammarElement
    //                 if (rule instanceof TerminalRule) {
    //                     var String line;
    //                     if ("SL_COMMENT".equals(rule.name)) {
    //                         if (node.text.contains(key)) {
    //                             line = node.text
    //                         }
    //                     } else if ("ML_COMMENT".equals(rule.name)) {
    //                         var found = false
    //                         for (str : node.text.split("\n")) {
    //                             if (!found && str.contains(key)) {
    //                                 line = str
    //                             }
    //                         }
    //                         // This is shorter but causes a warning:
    //                         //line = node.text.split("\n").filterNull.findFirst[it.contains(key)]
    //                     }
    //                     if (line !== null) {
    //                         var value = line.substring(line.indexOf(key) + key.length).trim()
    //                         if (value.contains("*")) { // in case of single line block comment (e.g. /** @anno 1503 */)
    //                             value = value.substring(0, value.indexOf("*")).trim()
    //                         }
    //                         return value
    //                     }
    //                 }
    //                 node = node.nextSibling
    //             }
    //         }
    //     }
    //     return null
    }
    
    /**
     * Remove quotation marks surrounding the specified string.
     */
    public static String withoutQuotes(String s) {
    //     var result = s
    //     if (s.startsWith("\"") || s.startsWith("\'")) {
    //         result = s.substring(1)
    //     }
    //     if (result.endsWith("\"") || result.endsWith("\'")) {
    //         result = result.substring(0, result.length - 1)
    //     }
    //     result
    }
    
    /**
     * Search for an `@label` annotation for a given reaction.
     * 
     * @param n the reaction for which the label should be searched
     * @return The annotated string if an `@label` annotation was found. `null` otherwise.
     */
    public static String label(Reaction n) {
    //     return n.findAnnotationInComments("@label")
    }
    
    /**
     * Find the main reactor and set its name if none was defined.
     * @param resource The resource to find the main reactor in.
     */
    public static void setMainName(Resource resource, String name) {
    //     val main = resource.allContents.filter(Reactor).findFirst[it.isMain || it.isFederated]
    //     if (main !== null && main.name.isNullOrEmpty) {
    //         main.name = name
    //     }
    }
    
    /**
     * Create a new instantiation node with the given reactor as its defining class.
     * @param reactor The reactor class to create an instantiation of.
     */
    public static Instantiation createInstantiation(Reactor reactor) {
    //     val inst = LfFactory.eINSTANCE.createInstantiation
    //     inst.reactorClass = reactor
    //     // If the reactor is federated or at the top level, then it
    //     // may not have a name. In the generator's doGenerate()
    //     // method, the name gets set using setMainName().
    //     // But this may be called before that, e.g. during
    //     // diagram synthesis.  We assign a temporary name here.
    //     if (reactor.name === null) {
    //         if (reactor.isFederated || reactor.isMain) {
    //             inst.setName("main")    
    //         } else {
    //             inst.setName("")
    //         }
            
    //     } else {
    //         inst.setName(reactor.name)
    //     }
    //     return inst
    }

    /**
     * Returns the target declaration in the given model.
     * Non-null because it would cause a parse error.
     */
    public static TargetDecl targetDecl(Model model) {
    //     return model.eAllContents.filter(TargetDecl).head
    }

    /**
     * Returns the target declaration in the given resource.
     * Non-null because it would cause a parse error.
     */
    public static TargetDecl targetDecl(Resource model) {
    //     return model.allContents.filter(TargetDecl).head
    }

}
