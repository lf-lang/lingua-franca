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

package org.lflang.ast;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.TerminalRule;
import org.eclipse.xtext.impl.ParserRuleImpl;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.util.Pair;
import org.eclipse.xtext.util.Tuples;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.CodeMap;
import org.lflang.generator.InvalidSourceException;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Assignment;
import org.lflang.lf.Code;
import org.lflang.lf.Connection;
import org.lflang.lf.Element;
import org.lflang.lf.Expression;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Literal;
import org.lflang.lf.Method;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.Time;
import org.lflang.lf.Timer;
import org.lflang.lf.Type;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.util.StringUtil;

/**
 * A helper class for modifying and analyzing the AST.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @author Christian Menard
 */
public class ASTUtils {

  /** The Lingua Franca factory for creating new AST nodes. */
  public static final LfFactory factory = LfFactory.eINSTANCE;

  /** The Lingua Franca feature package. */
  public static final LfPackage featurePackage = LfPackage.eINSTANCE;

  /* Match an abbreviated form of a float literal. */
  private static final Pattern ABBREVIATED_FLOAT = Pattern.compile("[+\\-]?\\.\\d+[\\deE+\\-]*");

  /**
   * A mapping from Reactor features to corresponding Mode features for collecting contained
   * elements.
   */
  private static final Map<EStructuralFeature, EStructuralFeature> reactorModeFeatureMap =
      Map.of(
          featurePackage.getReactor_Actions(), featurePackage.getMode_Actions(),
          featurePackage.getReactor_Connections(), featurePackage.getMode_Connections(),
          featurePackage.getReactor_Instantiations(), featurePackage.getMode_Instantiations(),
          featurePackage.getReactor_Reactions(), featurePackage.getMode_Reactions(),
          featurePackage.getReactor_StateVars(), featurePackage.getMode_StateVars(),
          featurePackage.getReactor_Timers(), featurePackage.getMode_Timers());

  /**
   * Get all reactors defined in the given resource.
   *
   * @param resource the resource to extract reactors from
   * @return An iterable over all reactors found in the resource
   */
  public static List<Reactor> getAllReactors(Resource resource) {
    return StreamSupport.stream(
            IteratorExtensions.toIterable(resource.getAllContents()).spliterator(), false)
        .filter(Reactor.class::isInstance)
        .map(Reactor.class::cast)
        .collect(Collectors.toList());
  }

  /**
   * Get the main reactor defined in the given resource.
   *
   * @param resource the resource to extract reactors from
   * @return An iterable over all reactors found in the resource
   */
  public static Reactor getMainReactor(Resource resource) {
    return StreamSupport.stream(
            IteratorExtensions.toIterable(resource.getAllContents()).spliterator(), false)
        .filter(Reactor.class::isInstance)
        .map(Reactor.class::cast)
        .filter(it -> it.isMain())
        .findFirst()
        .get();
  }

  /**
   * Find connections in the given resource that would be conflicting writes if they were not
   * located in mutually exclusive modes.
   *
   * @param resource The AST.
   * @return a list of connections being able to be transformed
   */
  public static Collection<Connection> findConflictingConnectionsInModalReactors(
      Resource resource) {
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
        for (var con :
            ASTUtils.<Connection>collectElements(
                reactor, featurePackage.getReactor_Connections(), false, true)) {
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
            if (!writerModes.containsKey(null)
                && // no writer is on root level (outside of modes) and...
                writerModes.keySet().stream()
                    .map(writerModes::get)
                    .allMatch(
                        writersInMode -> // all writers in a mode are either...
                        writersInMode.size() == 1
                                || // the only writer or...
                                writersInMode.stream()
                                    .allMatch(
                                        w ->
                                            w
                                                instanceof
                                                Reaction) // all are reactions and hence ordered
                        )) {
              // Add connections to transform list
              writers.stream()
                  .filter(w -> w instanceof Connection)
                  .forEach(c -> transform.add((Connection) c));
            }
          }
        }
      }
    }

    return transform;
  }

  /**
   * Return the enclosing reactor of an LF EObject in a reactor or mode.
   *
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

  /** Return the main reactor in the given resource if there is one, null otherwise. */
  public static Reactor findMainReactor(Resource resource) {
    return IteratorExtensions.findFirst(
        Iterators.filter(resource.getAllContents(), Reactor.class), Reactor::isMain);
  }

  /**
   * Find the main reactor and change it to a federated reactor. Return true if the transformation
   * was successful (or the given resource already had a federated reactor); return false otherwise.
   */
  public static boolean makeFederated(Resource resource) {
    // Find the main reactor
    Reactor r = findMainReactor(resource);
    if (r == null) {
      return false;
    }
    r.setMain(false);
    r.setFederated(true);
    return true;
  }

  /** Change the target name to 'newTargetName'. For example, change C to CCpp. */
  public static boolean changeTargetName(Resource resource, String newTargetName) {
    targetDecl(resource).setName(newTargetName);
    return true;
  }

  /** Return the target of the file in which the given node lives. */
  public static Target getTarget(EObject object) {
    TargetDecl targetDecl = targetDecl(object.eResource());
    return Target.fromDecl(targetDecl);
  }

  /**
   * Return true if the connection involves multiple ports on the left or right side of the
   * connection, or if the port on the left or right of the connection involves a bank of reactors
   * or a multiport.
   *
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
    Port leftPortAsPort = (Port) leftPort.getVariable();
    Port rightPortAsPort = (Port) rightPort.getVariable();
    return leftPortAsPort.getWidthSpec() != null
        || leftContainer != null && leftContainer.getWidthSpec() != null
        || rightPortAsPort.getWidthSpec() != null
        || rightContainer != null && rightContainer.getWidthSpec() != null;
  }

  /**
   * Produce a unique identifier within a reactor based on a given based name. If the name does not
   * exists, it is returned; if does exist, an index is appended that makes the name unique.
   *
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
   * Given a reactor class, return a list of all its actions, which includes actions of base classes
   * that it extends. This also includes actions in modes, returning a flattened view over all
   * modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<Action> allActions(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Actions());
  }

  /**
   * Given a reactor class, return a list of all its connections, which includes connections of base
   * classes that it extends. This also includes connections in modes, returning a flattened view
   * over all modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<Connection> allConnections(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Connections());
  }

  /**
   * Given a reactor class, return a list of all its inputs, which includes inputs of base classes
   * that it extends. If the base classes include a cycle, where X extends Y and Y extends X, then
   * return only the input defined in the base class. The returned list may be empty.
   *
   * @param definition Reactor class definition.
   */
  public static List<Input> allInputs(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Inputs());
  }

  /** A list of all ports of {@code definition}, in an unspecified order. */
  public static List<Port> allPorts(Reactor definition) {
    return Stream.concat(
            ASTUtils.allInputs(definition).stream(), ASTUtils.allOutputs(definition).stream())
        .toList();
  }

  /**
   * Given a reactor class, return a list of all its preambles, which includes preambles of base
   * classes that it extends. If the base classes include a cycle, where X extends Y and Y extends
   * X, then return only the input defined in the base class. The returned list may be empty.
   *
   * @param definition Reactor class definition.
   */
  public static List<Preamble> allPreambles(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Preambles());
  }

  /**
   * Given a reactor class, return a list of all its instantiations, which includes instantiations
   * of base classes that it extends. This also includes instantiations in modes, returning a
   * flattened view over all modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<Instantiation> allInstantiations(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Instantiations());
  }

  /*
   * Given a reactor class, return a stream of reactor classes that it instantiates.
   * @param definition Reactor class definition.
   * @return A stream of reactor classes.
   */
  public static Stream<Reactor> allNestedClasses(Reactor definition) {
    return new HashSet<>(ASTUtils.allInstantiations(definition))
        .stream().map(Instantiation::getReactorClass).map(ASTUtils::toDefinition);
  }

  /**
   * Given a reactor class, return a list of all its methods, which includes methods of base classes
   * that it extends.
   *
   * @param definition Reactor class definition.
   */
  public static List<Method> allMethods(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Methods());
  }

  /**
   * Given a reactor class, return a list of all its outputs, which includes outputs of base classes
   * that it extends.
   *
   * @param definition Reactor class definition.
   */
  public static List<Output> allOutputs(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Outputs());
  }

  /**
   * Given a reactor class, return a list of all its parameters, which includes parameters of base
   * classes that it extends.
   *
   * @param definition Reactor class definition.
   */
  public static List<Parameter> allParameters(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Parameters());
  }

  /**
   * Given a reactor class, return a list of all its reactions, which includes reactions of base
   * classes that it extends. This also includes reactions in modes, returning a flattened view over
   * all modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<Reaction> allReactions(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Reactions());
  }

  /**
   * Given a reactor class, return a list of all its watchdogs.
   *
   * @param definition Reactor class definition
   * @return List<Watchdog>
   */
  public static List<Watchdog> allWatchdogs(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Watchdogs());
  }

  /**
   * Given a reactor class, return a list of all its state variables, which includes state variables
   * of base classes that it extends. This also includes reactions in modes, returning a flattened
   * view over all modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<StateVar> allStateVars(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_StateVars());
  }

  /**
   * Given a reactor class, return a list of all its timers, which includes timers of base classes
   * that it extends. This also includes reactions in modes, returning a flattened view over all
   * modes.
   *
   * @param definition Reactor class definition.
   */
  public static List<Timer> allTimers(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Timers());
  }

  /**
   * Given a reactor class, returns a list of all its modes, which includes modes of base classes
   * that it extends.
   *
   * @param definition Reactor class definition.
   */
  public static List<Mode> allModes(Reactor definition) {
    return ASTUtils.collectElements(definition, featurePackage.getReactor_Modes());
  }

  public static List<ReactorInstance> recursiveChildren(ReactorInstance r) {
    List<ReactorInstance> ret = new ArrayList<>();
    ret.add(r);
    for (var child : r.children) {
      ret.addAll(recursiveChildren(child));
    }
    return ret;
  }

  /**
   * Return all the superclasses of the specified reactor in deepest-first order. For example, if A
   * extends B and C, and B and C both extend D, this will return the list [D, B, C, A]. Duplicates
   * are removed. If the specified reactor does not extend any other reactor, then return an empty
   * list. If a cycle is found, where X extends Y and Y extends X, or if a superclass is declared
   * that is not found, then return null.
   *
   * @param reactor The specified reactor.
   */
  public static LinkedHashSet<Reactor> superClasses(Reactor reactor) {
    return superClasses(reactor, new LinkedHashSet<>());
  }

  /**
   * Return all the file-level preambles in the files that define the specified class and its
   * superclasses in deepest-first order. Duplicates are removed. If there are no file-level
   * preambles, then return an empty list. If a cycle is found, where X extends Y and Y extends X,
   * or if a superclass is declared that is not found, then return null.
   *
   * @param reactor The specified reactor.
   */
  public static LinkedHashSet<Preamble> allFileLevelPreambles(Reactor reactor) {
    return allFileLevelPreambles(reactor, new LinkedHashSet<>());
  }

  /**
   * Collect elements of type T from the class hierarchy and modes defined by a given reactor
   * definition.
   *
   * @param definition The reactor definition.
   * @param <T> The type of elements to collect (e.g., Port, Timer, etc.)
   * @return A list of all elements of type T found
   */
  public static <T extends EObject> List<T> collectElements(
      Reactor definition, EStructuralFeature feature) {
    return ASTUtils.collectElements(definition, feature, true, true);
  }

  /**
   * Collect elements of type T contained in given reactor definition, including modes and the class
   * hierarchy defined depending on configuration.
   *
   * @param definition The reactor definition.
   * @param feature The structual model elements to collect.
   * @param includeSuperClasses Whether to include elements in super classes.
   * @param includeModes Whether to include elements in modes.
   * @param <T> The type of elements to collect (e.g., Port, Timer, etc.)
   * @return A list of all elements of type T found
   */
  @SuppressWarnings("unchecked")
  public static <T extends EObject> List<T> collectElements(
      Reactor definition,
      EStructuralFeature feature,
      boolean includeSuperClasses,
      boolean includeModes) {
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
        insertModeElementsAtTextualPosition(result, (EList<T>) mode.eGet(modeFeature), mode);
      }
    }

    return result;
  }

  /**
   * If a main or federated reactor has been declared, create a ReactorInstance for this top level.
   * This will also assign levels to reactions, then, if the program is federated, perform an AST
   * transformation to disconnect connections between federates.
   */
  public static ReactorInstance createMainReactorInstance(
      Instantiation mainDef,
      List<Reactor> reactors,
      MessageReporter messageReporter,
      TargetConfig targetConfig) {
    if (mainDef != null) {
      // Recursively build instances.
      ReactorInstance main =
          new ReactorInstance(toDefinition(mainDef.getReactorClass()), messageReporter, reactors);
      var reactionInstanceGraph = main.assignLevels();
      if (reactionInstanceGraph.nodeCount() > 0) {
        messageReporter
            .nowhere()
            .error("Main reactor has causality cycles. Skipping code generation.");
        return null;
      }
      // Inform the run-time of the breadth/parallelism of the reaction graph
      var breadth = reactionInstanceGraph.getBreadth();
      if (breadth == 0) {
        messageReporter.nowhere().warning("The program has no reactions");
      } else {
        targetConfig.compileDefinitions.put(
            "LF_REACTION_GRAPH_BREADTH", String.valueOf(reactionInstanceGraph.getBreadth()));
      }
      return main;
    }
    return null;
  }

  /**
   * Adds the elements into the given list at a location matching to their textual position.
   *
   * <p>When creating a flat view onto reactor elements including modes, the final list must be
   * ordered according to the textual positions.
   *
   * <p>Example: reactor R { reaction // -> is R.reactions[0] mode M { reaction // -> is
   * R.mode[0].reactions[0] reaction // -> is R.mode[0].reactions[1] } reaction // -> is
   * R.reactions[1] } In this example, it is important that the reactions in the mode are inserted
   * between the top-level reactions to retain the correct global reaction ordering, which will be
   * derived from this flattened view.
   *
   * @param list The list to add the elements into.
   * @param elements The elements to add.
   * @param mode The mode containing the elements.
   * @param <T> The type of elements to add (e.g., Port, Timer, etc.)
   */
  private static <T extends EObject> void insertModeElementsAtTextualPosition(
      List<T> list, List<T> elements, Mode mode) {
    if (elements.isEmpty()) {
      return; // Nothing to add
    }

    var idx = list.size();
    if (idx > 0) {
      // If there are elements in the list, first check if the last element has the same container
      // as the mode.
      // I.e. we don't want to compare textual positions of different reactors (super classes)
      if (mode.eContainer() == list.get(list.size() - 1).eContainer()) {
        var modeAstNode = NodeModelUtils.getNode(mode);
        if (modeAstNode != null) {
          var modePos = modeAstNode.getOffset();
          // Now move the insertion index from the last element forward as long as this element has
          // a textual
          // position after the mode.
          do {
            var astNode = NodeModelUtils.getNode(list.get(idx - 1));
            if (astNode != null && astNode.getOffset() > modePos) {
              idx--;
            } else {
              break; // Insertion index is ok.
            }
          } while (idx > 0);
        }
      }
    }
    list.addAll(idx, elements);
  }

  public static <T extends EObject> Iterable<T> allElementsOfClass(
      Resource resource, Class<T> elementClass) {
    //noinspection StaticPseudoFunctionalStyleMethod
    return Iterables.filter(IteratorExtensions.toIterable(resource.getAllContents()), elementClass);
  }

  ////////////////////////////////
  //// Utility functions for translating AST nodes into text

  /**
   * Translate the given code into its textual representation with {@code CodeMap.Correspondence}
   * tags inserted, or return the empty string if {@code node} is {@code null}. This method should
   * be used to generate code.
   *
   * @param node AST node to render as string.
   * @return Textual representation of the given argument.
   */
  public static String toText(EObject node) {
    if (node == null) return "";
    return CodeMap.Correspondence.tag(node, toOriginalText(node), node instanceof Code);
  }

  /**
   * Translate the given code into its textual representation without {@code CodeMap.Correspondence}
   * tags, or return the empty string if {@code node} is {@code null}. This method should be used
   * for analyzing AST nodes in cases where they are easiest to analyze as strings.
   *
   * @param node AST node to render as string.
   * @return Textual representation of the given argument.
   */
  public static String toOriginalText(EObject node) {
    if (node == null) return "";
    return ToText.instance.doSwitch(node);
  }

  /**
   * Return an integer representation of the given element.
   *
   * <p>Internally, this method uses Integer.decode, so it will also understand hexadecimal, binary,
   * etc.
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

  /** Returns the time value represented by the given AST node. */
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
    return elementToSingleString(e).equalsIgnoreCase("true");
  }

  /**
   * Given the right-hand side of a target property, return a string that represents the given
   * value/
   *
   * <p>If the given value is not a literal or and id (but for instance and array or dict), an empty
   * string is returned. If the element is a string, any quotes are removed.
   *
   * @param e The right-hand side of a target property.
   */
  public static String elementToSingleString(Element e) {
    if (e.getLiteral() != null) {
      return StringUtil.removeQuotes(e.getLiteral()).trim();
    } else if (e.getId() != null) {
      return e.getId();
    }
    return "";
  }

  /**
   * Given the right-hand side of a target property, return a list with all the strings that the
   * property lists.
   *
   * <p>Arrays are traversed, so strings are collected recursively. Empty strings are ignored; they
   * are not added to the list.
   *
   * @param value The right-hand side of a target property.
   */
  public static List<String> elementToListOfStrings(Element value) {
    List<String> elements = new ArrayList<>();
    if (value.getArray() != null) {
      for (Element element : value.getArray().getElements()) {
        elements.addAll(elementToListOfStrings(element));
      }
      return elements;
    } else {
      String v = elementToSingleString(value);
      if (!v.isEmpty()) {
        elements.add(v);
      }
    }
    return elements;
  }

  /**
   * Convert key-value pairs in an Element to a map, assuming that both the key and the value are
   * strings.
   */
  public static Map<String, String> elementToStringMaps(Element value) {
    Map<String, String> elements = new HashMap<>();
    for (var element : value.getKeyvalue().getPairs()) {
      elements.put(
          element.getName().trim(),
          StringUtil.removeQuotes(elementToSingleString(element.getValue())));
    }
    return elements;
  }

  // Various utility methods to convert various data types to Elements

  /** Convert a <String, String> map to key-value pairs in an Element. */
  public static Element toElement(Map<String, String> map) {
    Element e = LfFactory.eINSTANCE.createElement();
    if (map.size() == 0) return null;
    else {
      var kv = LfFactory.eINSTANCE.createKeyValuePairs();
      for (var entry : map.entrySet()) {
        var pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(entry.getKey());
        var element = LfFactory.eINSTANCE.createElement();
        element.setLiteral(StringUtil.addDoubleQuotes(entry.getValue()));
        pair.setValue(element);
        kv.getPairs().add(pair);
      }
      e.setKeyvalue(kv);
    }

    return e;
  }

  /**
   * Given a single string, convert it into its AST representation. {@code addQuotes} controls if
   * the generated representation should be accompanied by double quotes ("") or not.
   */
  private static Element toElement(String str, boolean addQuotes) {
    if (str == null) return null;
    var strToReturn = addQuotes ? StringUtil.addDoubleQuotes(str) : str;
    Element e = LfFactory.eINSTANCE.createElement();
    e.setLiteral(strToReturn);
    return e;
  }

  /** Given a single string, convert it into its AST representation. */
  public static Element toElement(String str) {
    return toElement(str, true);
  }

  /**
   * Given a list of strings, convert it into its AST representation. Stores the list in the Array
   * field of the element, unless the list only has one string, in which case it is stored in the
   * Literal field. Returns null if the provided list is empty.
   */
  public static Element toElement(List<String> list) {
    Element e = LfFactory.eINSTANCE.createElement();
    if (list.size() == 0) return null;
    else if (list.size() == 1) {
      return toElement(list.get(0));
    } else {
      var arr = LfFactory.eINSTANCE.createArray();
      for (String s : list) {
        arr.getElements().add(ASTUtils.toElement(s));
      }
      e.setArray(arr);
    }
    return e;
  }

  /**
   * Convert a TimeValue to its AST representation. The value is type-cast to int in order to fit
   * inside an Element.
   */
  public static Element toElement(TimeValue tv) {
    Element e = LfFactory.eINSTANCE.createElement();
    e.setTime((int) tv.time);
    if (tv.unit != null) {
      e.setUnit(tv.unit.toString());
    }
    return e;
  }

  public static Element toElement(boolean val) {
    return toElement(Boolean.toString(val), false);
  }

  public static Element toElement(int val) {
    return toElement(Integer.toString(val), false);
  }

  /**
   * Translate the given type into its textual representation, but do not append any array
   * specifications or type arguments.
   *
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
          StringBuilder result = new StringBuilder(type.getId());

          for (String s : convertToEmptyListIfNull(type.getStars())) {
            result.append(s);
          }
          return result.toString();
        }
      }
    }
    return "";
  }

  /**
   * Report whether the given literal is zero or not.
   *
   * @param literal AST node to inspect.
   * @return True if the given literal denotes the constant {@code 0}, false otherwise.
   */
  public static boolean isZero(String literal) {
    try {
      if (literal != null && Integer.parseInt(literal) == 0) {
        return true;
      }
    } catch (NumberFormatException e) {
      // Not an int.
    }
    return false;
  }

  /**
   * Report whether the given expression is zero or not.
   *
   * @param expr AST node to inspect.
   * @return True if the given value denotes the constant {@code 0}, false otherwise.
   */
  public static boolean isZero(Expression expr) {
    if (expr instanceof Literal) {
      return isZero(((Literal) expr).getLiteral());
    }
    return false;
  }

  /**
   * Report whether the given string literal is an integer number or not.
   *
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
   * Report whether the given string literal is a boolean value or not.
   *
   * @param literal AST node to inspect.
   * @return True if the given value is a boolean, false otherwise.
   */
  public static boolean isBoolean(String literal) {
    return literal.equalsIgnoreCase("true") || literal.equalsIgnoreCase("false");
  }

  /**
   * Report whether the given string literal is a float value or not.
   *
   * @param literal AST node to inspect.
   * @return True if the given value is a float, false otherwise.
   */
  public static boolean isFloat(String literal) {
    try {
      //noinspection ResultOfMethodCallIgnored
      Float.parseFloat(literal);
    } catch (NumberFormatException e) {
      return false;
    }
    return true;
  }

  /**
   * Report whether the given code is an integer number or not.
   *
   * @param code AST node to inspect.
   * @return True if the given code is an integer, false otherwise.
   */
  public static boolean isInteger(Code code) {
    return isInteger(toText(code));
  }

  /**
   * Report whether the given expression is an integer number or not.
   *
   * @param expr AST node to inspect.
   * @return True if the given value is an integer, false otherwise.
   */
  public static boolean isInteger(Expression expr) {
    if (expr instanceof Literal) {
      return isInteger(((Literal) expr).getLiteral());
    } else if (expr instanceof Code) {
      return isInteger((Code) expr);
    }
    return false;
  }

  /**
   * Report whether the given expression denotes a valid time or not.
   *
   * @param expr AST node to inspect.
   * @return True if the argument denotes a valid time, false otherwise.
   */
  public static boolean isValidTime(Expression expr) {
    if (expr instanceof ParameterReference) {
      return isOfTimeType(((ParameterReference) expr).getParameter());
    } else if (expr instanceof Time) {
      return isValidTime((Time) expr);
    } else if (expr instanceof Literal) {
      return isZero(((Literal) expr).getLiteral());
    }
    return false;
  }

  /**
   * Report whether the given time denotes a valid time or not.
   *
   * @param t AST node to inspect.
   * @return True if the argument denotes a valid time, false otherwise.
   */
  public static boolean isValidTime(Time t) {
    if (t == null) return false;
    String unit = t.getUnit();
    return t.getInterval() == 0 || TimeUnit.isValidUnit(unit);
  }

  /** If the initializer contains exactly one expression, return it. Otherwise, return null. */
  public static Expression asSingleExpr(Initializer init) {
    if (init == null) {
      return null;
    }
    var exprs = init.getExprs();
    return exprs.size() == 1 ? exprs.get(0) : null;
  }

  public static boolean isSingleExpr(Initializer init) {
    // todo expand that to = initialization
    if (init == null) {
      return false;
    }
    var exprs = init.getExprs();
    return exprs.size() == 1;
  }

  public static boolean isListInitializer(Initializer init) {
    return init != null && !isSingleExpr(init);
  }

  /**
   * Return the type of a declaration with the given (nullable) explicit type, and the given
   * (nullable) initializer. If the explicit type is null, then the type is inferred from the
   * initializer. Only two types can be inferred: "time" and "timeList". Return the "undefined" type
   * if neither can be inferred.
   *
   * @param type Explicit type declared on the declaration
   * @param init The initializer expression
   * @return The inferred type, or "undefined" if none could be inferred.
   */
  public static InferredType getInferredType(Type type, Initializer init) {
    if (type != null) {
      return InferredType.fromAST(type);
    } else if (init == null) {
      return InferredType.undefined();
    }

    var single = asSingleExpr(init);
    if (single != null) {
      // If there is a single element in the list, and it is a proper
      // time value with units, we infer the type "time".
      if (single instanceof ParameterReference) {
        return getInferredType(((ParameterReference) single).getParameter());
      } else if (single instanceof Time) {
        return InferredType.time();
      }
    } else if (init.getExprs().size() > 1) {
      // If there are multiple elements in the list, and there is at
      // least one proper time value with units, and all other elements
      // are valid times (including zero without units), we infer the
      // type "time list".
      var allValidTime = true;
      var foundNonZero = false;

      for (var e : init.getExprs()) {
        if (!ASTUtils.isValidTime(e)) {
          allValidTime = false;
        }
        if (!ASTUtils.isZero(e)) {
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
   * Given a parameter, return an inferred type. Only two types can be inferred: "time" and
   * "timeList". Return the "undefined" type if neither can be inferred.
   *
   * @param p A parameter to infer the type of.
   * @return The inferred type, or "undefined" if none could be inferred.
   */
  public static InferredType getInferredType(Parameter p) {
    return getInferredType(p.getType(), p.getInit());
  }

  /**
   * Given a state variable, return an inferred type. Only two types can be inferred: "time" and
   * "timeList". Return the "undefined" type if neither can be inferred.
   *
   * @param s A state variable to infer the type of.
   * @return The inferred type, or "undefined" if none could be inferred.
   */
  public static InferredType getInferredType(StateVar s) {
    return getInferredType(s.getType(), s.getInit());
  }

  /**
   * Construct an inferred type from an "action" AST node based on its declared type. If no type is
   * declared, return the "undefined" type.
   *
   * @param a An action to construct an inferred type object for.
   * @return The inferred type, or "undefined" if none was declared.
   */
  public static InferredType getInferredType(Action a) {
    return getInferredType(a.getType(), null);
  }

  /**
   * Construct an inferred type from a "port" AST node based on its declared type. If no type is
   * declared, return the "undefined" type.
   *
   * @param p A port to construct an inferred type object for.
   * @return The inferred type, or "undefined" if none was declared.
   */
  public static InferredType getInferredType(Port p) {
    return getInferredType(p.getType(), null);
  }

  /**
   * If the given string can be recognized as a floating-point number that has a leading decimal
   * point, prepend the string with a zero and return it. Otherwise, return the original string.
   *
   * @param literal A string might be recognizable as a floating point number with a leading decimal
   *     point.
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
   *
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
   *
   * @param reference The reference to the variable.
   */
  public static String generateVarRef(VarRef reference) {
    var prefix = "";
    if (reference.getContainer() != null) {
      prefix = reference.getContainer().getName() + ".";
    }
    return prefix + reference.getVariable().getName();
  }

  /** Assuming that the given expression denotes a valid time literal, return a time value. */
  public static TimeValue getLiteralTimeValue(Expression expr) {
    if (expr instanceof Time) {
      return toTimeValue((Time) expr);
    } else if (expr instanceof Literal && isZero(((Literal) expr).getLiteral())) {
      return TimeValue.ZERO;
    } else {
      return null;
    }
  }

  /** If the parameter is of time type, return its default value. Otherwise, return null. */
  public static TimeValue getDefaultAsTimeValue(Parameter p) {
    if (isOfTimeType(p)) {
      var init = asSingleExpr(p.getInit());
      if (init != null) {
        return getLiteralTimeValue(init);
      }
    }
    return null;
  }

  /** Return whether the given state variable is inferred to a time type. */
  public static boolean isOfTimeType(StateVar state) {
    InferredType t = getInferredType(state);
    return t.isTime && !t.isList;
  }

  /** Return whether the given parameter is inferred to a time type. */
  public static boolean isOfTimeType(Parameter param) {
    InferredType t = getInferredType(param);
    return t.isTime && !t.isList;
  }

  /**
   * Given a parameter, return its initial value. The initial value is a list of instances of
   * Expressions.
   *
   * <p>If the instantiations argument is null or an empty list, then the value returned is simply
   * the default value given when the parameter is defined.
   *
   * <p>If a list of instantiations is given, then the first instantiation is required to be an
   * instantiation of the reactor class that is parameterized by the parameter. I.e.,
   *
   * <pre>
   * <code>     <span class="hljs-keyword">parameter</span>.eContainer <span class="hljs-comment">== instantiations.get(0).reactorClass</span>
   * </code></pre>
   *
   * <p>If a second instantiation is given, then it is required to be an instantiation of a reactor
   * class that contains the first instantiation. That is,
   *
   * <pre>
   * <code>     instantiations.<span class="hljs-keyword">get</span>(<span class="hljs-number">0</span>).eContainer == instantiations.<span class="hljs-keyword">get</span>(<span class="hljs-number">1</span>).reactorClass
   * </code></pre>
   *
   * <p>More generally, for all 0 &lt;= i &lt; instantiations.size - 1,
   *
   * <pre>
   * <code>     instantiations.get(i)<span class="hljs-selector-class">.eContainer</span> == instantiations.get(<span class="hljs-selector-tag">i</span> + <span class="hljs-number">1</span>).reactorClass
   * </code></pre>
   *
   * <p>If any of these conditions is not satisfied, then an IllegalArgumentException will be
   * thrown.
   *
   * <p>Note that this chain of reactions cannot be inferred from the parameter because in each of
   * the predicates above, there may be more than one instantiation that can appear on the right
   * hand side of the predicate.
   *
   * <p>For example, consider the following program:
   *
   * <pre><code>     reactor A(x:int(<span class="hljs-number">1</span>)) {}
   *      reactor <span class="hljs-keyword">B(y:int(2)) </span>{
   *          <span class="hljs-built_in">a1</span> = new A(x = y)<span class="hljs-comment">;</span>
   *          <span class="hljs-built_in">a2</span> = new A(x = -<span class="hljs-number">1</span>)<span class="hljs-comment">;</span>
   *      }
   *      reactor C(z:int(<span class="hljs-number">3</span>)) {
   *          <span class="hljs-keyword">b1 </span>= new <span class="hljs-keyword">B(y </span>= z)<span class="hljs-comment">;</span>
   *          <span class="hljs-keyword">b2 </span>= new <span class="hljs-keyword">B(y </span>= -<span class="hljs-number">2</span>)<span class="hljs-comment">;</span>
   *      }
   * </code></pre>
   *
   * <p>Notice that there are a total of four instances of reactor class A. Then
   *
   * <pre>
   * <code>     initialValue(<span class="hljs-name">x</span>, null) returns <span class="hljs-number">1</span>
   *      initialValue(<span class="hljs-name">x</span>, [a1]) returns <span class="hljs-number">2</span>
   *      initialValue(<span class="hljs-name">x</span>, [a2]) returns <span class="hljs-number">-1</span>
   *      initialValue(<span class="hljs-name">x</span>, [a1, b1]) returns <span class="hljs-number">3</span>
   *      initialValue(<span class="hljs-name">x</span>, [a2, b1]) returns <span class="hljs-number">-1</span>
   *      initialValue(<span class="hljs-name">x</span>, [a1, b2]) returns <span class="hljs-number">-2</span>
   *      initialValue(<span class="hljs-name">x</span>, [a2, b2]) returns <span class="hljs-number">-1</span>
   * </code></pre>
   *
   * <p>(Actually, in each of the above cases, the returned value is a list with one entry, a
   * Literal, e.g. [&quot;1&quot;]).
   *
   * <p>There are two instances of reactor class B.
   *
   * <pre>
   * <code>     initialValue(<span class="hljs-name">y</span>, null) returns <span class="hljs-number">2</span>
   *      initialValue(<span class="hljs-name">y</span>, [a1]) throws an IllegalArgumentException
   *      initialValue(<span class="hljs-name">y</span>, [b1]) returns <span class="hljs-number">3</span>
   *      initialValue(<span class="hljs-name">y</span>, [b2]) returns <span class="hljs-number">-2</span>
   * </code></pre>
   *
   * @param parameter The parameter.
   * @param instantiations The (optional) list of instantiations.
   * @return The value of the parameter.
   * @throws IllegalArgumentException If an instantiation provided is not an instantiation of the
   *     reactor class that is parameterized by the respective parameter or if the chain of
   *     instantiations is not nested.
   */
  public static List<Expression> initialValue(
      Parameter parameter, List<Instantiation> instantiations) {
    // If instantiations are given, then check to see whether this parameter gets overridden in
    // the first of those instantiations.
    if (instantiations != null && instantiations.size() > 0) {
      // Check to be sure that the instantiation is in fact an instantiation
      // of the reactor class for which this is a parameter.
      Instantiation instantiation = instantiations.get(0);

      if (!belongsTo(parameter, instantiation)) {
        throw new IllegalArgumentException(
            "Parameter "
                + parameter.getName()
                + " is not a parameter of reactor instance "
                + instantiation.getName()
                + ".");
      }
      // In case there is more than one assignment to this parameter, we need to
      // find the last one.
      Assignment lastAssignment = null;
      for (Assignment assignment : instantiation.getParameters()) {
        if (assignment.getLhs().equals(parameter)) {
          lastAssignment = assignment;
        }
      }
      if (lastAssignment != null) {
        // Right hand side can be a list. Collect the entries.
        List<Expression> result = new ArrayList<>();
        for (Expression expr : lastAssignment.getRhs().getExprs()) {
          if (expr instanceof ParameterReference) {
            if (instantiations.size() > 1
                && instantiation.eContainer() != instantiations.get(1).getReactorClass()) {
              throw new IllegalArgumentException(
                  "Reactor instance "
                      + instantiation.getName()
                      + " is not contained by instance "
                      + instantiations.get(1).getName()
                      + ".");
            }
            result.addAll(
                initialValue(
                    ((ParameterReference) expr).getParameter(),
                    instantiations.subList(1, instantiations.size())));
          } else {
            result.add(expr);
          }
        }
        return result;
      }
    }
    // If we reach here, then either no instantiation was supplied or
    // there was no assignment in the instantiation. So just use the
    // parameter's initial value.
    return parameter.getInit().getExprs();
  }

  /**
   * Return true if the specified object (a Parameter, Port, Action, or Timer) belongs to the
   * specified instantiation, meaning that it is defined in the reactor class being instantiated or
   * one of its base classes.
   *
   * @param eobject The object.
   * @param instantiation The instantiation.
   */
  public static boolean belongsTo(EObject eobject, Instantiation instantiation) {
    Reactor reactor = toDefinition(instantiation.getReactorClass());
    return belongsTo(eobject, reactor);
  }

  /**
   * Return true if the specified object (a Parameter, Port, Action, or Timer) belongs to the
   * specified reactor, meaning that it is defined in reactor class or one of its base classes.
   *
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
   * Given a parameter return its integer value or null if it does not have an integer value. If the
   * value of the parameter is a list of integers, return the sum of value in the list. The
   * instantiations parameter is as in {@link #initialValue(Parameter, List)}.
   *
   * @param parameter The parameter.
   * @param instantiations The (optional) list of instantiations.
   * @return The integer value of the parameter, or null if it does not have an integer value.
   * @throws IllegalArgumentException If an instantiation provided is not an instantiation of the
   *     reactor class that is parameterized by the respective parameter or if the chain of
   *     instantiations is not nested.
   */
  public static Integer initialValueInt(Parameter parameter, List<Instantiation> instantiations) {
    List<Expression> expressions = initialValue(parameter, instantiations);
    int result = 0;
    for (Expression expr : expressions) {
      if (!(expr instanceof Literal)) {
        return null;
      }
      try {
        result += Integer.decode(((Literal) expr).getLiteral());
      } catch (NumberFormatException ex) {
        return null;
      }
    }
    return result;
  }

  /**
   * Given the width specification of port or instantiation and an (optional) list of nested
   * instantiations, return the width if it can be determined and -1 if not. It will not be able to
   * be determined if either the width is variable (in which case you should use {@link
   * #inferPortWidth(VarRef, Connection, List)} ) or the list of instantiations is incomplete or
   * missing. If there are parameter references in the width, they are evaluated to the extent
   * possible given the instantiations list.
   *
   * <p>The instantiations list is as in {@link #initialValue(Parameter, List)}. If the spec belongs
   * to an instantiation (for a bank of reactors), then the first element on this list should be the
   * instantiation that contains this instantiation. If the spec belongs to a port, then the first
   * element on the list should be the instantiation of the reactor that contains the port.
   *
   * @param spec The width specification or null (to return 1).
   * @param instantiations The (optional) list of instantiations.
   * @return The width, or -1 if the width could not be determined.
   * @throws IllegalArgumentException If an instantiation provided is not as given above or if the
   *     chain of instantiations is not nested.
   */
  public static int width(WidthSpec spec, List<Instantiation> instantiations) {
    if (spec == null) {
      return 1;
    }
    if (spec.isOfVariableLength() && spec.eContainer() instanceof Instantiation) {
      return inferWidthFromConnections(spec, instantiations);
    }
    var result = 0;
    for (WidthTerm term : spec.getTerms()) {
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
        // If the width cannot be determined because term's width <= 0, which means the term's width
        // must be inferred, try to infer the width using connections.
        if (spec.eContainer() instanceof Instantiation) {
          try {
            return inferWidthFromConnections(spec, instantiations);
          } catch (InvalidSourceException e) {
            // If the inference fails, return -1.
            return -1;
          }
        }
      }
    }
    return result;
  }

  /**
   * Infer the width of a port reference in a connection. The port reference one or two parts, a
   * port and an (optional) container which is an Instantiation that may refer to a bank of
   * reactors. The width will be the product of the bank width and the port width. The returned
   * value will be 1 if the port is not in a bank and is not a multiport.
   *
   * <p>If the width cannot be determined, this will return -1. The width cannot be determined if
   * the list of instantiations is missing or incomplete.
   *
   * <p>The instantiations list is as in {@link #initialValue(Parameter, List)}. The first element
   * on this list should be the instantiation that contains the specified connection.
   *
   * @param reference A port reference.
   * @param connection A connection, or null if not in the context of a connection.
   * @param instantiations The (optional) list of instantiations.
   * @return The width or -1 if it could not be determined.
   * @throws IllegalArgumentException If an instantiation provided is not as given above or if the
   *     chain of instantiations is not nested.
   */
  public static int inferPortWidth(
      VarRef reference, Connection connection, List<Instantiation> instantiations) {
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
                  throw new InvalidSourceException(
                      "Multiple ports with variable width on a connection.");
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
                  throw new InvalidSourceException(
                      "Multiple ports with variable width on a connection.");
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
   * Given an instantiation of a reactor or bank of reactors, return the width. This will be 1 if
   * this is not a reactor bank. Otherwise, this will attempt to determine the width. If the width
   * is declared as a literal constant, it will return that constant. If the width is specified as a
   * reference to a parameter, this will throw an exception. If the width is variable, this will
   * find connections in the enclosing reactor and attempt to infer the width. If the width cannot
   * be determined, it will throw an exception.
   *
   * <p>IMPORTANT: This method should not be used you really need to determine the width! It will
   * not evaluate parameter values.
   *
   * @see #width(WidthSpec, List)
   * @param instantiation A reactor instantiation.
   * @return The width, if it can be determined.
   * @deprecated
   */
  @Deprecated
  public static int widthSpecification(Instantiation instantiation) {
    int result = width(instantiation.getWidthSpec(), null);
    if (result < 0) {
      throw new InvalidSourceException(
          "Cannot determine width for the instance " + instantiation.getName());
    }
    return result;
  }

  /**
   * Report whether a state variable has been initialized or not.
   *
   * @param v The state variable to be checked.
   * @return True if the variable was initialized, false otherwise.
   */
  public static boolean isInitialized(StateVar v) {
    return v != null && v.getInit() != null;
  }

  /**
   * Report whether the given time state variable is initialized using a parameter or not.
   *
   * @param s A state variable.
   * @return True if the argument is initialized using a parameter, false otherwise.
   */
  public static boolean isParameterized(StateVar s) {
    return s.getInit() != null
        && IterableExtensions.exists(
            s.getInit().getExprs(), it -> it instanceof ParameterReference);
  }

  /**
   * Check if the reactor class uses generics
   *
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
   * If the specified reactor declaration is an import, then return the imported reactor class
   * definition. Otherwise, just return the argument.
   *
   * @param r A Reactor or an ImportedReactor.
   * @return The Reactor class definition or null if no definition is found.
   */
  public static Reactor toDefinition(ReactorDecl r) {
    if (r == null) return null;
    if (r instanceof Reactor) {
      return (Reactor) r;
    } else if (r instanceof ImportedReactor) {
      return ((ImportedReactor) r).getReactorClass();
    }
    return null;
  }

  /** Return all single-line or multi-line comments immediately preceding the given EObject. */
  public static Stream<String> getPrecedingComments(
      ICompositeNode compNode, Predicate<INode> filter) {
    return getPrecedingCommentNodes(compNode, filter).map(INode::getText);
  }

  /** Return all single-line or multi-line comments immediately preceding the given EObject. */
  public static Stream<INode> getPrecedingCommentNodes(
      ICompositeNode compNode, Predicate<INode> filter) {
    if (compNode == null) return Stream.of();
    List<INode> ret = new ArrayList<>();
    for (INode node : compNode.getAsTreeIterable()) {
      if (!(node instanceof ICompositeNode)) {
        if (isComment(node)) {
          if (filter.test(node)) ret.add(node);
        } else if (!node.getText().isBlank()) {
          break;
        }
      }
    }
    return ret.stream();
  }

  /** Return whether {@code node} is a comment. */
  public static boolean isComment(INode node) {
    return isMultilineComment(node) || isSingleLineComment(node);
  }

  /** Return whether {@code node} is a multiline comment. */
  public static boolean isMultilineComment(INode node) {
    return node instanceof HiddenLeafNode hlNode
        && hlNode.getGrammarElement() instanceof TerminalRule tRule
        && tRule.getName().equals("ML_COMMENT");
  }

  /** Return whether {@code node} is a multiline comment. */
  public static boolean isSingleLineComment(INode node) {
    return node instanceof HiddenLeafNode hlNode
        && hlNode.getGrammarElement() instanceof TerminalRule tRule
        && tRule.getName().equals("SL_COMMENT");
  }

  public static boolean isInCode(INode node) {
    return node.getParent() != null
        && node.getParent().getGrammarElement().eContainer() instanceof ParserRuleImpl pri
        && pri.getName().equals("Body");
  }

  /**
   * Return {@code true} if the given instance is top-level, i.e., its parent is {@code null}.
   *
   * @param instance The instance to check.
   */
  public static boolean isTopLevel(NamedInstance instance) {
    return instance.getParent() == null;
  }

  /** Return true if the given node starts on the same line as the given other node. */
  public static Predicate<INode> sameLine(ICompositeNode compNode) {
    return other -> {
      for (INode node : compNode.getAsTreeIterable()) {
        if (!(node instanceof ICompositeNode) && !node.getText().isBlank() && !isComment(node)) {
          return node.getStartLine() == other.getStartLine();
        }
      }
      return false;
    };
  }

  /**
   * Find the main reactor and set its name if none was defined.
   *
   * @param resource The resource to find the main reactor in.
   */
  public static void setMainName(Resource resource, String name) {
    Reactor main =
        IteratorExtensions.findFirst(
            Iterators.filter(resource.getAllContents(), Reactor.class),
            it -> it.isMain() || it.isFederated());
    if (main != null && StringExtensions.isNullOrEmpty(main.getName())) {
      main.setName(name);
    }
  }

  /**
   * Create a new instantiation node with the given reactor as its defining class.
   *
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
    for (int i = 0; i < reactor.getTypeParms().size(); i++) {
      Type t = LfFactory.eINSTANCE.createType();
      t.setId("UNSPECIFIED_TYPE");
      inst.getTypeArgs().add(t);
    }
    return inst;
  }

  /**
   * Returns the target declaration in the given model. Non-null because it would cause a parse
   * error.
   */
  public static TargetDecl targetDecl(Model model) {
    return IteratorExtensions.head(Iterators.filter(model.eAllContents(), TargetDecl.class));
  }

  /**
   * Returns the target declaration in the given resource. Non-null because it would cause a parse
   * error.
   */
  public static TargetDecl targetDecl(Resource model) {
    return IteratorExtensions.head(Iterators.filter(model.getAllContents(), TargetDecl.class));
  }

  /////////////////////////////////////////////////////////
  //// Private methods

  /** Returns the list if it is not null. Otherwise, return an empty list. */
  public static <T> List<T> convertToEmptyListIfNull(List<T> list) {
    return list != null ? list : new ArrayList<>();
  }

  /**
   * Return all the superclasses of the specified reactor in deepest-first order. For example, if A
   * extends B and C, and B and C both extend D, this will return the list [D, B, C, A]. Duplicates
   * are removed. If the specified reactor does not extend any other reactor, then return an empty
   * list. If a cycle is found, where X extends Y and Y extends X, or if a superclass is declared
   * that is not found, then return null.
   *
   * @param reactor The specified reactor.
   * @param extensions A set of reactors extending the specified reactor (used to detect circular
   *     extensions).
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

  /**
   * Return all the file-level preambles in the files that define the specified class and its
   * superclasses in deepest-first order. Duplicates are removed. If there are no file-level
   * preambles, then return an empty list. If a cycle is found, where X extends Y and Y extends X,
   * or if a superclass is declared that is not found, then return null.
   *
   * @param reactor The specified reactor.
   * @param extensions A set of reactors extending the specified reactor (used to detect circular
   *     extensions).
   */
  private static LinkedHashSet<Preamble> allFileLevelPreambles(
      Reactor reactor, Set<Reactor> extensions) {
    LinkedHashSet<Preamble> result = new LinkedHashSet<>();
    for (ReactorDecl superDecl : convertToEmptyListIfNull(reactor.getSuperClasses())) {
      Reactor r = toDefinition(superDecl);
      if (r == reactor || r == null) return null;
      // If r is in the extensions, then we have a circular inheritance structure.
      if (extensions.contains(r)) return null;
      extensions.add(r);
      LinkedHashSet<Preamble> basePreambles = allFileLevelPreambles(r, extensions);
      extensions.remove(r);
      if (basePreambles == null) return null;
      result.addAll(basePreambles);
    }
    result.addAll(((Model) reactor.eContainer()).getPreambles());
    return result;
  }

  /**
   * We may be able to infer the width by examining the connections of the enclosing reactor
   * definition. This works, for example, with delays between multiports or banks of reactors.
   * Attempt to infer the width from connections and return -1 if the width cannot be inferred.
   *
   * @param spec The width specification or null (to return 1).
   * @param instantiations The (optional) list of instantiations.
   * @return The width, or -1 if the width could not be inferred from connections.
   */
  private static int inferWidthFromConnections(WidthSpec spec, List<Instantiation> instantiations) {
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

  public static void addReactionAttribute(Reaction reaction, String name) {
    var fedAttr = factory.createAttribute();
    fedAttr.setAttrName(name);
    reaction.getAttributes().add(fedAttr);
  }
}
