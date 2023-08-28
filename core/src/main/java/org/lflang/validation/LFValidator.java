/* Validation checks for Lingua Franca code. */

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

package org.lflang.validation;

import static org.lflang.ast.ASTUtils.inferPortWidth;
import static org.lflang.ast.ASTUtils.isGeneric;
import static org.lflang.ast.ASTUtils.toDefinition;
import static org.lflang.ast.ASTUtils.toOriginalText;

import com.google.inject.Inject;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EAttribute;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.validation.Check;
import org.eclipse.xtext.validation.CheckType;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.lflang.AttributeUtils;
import org.lflang.InferredType;
import org.lflang.ModelInfo;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.Platform;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.federated.validation.FedValidator;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Assignment;
import org.lflang.lf.Attribute;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Expression;
import org.lflang.lf.Host;
import org.lflang.lf.IPV4Host;
import org.lflang.lf.IPV6Host;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Literal;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Model;
import org.lflang.lf.NamedHost;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.STP;
import org.lflang.lf.Serializer;
import org.lflang.lf.StateVar;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.Time;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.Type;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Visibility;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.util.FileUtil;

/**
 * Custom validation checks for Lingua Franca programs.
 *
 * <p>Also see: <a
 * href="https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#validation">...</a>
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Matt Weber
 * @author Christian Menard
 * @author Hou Seng Wong
 * @author Clément Fournier
 */
public class LFValidator extends BaseLFValidator {

  // The methods annotated with @Check are automatically invoked on AST nodes matching the types of
  // their arguments. CheckType.FAST ensures that these checks run whenever a file is modified;
  // when CheckType.NORMAL is used, the check is run upon saving.
  // NOTE: please list methods in alphabetical order, and follow a naming convention checkClass,
  // where Class is the AST class.

  @Check(CheckType.FAST)
  public void checkAction(Action action) {
    checkName(action.getName(), Literals.VARIABLE__NAME);
    if (action.getOrigin() == ActionOrigin.NONE) {
      error(
          "Action must have modifier {@code logical} or {@code physical}.",
          Literals.ACTION__ORIGIN);
    }
    if (action.getPolicy() != null && !SPACING_VIOLATION_POLICIES.contains(action.getPolicy())) {
      error(
          "Unrecognized spacing violation policy: "
              + action.getPolicy()
              + ". Available policies are: "
              + String.join(", ", SPACING_VIOLATION_POLICIES)
              + ".",
          Literals.ACTION__POLICY);
    }
    checkExpressionIsTime(action.getMinDelay(), Literals.ACTION__MIN_DELAY);
    checkExpressionIsTime(action.getMinSpacing(), Literals.ACTION__MIN_SPACING);
  }

  @Check(CheckType.FAST)
  public void checkInitializer(Initializer init) {
    if (init.isBraces() && target != Target.CPP) {
      error(
          "Brace initializers are only supported for the C++ target", Literals.INITIALIZER__BRACES);
    } else if (init.isParens() && target.mandatesEqualsInitializers()) {
      var message =
          "This syntax is deprecated in the "
              + target
              + " target, use an equal sign instead of parentheses for assignment.";
      if (init.getExprs().size() == 1) {
        message += " (run the formatter to fix this automatically)";
      }
      warning(message, Literals.INITIALIZER__PARENS);
    } else if (!init.isAssign() && init.eContainer() instanceof Assignment) {
      var feature = init.isBraces() ? Literals.INITIALIZER__BRACES : Literals.INITIALIZER__PARENS;
      var message =
          "This syntax is deprecated, do not use parentheses or braces but an equal sign.";
      if (init.getExprs().size() == 1) {
        message += " (run the formatter to fix this automatically)";
      }
      warning(message, feature);
    }
  }

  @Check(CheckType.FAST)
  public void checkBracedExpression(BracedListExpression expr) {
    if (!target.allowsBracedListExpressions()) {
      var message =
          "Braced expression lists are not a valid expression for the " + target + " target.";
      error(message, Literals.BRACED_LIST_EXPRESSION.eContainmentFeature());
    }
  }

  @Check(CheckType.FAST)
  public void checkAssignment(Assignment assignment) {

    // If the left-hand side is a time parameter, make sure the assignment has units
    typeCheck(
        assignment.getRhs(),
        ASTUtils.getInferredType(assignment.getLhs()),
        Literals.ASSIGNMENT__RHS);
    // If this assignment overrides a parameter that is used in a deadline,
    // report possible overflow.
    if (isCBasedTarget() && this.info.overflowingAssignments.contains(assignment)) {
      error(
          "Time value used to specify a deadline exceeds the maximum of "
              + TimeValue.MAX_LONG_DEADLINE
              + " nanoseconds.",
          Literals.ASSIGNMENT__RHS);
    }
  }

  /** Resolve the port type if it is known. */
  private Type portTypeIfResolvable(VarRef port) {
    var portType = ((Port) port.getVariable()).getType();
    return port.getContainer() == null
        ? portType
        : new TypeParameterizedReactor(port.getContainer(), List.of()).resolveType(portType);
  }

  @Check(CheckType.FAST)
  public void checkConnection(Connection connection) {

    // Report if connection is part of a cycle.
    Set<NamedInstance<?>> cycles = this.info.topologyCycles();
    for (VarRef lp : connection.getLeftPorts()) {
      for (VarRef rp : connection.getRightPorts()) {
        boolean leftInCycle = false;

        for (NamedInstance<?> it : cycles) {
          if ((lp.getContainer() == null && it.getDefinition().equals(lp.getVariable()))
              || (it.getDefinition().equals(lp.getVariable())
                  && it.getParent().equals(lp.getContainer()))) {
            leftInCycle = true;
            break;
          }
        }

        for (NamedInstance<?> it : cycles) {
          if ((rp.getContainer() == null && it.getDefinition().equals(rp.getVariable()))
              || (it.getDefinition().equals(rp.getVariable())
                  && it.getParent().equals(rp.getContainer()))) {
            if (leftInCycle) {
              Reactor reactor = ASTUtils.getEnclosingReactor(connection);
              String reactorName = reactor.getName();
              error(
                  String.format("Connection in reactor %s creates", reactorName)
                      + String.format(
                          "a cyclic dependency between %s and %s.",
                          toOriginalText(lp), toOriginalText(rp)),
                  Literals.CONNECTION__DELAY);
            }
          }
        }
      }
    }

    // FIXME: look up all ReactorInstance objects that have a definition equal to the
    // container of this connection. For each of those occurrences, the widths have to match.
    // For the C target, since C has such a weak type system, check that
    // the types on both sides of every connection match. For other languages,
    // we leave type compatibility that language's compiler or interpreter.
    if (isCBasedTarget()) {
      Type type = (Type) null;
      for (VarRef port :
          (Iterable<? extends VarRef>)
              () ->
                  Stream.concat(
                          connection.getLeftPorts().stream(), connection.getRightPorts().stream())
                      .iterator()) {
        // If the variable is not a port, then there is some other
        // error. Avoid a class cast exception.
        if (port.getVariable() instanceof Port) {
          if (type == null) {
            type = portTypeIfResolvable(port);
          } else {
            var portType = portTypeIfResolvable(port);
            if (!sameType(type, portType)) {
              error("Types do not match.", Literals.CONNECTION__LEFT_PORTS);
            }
          }
        }
      }
    }

    // Check whether the total width of the left side of the connection
    // matches the total width of the right side. This cannot be determined
    // here if the width is not given as a constant. In that case, it is up
    // to the code generator to check it.
    int leftWidth = 0;
    for (VarRef port : connection.getLeftPorts()) {
      int width = inferPortWidth(port, null, null); // null args imply incomplete check.
      if (width < 0 || leftWidth < 0) {
        // Cannot determine the width of the left ports.
        leftWidth = -1;
      } else {
        leftWidth += width;
      }
    }
    int rightWidth = 0;
    for (VarRef port : connection.getRightPorts()) {
      int width = inferPortWidth(port, null, null); // null args imply incomplete check.
      if (width < 0 || rightWidth < 0) {
        // Cannot determine the width of the left ports.
        rightWidth = -1;
      } else {
        rightWidth += width;
      }
    }

    if (leftWidth != -1 && rightWidth != -1 && leftWidth != rightWidth) {
      if (connection.isIterated()) {
        if (leftWidth == 0 || rightWidth % leftWidth != 0) {
          // FIXME: The second argument should be Literals.CONNECTION, but
          // stupidly, xtext will not accept that. There seems to be no way to
          // report an error for the whole connection statement.
          warning(
              String.format("Left width %s does not divide right width %s", leftWidth, rightWidth),
              Literals.CONNECTION__LEFT_PORTS);
        }
      } else {
        // FIXME: The second argument should be Literals.CONNECTION, but
        // stupidly, xtext will not accept that. There seems to be no way to
        // report an error for the whole connection statement.
        warning(
            String.format("Left width %s does not match right width %s", leftWidth, rightWidth),
            Literals.CONNECTION__LEFT_PORTS);
      }
    }

    Reactor reactor = ASTUtils.getEnclosingReactor(connection);

    // Make sure the right port is not already an effect of a reaction.
    for (Reaction reaction : ASTUtils.allReactions(reactor)) {
      for (VarRef effect : reaction.getEffects()) {
        for (VarRef rightPort : connection.getRightPorts()) {
          if (rightPort.getVariable().equals(effect.getVariable())
              && // Refers to the same variable
              rightPort.getContainer() == effect.getContainer()
              && // Refers to the same instance
              (reaction.eContainer() instanceof Reactor
                  || // Either is not part of a mode
                  connection.eContainer() instanceof Reactor
                  || connection.eContainer()
                      == reaction.eContainer() // Or they are in the same mode
              )) {
            error(
                "Cannot connect: Port named '"
                    + effect.getVariable().getName()
                    + "' is already effect of a reaction.",
                Literals.CONNECTION__RIGHT_PORTS);
          }
        }
      }
    }

    // Check that the right port does not already have some other
    // upstream connection.
    for (Connection c : reactor.getConnections()) {
      if (c != connection) {
        for (VarRef thisRightPort : connection.getRightPorts()) {
          for (VarRef thatRightPort : c.getRightPorts()) {
            if (thisRightPort.getVariable().equals(thatRightPort.getVariable())
                && // Refers to the same variable
                thisRightPort.getContainer() == thatRightPort.getContainer()
                && // Refers to the same instance
                (connection.eContainer() instanceof Reactor
                    || // Or either of the connections in not part of a mode
                    c.eContainer() instanceof Reactor
                    || connection.eContainer() == c.eContainer() // Or they are in the same mode
                )) {
              error(
                  "Cannot connect: Port named '"
                      + thisRightPort.getVariable().getName()
                      + "' may only appear once on the right side of a connection.",
                  Literals.CONNECTION__RIGHT_PORTS);
            }
          }
        }
      }
    }

    // Check the after delay
    if (connection.getDelay() != null) {
      final var delay = connection.getDelay();
      if (delay instanceof ParameterReference
          || delay instanceof Time
          || delay instanceof Literal) {
        checkExpressionIsTime(delay, Literals.CONNECTION__DELAY);
      } else {
        error(
            "After delays can only be given by time literals or parameters.",
            Literals.CONNECTION__DELAY);
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkDeadline(Deadline deadline) {
    if (isCBasedTarget() && this.info.overflowingDeadlines.contains(deadline)) {
      error(
          "Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
          Literals.DEADLINE__DELAY);
    }
    checkExpressionIsTime(deadline.getDelay(), Literals.DEADLINE__DELAY);
  }

  @Check(CheckType.FAST)
  public void checkHost(Host host) {
    String addr = host.getAddr();
    String user = host.getUser();
    if (user != null && !user.matches(USERNAME_REGEX)) {
      warning("Invalid user name.", Literals.HOST__USER);
    }
    if (host instanceof IPV4Host && !addr.matches(IPV4_REGEX)) {
      warning("Invalid IP address.", Literals.HOST__ADDR);
    } else if (host instanceof IPV6Host && !addr.matches(IPV6_REGEX)) {
      warning("Invalid IP address.", Literals.HOST__ADDR);
    } else if (host instanceof NamedHost && !addr.matches(HOST_OR_FQN_REGEX)) {
      warning("Invalid host name or fully qualified domain name.", Literals.HOST__ADDR);
    }
  }

  @Check
  public void checkImport(Import imp) {
    if (toDefinition(imp.getReactorClasses().get(0)).eResource().getErrors().size() > 0) {
      error("Error loading resource.", Literals.IMPORT__IMPORT_URI); // FIXME: print specifics.
      return;
    }

    // FIXME: report error if resource cannot be resolved.
    for (ImportedReactor reactor : imp.getReactorClasses()) {
      if (!isUnused(reactor)) {
        return;
      }
    }
    warning("Unused import.", Literals.IMPORT__IMPORT_URI);
  }

  @Check
  public void checkImportedReactor(ImportedReactor reactor) {
    if (isUnused(reactor)) {
      warning("Unused reactor class.", Literals.IMPORTED_REACTOR__REACTOR_CLASS);
    }

    if (info.instantiationGraph.hasCycles()) {
      Set<Reactor> cycleSet = new HashSet<>();
      for (Set<Reactor> cycle : info.instantiationGraph.getCycles()) {
        cycleSet.addAll(cycle);
      }
      if (dependsOnCycle(toDefinition(reactor), cycleSet, new HashSet<>())) {
        error(
            "Imported reactor '"
                + toDefinition(reactor).getName()
                + "' has cyclic instantiation in it.",
            Literals.IMPORTED_REACTOR__REACTOR_CLASS);
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkInput(Input input) {
    Reactor parent = (Reactor) input.eContainer();
    if (parent.isMain() || parent.isFederated()) {
      error("Main reactor cannot have inputs.", Literals.VARIABLE__NAME);
    }
    checkName(input.getName(), Literals.VARIABLE__NAME);
    if (target.requiresTypes) {
      if (input.getType() == null) {
        error("Input must have a type.", Literals.TYPED_VARIABLE__TYPE);
      }
    }

    // mutable has no meaning in C++
    if (input.isMutable() && this.target == Target.CPP) {
      warning(
          "The mutable qualifier has no meaning for the C++ target and should be removed. "
              + "In C++, any value can be made mutable by calling get_mutable_copy().",
          Literals.INPUT__MUTABLE);
    }

    // Variable width multiports are not supported (yet?).
    if (input.getWidthSpec() != null && input.getWidthSpec().isOfVariableLength()) {
      error("Variable-width multiports are not supported.", Literals.PORT__WIDTH_SPEC);
    }
  }

  @Check(CheckType.FAST)
  public void checkInstantiation(Instantiation instantiation) {
    checkName(instantiation.getName(), Literals.INSTANTIATION__NAME);
    Reactor reactor = toDefinition(instantiation.getReactorClass());
    if (reactor.isMain() || reactor.isFederated()) {
      error(
          "Cannot instantiate a main (or federated) reactor: "
              + instantiation.getReactorClass().getName(),
          Literals.INSTANTIATION__REACTOR_CLASS);
    }

    // Report error if this instantiation is part of a cycle.
    // FIXME: improve error message.
    // FIXME: Also report if there exists a cycle within.
    if (this.info.instantiationGraph.getCycles().size() > 0) {
      for (Set<Reactor> cycle : this.info.instantiationGraph.getCycles()) {
        Reactor container = (Reactor) instantiation.eContainer();
        if (cycle.contains(container) && cycle.contains(reactor)) {
          List<String> names = new ArrayList<>();
          for (Reactor r : cycle) {
            names.add(r.getName());
          }

          error(
              "Instantiation is part of a cycle: " + String.join(", ", names) + ".",
              Literals.INSTANTIATION__REACTOR_CLASS);
        }
      }
    }
    // Variable width multiports are not supported (yet?).
    if (instantiation.getWidthSpec() != null && instantiation.getWidthSpec().isOfVariableLength()) {
      if (isCBasedTarget()) {
        warning(
            "Variable-width banks are for internal use only.", Literals.INSTANTIATION__WIDTH_SPEC);
      } else {
        error("Variable-width banks are not supported.", Literals.INSTANTIATION__WIDTH_SPEC);
      }
    }
  }

  /** Check target parameters, which are key-value pairs. */
  @Check(CheckType.FAST)
  public void checkKeyValuePair(KeyValuePair param) {
    // Check only if the container's container is a Target.
    if (param.eContainer().eContainer() instanceof TargetDecl) {
      TargetProperty prop = TargetProperty.forName(param.getName());

      // Make sure the key is valid.
      if (prop == null) {
        String options =
            TargetProperty.getOptions().stream()
                .map(p -> p.description)
                .sorted()
                .collect(Collectors.joining(", "));
        warning(
            "Unrecognized target parameter: "
                + param.getName()
                + ". Recognized parameters are: "
                + options,
            Literals.KEY_VALUE_PAIR__NAME);
      } else {
        // Check whether the property is supported by the target.
        if (!prop.supportedBy.contains(this.target)) {
          warning(
              "The target parameter: "
                  + param.getName()
                  + " is not supported by the "
                  + this.target
                  + " target and will thus be ignored.",
              Literals.KEY_VALUE_PAIR__NAME);
        }

        // Run checks on the property. After running the check, errors/warnings
        // are retrievable from the targetPropertyErrors collection.
        prop.type.check(param.getValue(), param.getName(), this);
      }

      // Retrieve the errors that resulted from the check.
      for (String it : targetPropertyErrors) {
        error(it, Literals.KEY_VALUE_PAIR__VALUE);
      }
      targetPropertyErrors.clear();

      for (String it : targetPropertyWarnings) {
        error(it, Literals.KEY_VALUE_PAIR__VALUE);
      }
      targetPropertyWarnings.clear();
    }
  }

  @Check(CheckType.FAST)
  public void checkModel(Model model) {
    // Since we're doing a fast check, we only want to update
    // if the model info hasn't been initialized yet. If it has,
    // we use the old information and update it during a normal
    // check (see below).
    if (!info.updated) {
      info.update(model, errorReporter);
    }
  }

  @Check(CheckType.NORMAL)
  public void updateModelInfo(Model model) {
    info.update(model, errorReporter);
  }

  @Check(CheckType.FAST)
  public void checkOutput(Output output) {
    Reactor parent = (Reactor) output.eContainer();
    if (parent.isMain() || parent.isFederated()) {
      error("Main reactor cannot have outputs.", Literals.VARIABLE__NAME);
    }
    checkName(output.getName(), Literals.VARIABLE__NAME);
    if (this.target.requiresTypes) {
      if (output.getType() == null) {
        error("Output must have a type.", Literals.TYPED_VARIABLE__TYPE);
      }
    }

    // Variable width multiports are not supported (yet?).
    if (output.getWidthSpec() != null && output.getWidthSpec().isOfVariableLength()) {
      error("Variable-width multiports are not supported.", Literals.PORT__WIDTH_SPEC);
    }
  }

  @Check(CheckType.FAST)
  public void checkParameter(Parameter param) {
    checkName(param.getName(), Literals.PARAMETER__NAME);

    if (param.getInit() == null) {
      // todo make initialization non-mandatory
      //  https://github.com/lf-lang/lingua-franca/issues/623
      error("Parameter must have a default value.", Literals.PARAMETER__INIT);
      return;
    }

    if (this.target.requiresTypes) {
      // Report missing target type. param.inferredType.undefine
      if (ASTUtils.getInferredType(param).isUndefined()) {
        error("Type declaration missing.", Literals.PARAMETER__TYPE);
      }
    }

    if (param.getType() != null) {
      typeCheck(param.getInit(), ASTUtils.getInferredType(param), Literals.PARAMETER__INIT);
    }

    if (param.getInit() != null) {
      for (Expression expr : param.getInit().getExprs()) {
        if (expr instanceof ParameterReference) {
          // Initialization using parameters is forbidden.
          error("Parameter cannot be initialized using parameter.", Literals.PARAMETER__INIT);
        }
      }
    }

    if (this.target == Target.CPP) {
      EObject container = param.eContainer();
      Reactor reactor = (Reactor) container;
      if (reactor.isMain()) {
        // we need to check for the cli parameters that are always taken
        List<String> cliParams = List.of("t", "threads", "o", "timeout", "f", "fast", "help");
        if (cliParams.contains(param.getName())) {
          error(
              "Parameter '"
                  + param.getName()
                  + "' is already in use as command line argument by Lingua Franca,",
              Literals.PARAMETER__NAME);
        }
      }
    }

    if (isCBasedTarget() && this.info.overflowingParameters.contains(param)) {
      error(
          "Time value used to specify a deadline exceeds the maximum of "
              + TimeValue.MAX_LONG_DEADLINE
              + " nanoseconds.",
          Literals.PARAMETER__INIT);
    }
  }

  @Check(CheckType.FAST)
  public void checkPreamble(Preamble preamble) {
    if (this.target == Target.CPP) {
      if (preamble.getVisibility() == Visibility.NONE) {
        error(
            "Preambles for the C++ target need a visibility qualifier (private or public)!",
            Literals.PREAMBLE__VISIBILITY);
      } else if (preamble.getVisibility() == Visibility.PRIVATE) {
        EObject container = preamble.eContainer();
        if (container != null && container instanceof Reactor) {
          Reactor reactor = (Reactor) container;
          if (isGeneric(reactor)) {
            warning(
                "Private preambles in generic reactors are not truly private. "
                    + "Since the generated code is placed in a *_impl.hh file, it will "
                    + "be visible on the public interface. Consider using a public "
                    + "preamble within the reactor or a private preamble on file scope.",
                Literals.PREAMBLE__VISIBILITY);
          }
        }
      }
    } else if (preamble.getVisibility() != Visibility.NONE) {
      warning(
          String.format(
              "The %s qualifier has no meaning for the %s target. It should be removed.",
              preamble.getVisibility(), this.target.name()),
          Literals.PREAMBLE__VISIBILITY);
    }
  }

  @Check(CheckType.FAST)
  public void checkReaction(Reaction reaction) {

    if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
      warning("Reaction has no trigger.", Literals.REACTION__TRIGGERS);
    }

    if (reaction.getCode() == null) {
      if (!this.target.supportsReactionDeclarations()) {
        error(
            "The "
                + this.target
                + " target does not support reaction declarations. Please specify a reaction body.",
            Literals.REACTION__CODE);
        return;
      }
      if (reaction.getDeadline() == null && reaction.getStp() == null) {
        var text = NodeModelUtils.findActualNodeFor(reaction).getText();
        var matcher = Pattern.compile("\\)\\s*[\\n\\r]+(.*[\\n\\r])*.*->").matcher(text);
        if (matcher.find()) {
          error(
              "A connection statement may have been unintentionally parsed as the sources and"
                  + " effects of a reaction declaration. To correct this, add a semicolon at the"
                  + " end of the reaction declaration. To instead silence this message, remove any"
                  + " newlines between the reaction triggers and sources.",
              Literals.REACTION__CODE);
        }
      }
    }
    HashSet<VarRef> triggers = new HashSet<>();
    // Make sure input triggers have no container and output sources do.
    for (TriggerRef trigger : reaction.getTriggers()) {
      if (trigger instanceof VarRef) {
        VarRef triggerVarRef = (VarRef) trigger;
        triggers.add(triggerVarRef);
        if (triggerVarRef instanceof Input) {
          if (triggerVarRef.getContainer() != null) {
            error(
                String.format(
                    "Cannot have an input of a contained reactor as a trigger: %s.%s",
                    triggerVarRef.getContainer().getName(), triggerVarRef.getVariable().getName()),
                Literals.REACTION__TRIGGERS);
          }
        } else if (triggerVarRef.getVariable() instanceof Output) {
          if (triggerVarRef.getContainer() == null) {
            error(
                String.format(
                    "Cannot have an output of this reactor as a trigger: %s",
                    triggerVarRef.getVariable().getName()),
                Literals.REACTION__TRIGGERS);
          }
        }
      }
    }

    // Make sure input sources have no container and output sources do.
    // Also check that a source is not already listed as a trigger.
    for (VarRef source : reaction.getSources()) {
      var duplicate =
          triggers.stream()
              .anyMatch(
                  t -> {
                    return t.getVariable().equals(source.getVariable())
                        && t.getContainer().equals(source.getContainer());
                  });
      if (duplicate) {
        error(
            String.format(
                "Source is already listed as a trigger: %s", source.getVariable().getName()),
            Literals.REACTION__SOURCES);
      }
      if (source.getVariable() instanceof Input) {
        if (source.getContainer() != null) {
          error(
              String.format(
                  "Cannot have an input of a contained reactor as a source: %s.%s",
                  source.getContainer().getName(), source.getVariable().getName()),
              Literals.REACTION__SOURCES);
        }
      } else if (source.getVariable() instanceof Output) {
        if (source.getContainer() == null) {
          error(
              String.format(
                  "Cannot have an output of this reactor as a source: %s",
                  source.getVariable().getName()),
              Literals.REACTION__SOURCES);
        }
      }
    }

    // Make sure output effects have no container and input effects do.
    for (VarRef effect : reaction.getEffects()) {
      if (effect.getVariable() instanceof Input) {
        if (effect.getContainer() == null) {
          error(
              String.format(
                  "Cannot have an input of this reactor as an effect: %s",
                  effect.getVariable().getName()),
              Literals.REACTION__EFFECTS);
        }
      } else if (effect.getVariable() instanceof Output) {
        if (effect.getContainer() != null) {
          error(
              String.format(
                  "Cannot have an output of a contained reactor as an effect: %s.%s",
                  effect.getContainer().getName(), effect.getVariable().getName()),
              Literals.REACTION__EFFECTS);
        }
      }
    }

    // // Report error if this reaction is part of a cycle.
    Set<NamedInstance<?>> cycles = this.info.topologyCycles();
    Reactor reactor = ASTUtils.getEnclosingReactor(reaction);
    boolean reactionInCycle = false;
    for (NamedInstance<?> it : cycles) {
      if (it.getDefinition().equals(reaction)) {
        reactionInCycle = true;
        break;
      }
    }
    if (reactionInCycle) {
      // Report involved triggers.
      List<CharSequence> trigs = new ArrayList<>();
      for (TriggerRef t : reaction.getTriggers()) {
        if (!(t instanceof VarRef)) {
          continue;
        }
        VarRef tVarRef = (VarRef) t;
        boolean triggerExistsInCycle = false;
        for (NamedInstance<?> c : cycles) {
          if (c.getDefinition().equals(tVarRef.getVariable())) {
            triggerExistsInCycle = true;
            break;
          }
        }
        if (triggerExistsInCycle) {
          trigs.add(toOriginalText(tVarRef));
        }
      }
      if (trigs.size() > 0) {
        error(
            String.format(
                "Reaction triggers involved in cyclic dependency in reactor %s: %s.",
                reactor.getName(), String.join(", ", trigs)),
            Literals.REACTION__TRIGGERS);
      }

      // Report involved sources.
      List<CharSequence> sources = new ArrayList<>();
      for (VarRef t : reaction.getSources()) {
        boolean sourceExistInCycle = false;
        for (NamedInstance<?> c : cycles) {
          if (c.getDefinition().equals(t.getVariable())) {
            sourceExistInCycle = true;
            break;
          }
        }
        if (sourceExistInCycle) {
          sources.add(toOriginalText(t));
        }
      }
      if (sources.size() > 0) {
        error(
            String.format(
                "Reaction sources involved in cyclic dependency in reactor %s: %s.",
                reactor.getName(), String.join(", ", sources)),
            Literals.REACTION__SOURCES);
      }

      // Report involved effects.
      List<CharSequence> effects = new ArrayList<>();
      for (VarRef t : reaction.getEffects()) {
        boolean effectExistInCycle = false;
        for (NamedInstance<?> c : cycles) {
          if (c.getDefinition().equals(t.getVariable())) {
            effectExistInCycle = true;
            break;
          }
        }
        if (effectExistInCycle) {
          effects.add(toOriginalText(t));
        }
      }
      if (effects.size() > 0) {
        error(
            String.format(
                "Reaction effects involved in cyclic dependency in reactor %s: %s.",
                reactor.getName(), String.join(", ", effects)),
            Literals.REACTION__EFFECTS);
      }

      if (trigs.size() + sources.size() == 0) {
        error(
            String.format(
                "Cyclic dependency due to preceding reaction. Consider reordering reactions within"
                    + " reactor %s to avoid causality loop.",
                reactor.getName()),
            reaction.eContainer(),
            Literals.REACTOR__REACTIONS,
            reactor.getReactions().indexOf(reaction));
      } else if (effects.size() == 0) {
        error(
            String.format(
                "Cyclic dependency due to succeeding reaction. Consider reordering reactions within"
                    + " reactor %s to avoid causality loop.",
                reactor.getName()),
            reaction.eContainer(),
            Literals.REACTOR__REACTIONS,
            reactor.getReactions().indexOf(reaction));
      }
      // Not reporting reactions that are part of cycle _only_ due to reaction ordering.
      // Moving them won't help solve the problem.
    }
    // FIXME: improve error message.
  }

  public void checkReactorName(String name) throws IOException {
    // Check for illegal names.
    checkName(name, Literals.REACTOR_DECL__NAME);

    // C++ reactors may not be called 'preamble'
    if (this.target == Target.CPP && name.equalsIgnoreCase("preamble")) {
      error("Reactor cannot be named '" + name + "'", Literals.REACTOR_DECL__NAME);
    }
  }

  @Check(CheckType.FAST)
  public void checkReactor(Reactor reactor) throws IOException {
    String fileName = FileUtil.nameWithoutExtension(reactor.eResource());

    if (reactor.isFederated() || reactor.isMain()) {
      // Do not allow multiple main/federated reactors.
      TreeIterator<EObject> iter = reactor.eResource().getAllContents();
      int nMain = countMainOrFederated(iter);
      if (nMain > 1) {
        EAttribute attribute = Literals.REACTOR__MAIN;
        if (reactor.isFederated()) {
          attribute = Literals.REACTOR__FEDERATED;
        }
        error("Multiple definitions of main or federated reactor.", attribute);
      }

      if (reactor.getName() != null && !reactor.getName().equals(fileName)) {
        // Make sure that if the name is given, it matches the expected name.
        error(
            "Name of main reactor must match the file name (or be omitted).",
            Literals.REACTOR_DECL__NAME);
      }

      // check the reactor name indicated by the file name
      // Skip this check if the file is named __synthetic0. This Name is used during testing,
      // and we would get an unexpected error due to the '__' prefix otherwise.
      if (!fileName.equals("__synthetic0")) {
        checkReactorName(fileName);
      }
    } else {
      // Not federated or main.
      if (reactor.getName() == null) {
        error("Reactor must be named.", Literals.REACTOR_DECL__NAME);
      } else {
        checkReactorName(reactor.getName());

        TreeIterator<EObject> iter = reactor.eResource().getAllContents();
        int nMain = countMainOrFederated(iter);
        if (nMain > 0 && reactor.getName().equals(fileName)) {
          error("Name conflict with main reactor.", Literals.REACTOR_DECL__NAME);
        }
      }
    }

    Set<Reactor> superClasses = ASTUtils.superClasses(reactor);
    if (superClasses == null) {
      error(
          "Problem with superclasses: Either they form a cycle or are not defined",
          Literals.REACTOR_DECL__NAME);
      // Continue checks, but without any superclasses.
      superClasses = new LinkedHashSet<>();
    }

    if (reactor.getHost() != null) {
      if (!reactor.isFederated()) {
        error(
            "Cannot assign a host to reactor '"
                + reactor.getName()
                + "' because it is not federated.",
            Literals.REACTOR__HOST);
      }
    }

    List<Variable> variables = new ArrayList<>();
    variables.addAll(reactor.getInputs());
    variables.addAll(reactor.getOutputs());
    variables.addAll(reactor.getActions());
    variables.addAll(reactor.getTimers());

    if (!reactor.getSuperClasses().isEmpty() && !target.supportsInheritance()) {
      error(
          "The " + target.getDisplayName() + " target does not support reactor inheritance.",
          Literals.REACTOR__SUPER_CLASSES);
    }

    // Perform checks on super classes.
    for (Reactor superClass : superClasses) {
      HashSet<Variable> conflicts = new HashSet<>();

      // Detect input conflicts
      checkConflict(superClass.getInputs(), reactor.getInputs(), variables, conflicts);
      // Detect output conflicts
      checkConflict(superClass.getOutputs(), reactor.getOutputs(), variables, conflicts);
      // Detect output conflicts
      checkConflict(superClass.getActions(), reactor.getActions(), variables, conflicts);
      // Detect conflicts
      for (Timer timer : superClass.getTimers()) {
        List<Variable> filteredVariables = new ArrayList<>(variables);
        filteredVariables.removeIf(it -> reactor.getTimers().contains(it));
        if (hasNameConflict(timer, filteredVariables)) {
          conflicts.add(timer);
        } else {
          variables.add(timer);
        }
      }

      // Report conflicts.
      if (conflicts.size() > 0) {
        List<String> names = new ArrayList<>();
        for (Variable it : conflicts) {
          names.add(it.getName());
        }
        error(
            String.format(
                "Cannot extend %s due to the following conflicts: %s.",
                superClass.getName(), String.join(",", names)),
            Literals.REACTOR__SUPER_CLASSES);
      }
    }

    if (reactor.isFederated()) {
      if (!target.supportsFederated()) {
        error(
            "The " + target.getDisplayName() + " target does not support federated execution.",
            Literals.REACTOR__FEDERATED);
      }

      FedValidator.validateFederatedReactor(reactor, this.errorReporter);
    }
  }

  /** Check if the requested serialization is supported. */
  @Check(CheckType.FAST)
  public void checkSerializer(Serializer serializer) {
    boolean isValidSerializer = false;
    for (SupportedSerializers method : SupportedSerializers.values()) {
      if (method.name().equalsIgnoreCase(serializer.getType())) {
        isValidSerializer = true;
      }
    }

    if (!isValidSerializer) {
      error(
          "Serializer can be " + Arrays.asList(SupportedSerializers.values()),
          Literals.SERIALIZER__TYPE);
    }
  }

  @Check(CheckType.FAST)
  public void checkState(StateVar stateVar) {
    checkName(stateVar.getName(), Literals.STATE_VAR__NAME);
    if (stateVar.getInit() != null && stateVar.getInit().getExprs().size() != 0) {
      typeCheck(stateVar.getInit(), ASTUtils.getInferredType(stateVar), Literals.STATE_VAR__INIT);
    }

    if (this.target.requiresTypes && ASTUtils.getInferredType(stateVar).isUndefined()) {
      // Report if a type is missing
      error("State must have a type.", Literals.STATE_VAR__TYPE);
    }

    if (isCBasedTarget()
        && ASTUtils.isListInitializer(stateVar.getInit())
        && stateVar.getInit().getExprs().stream()
            .anyMatch(it -> it instanceof ParameterReference)) {
      // In C, if initialization is done with a list, elements cannot
      // refer to parameters.
      error("List items cannot refer to a parameter.", Literals.STATE_VAR__INIT);
    }
  }

  @Check(CheckType.FAST)
  public void checkSTPOffset(STP stp) {
    if (isCBasedTarget() && this.info.overflowingSTP.contains(stp)) {
      error(
          "STP offset exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
          Literals.STP__VALUE);
    }
  }

  @Check(CheckType.FAST)
  public void checkTargetDecl(TargetDecl target) throws IOException {
    Optional<Target> targetOpt = Target.forName(target.getName());
    if (targetOpt.isEmpty()) {
      error("Unrecognized target: " + target.getName(), Literals.TARGET_DECL__NAME);
    } else {
      this.target = targetOpt.get();
    }
    String lfFileName = FileUtil.nameWithoutExtension(target.eResource());
    if (Character.isDigit(lfFileName.charAt(0))) {
      errorReporter.nowhere().error("LF file names must not start with a number");
    }
  }

  /**
   * Check for consistency of the target properties, which are defined as KeyValuePairs.
   *
   * @param targetProperties The target properties defined in the current Lingua Franca program.
   */
  @Check(CheckType.NORMAL)
  public void checkTargetProperties(KeyValuePairs targetProperties) {
    validateFastTargetProperty(targetProperties);
    validateClockSyncTargetProperties(targetProperties);
    validateSchedulerTargetProperties(targetProperties);
    validateRos2TargetProperties(targetProperties);
    validateKeepalive(targetProperties);
    validateThreading(targetProperties);
  }

  private KeyValuePair getKeyValuePair(KeyValuePairs targetProperties, TargetProperty property) {
    List<KeyValuePair> properties =
        targetProperties.getPairs().stream()
            .filter(pair -> pair.getName().equals(property.description))
            .toList();
    assert (properties.size() <= 1);
    return properties.size() > 0 ? properties.get(0) : null;
  }

  private void validateFastTargetProperty(KeyValuePairs targetProperties) {
    KeyValuePair fastTargetProperty = getKeyValuePair(targetProperties, TargetProperty.FAST);

    if (fastTargetProperty != null) {
      // Check for federated
      for (Reactor reactor : info.model.getReactors()) {
        // Check to see if the program has a federated reactor
        if (reactor.isFederated()) {
          error(
              "The fast target property is incompatible with federated programs.",
              fastTargetProperty,
              Literals.KEY_VALUE_PAIR__NAME);
          break;
        }
      }

      // Check for physical actions
      for (Reactor reactor : info.model.getReactors()) {
        // Check to see if the program has a physical action in a reactor
        for (Action action : reactor.getActions()) {
          if (action.getOrigin().equals(ActionOrigin.PHYSICAL)) {
            error(
                "The fast target property is incompatible with physical actions.",
                fastTargetProperty,
                Literals.KEY_VALUE_PAIR__NAME);
            break;
          }
        }
      }
    }
  }

  private void validateClockSyncTargetProperties(KeyValuePairs targetProperties) {
    KeyValuePair clockSyncTargetProperty =
        getKeyValuePair(targetProperties, TargetProperty.CLOCK_SYNC);

    if (clockSyncTargetProperty != null) {
      boolean federatedExists = false;
      for (Reactor reactor : info.model.getReactors()) {
        if (reactor.isFederated()) {
          federatedExists = true;
        }
      }
      if (!federatedExists) {
        warning(
            "The clock-sync target property is incompatible with non-federated programs.",
            clockSyncTargetProperty,
            Literals.KEY_VALUE_PAIR__NAME);
      }
    }
  }

  private void validateSchedulerTargetProperties(KeyValuePairs targetProperties) {
    KeyValuePair schedulerTargetProperty =
        getKeyValuePair(targetProperties, TargetProperty.SCHEDULER);
    if (schedulerTargetProperty != null) {
      String schedulerName = ASTUtils.elementToSingleString(schedulerTargetProperty.getValue());
      try {
        if (!TargetProperty.SchedulerOption.valueOf(schedulerName).prioritizesDeadline()) {
          // Check if a deadline is assigned to any reaction
          // Filter reactors that contain at least one reaction that
          // has a deadline handler.
          if (info.model.getReactors().stream()
              .anyMatch(
                  // Filter reactors that contain at least one reaction that
                  // has a deadline handler.
                  reactor ->
                      ASTUtils.allReactions(reactor).stream()
                          .anyMatch(reaction -> reaction.getDeadline() != null))) {
            warning(
                "This program contains deadlines, but the chosen "
                    + schedulerName
                    + " scheduler does not prioritize reaction execution "
                    + "based on deadlines. This might result in a sub-optimal "
                    + "scheduling.",
                schedulerTargetProperty,
                Literals.KEY_VALUE_PAIR__VALUE);
          }
        }
      } catch (IllegalArgumentException e) {
        // the given scheduler is invalid, but this is already checked by
        // checkTargetProperties
      }
    }
  }

  private void validateKeepalive(KeyValuePairs targetProperties) {
    KeyValuePair keepalive = getKeyValuePair(targetProperties, TargetProperty.KEEPALIVE);
    if (keepalive != null && target == Target.CPP) {
      warning(
          "The keepalive property is inferred automatically by the C++ "
              + "runtime and the value given here is ignored",
          keepalive,
          Literals.KEY_VALUE_PAIR__NAME);
    }
  }

  private void validateThreading(KeyValuePairs targetProperties) {
    var threadingP = getKeyValuePair(targetProperties, TargetProperty.THREADING);
    var tracingP = getKeyValuePair(targetProperties, TargetProperty.TRACING);
    var platformP = getKeyValuePair(targetProperties, TargetProperty.PLATFORM);
    if (threadingP != null) {
      if (tracingP != null) {
        if (!ASTUtils.toBoolean(threadingP.getValue())
            && !tracingP.getValue().toString().equalsIgnoreCase("false")) {
          error(
              "Cannot disable treading support because tracing is enabled",
              threadingP,
              Literals.KEY_VALUE_PAIR__NAME);
          error(
              "Cannot enable tracing because threading support is disabled",
              tracingP,
              Literals.KEY_VALUE_PAIR__NAME);
        }
      }
      if (platformP != null && ASTUtils.toBoolean(threadingP.getValue())) {
        var lit = ASTUtils.elementToSingleString(platformP.getValue());
        var dic = platformP.getValue().getKeyvalue();
        if (lit != null && lit.equalsIgnoreCase(Platform.RP2040.toString())) {
          error(
              "Platform " + Platform.RP2040 + " does not support threading",
              platformP,
              Literals.KEY_VALUE_PAIR__VALUE);
        }
        if (dic != null) {
          var rp =
              dic.getPairs().stream()
                  .filter(
                      kv ->
                          kv.getName().equalsIgnoreCase("name")
                              && ASTUtils.elementToSingleString(kv.getValue())
                                  .equalsIgnoreCase(Platform.RP2040.toString()))
                  .findFirst();
          if (rp.isPresent()) {
            error(
                "Platform " + Platform.RP2040 + " does not support threading",
                rp.get(),
                Literals.KEY_VALUE_PAIR__VALUE);
          }
        }
      }
    }
  }

  private void validateRos2TargetProperties(KeyValuePairs targetProperties) {
    KeyValuePair ros2 = getKeyValuePair(targetProperties, TargetProperty.ROS2);
    KeyValuePair ros2Dependencies =
        getKeyValuePair(targetProperties, TargetProperty.ROS2_DEPENDENCIES);
    if (ros2Dependencies != null && (ros2 == null || !ASTUtils.toBoolean(ros2.getValue()))) {
      warning(
          "Ignoring ros2-dependencies as ros2 compilation is disabled",
          ros2Dependencies,
          Literals.KEY_VALUE_PAIR__NAME);
    }
  }

  @Check(CheckType.FAST)
  public void checkTimer(Timer timer) {
    checkName(timer.getName(), Literals.VARIABLE__NAME);
    checkExpressionIsTime(timer.getOffset(), Literals.TIMER__OFFSET);
    checkExpressionIsTime(timer.getPeriod(), Literals.TIMER__PERIOD);
  }

  @Check(CheckType.FAST)
  public void checkType(Type type) {
    if (this.target == Target.Python) {
      if (type != null) {
        error("Types are not allowed in the Python target", Literals.TYPE__ID);
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkVarRef(VarRef varRef) {
    // check correct usage of interleaved
    if (varRef.isInterleaved()) {
      var supportedTargets = List.of(Target.CPP, Target.Python, Target.Rust);
      if (!supportedTargets.contains(this.target) && !isCBasedTarget()) {
        error(
            "This target does not support interleaved port references.",
            Literals.VAR_REF__INTERLEAVED);
      }
      if (!(varRef.eContainer() instanceof Connection)) {
        error("interleaved can only be used in connections.", Literals.VAR_REF__INTERLEAVED);
      }

      if (varRef.getVariable() instanceof Port) {
        // This test only works correctly if the variable is actually a port. If it is not a port,
        // other
        // validator rules will produce error messages.
        if (varRef.getContainer() == null
            || varRef.getContainer().getWidthSpec() == null
            || ((Port) varRef.getVariable()).getWidthSpec() == null) {
          error(
              "interleaved can only be used for multiports contained within banks.",
              Literals.VAR_REF__INTERLEAVED);
        }
      }
    }
  }

  /**
   * Check whether an attribute is supported and the validity of the attribute.
   *
   * @param attr The attribute being checked
   */
  @Check(CheckType.FAST)
  public void checkAttributes(Attribute attr) {
    String name = attr.getAttrName().toString();
    AttributeSpec spec = AttributeSpec.ATTRIBUTE_SPECS_BY_NAME.get(name);
    if (spec == null) {
      error("Unknown attribute.", Literals.ATTRIBUTE__ATTR_NAME);
      return;
    }
    // Check the validity of the attribute.
    spec.check(this, attr);
  }

  @Check(CheckType.FAST)
  public void checkWidthSpec(WidthSpec widthSpec) {
    if (!this.target.supportsMultiports()) {
      error(
          "Multiports and banks are currently not supported by the given target.",
          Literals.WIDTH_SPEC__TERMS);
    } else {
      for (WidthTerm term : widthSpec.getTerms()) {
        if (term.getParameter() != null) {
          if (!this.target.supportsParameterizedWidths()) {
            error(
                "Parameterized widths are not supported by this target.",
                Literals.WIDTH_SPEC__TERMS);
          }
        } else if (term.getPort() != null) {
          // Widths given with {@code widthof()} are not supported (yet?).
          // This feature is currently only used for after delays.
          error("widthof is not supported.", Literals.WIDTH_SPEC__TERMS);
        } else if (term.getCode() != null) {
          if (this.target != Target.CPP) {
            error("This target does not support width given as code.", Literals.WIDTH_SPEC__TERMS);
          }
        } else if (term.getWidth() < 0) {
          error("Width must be a positive integer.", Literals.WIDTH_SPEC__TERMS);
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkReactorIconAttribute(Reactor reactor) {
    var path = AttributeUtils.getIconPath(reactor);
    if (path != null) {
      var param = AttributeUtils.findAttributeByName(reactor, "icon").getAttrParms().get(0);
      // Check file extension
      var validExtensions = Set.of("bmp", "png", "gif", "ico", "jpeg");
      var extensionStrart = path.lastIndexOf(".");
      var extension = extensionStrart != -1 ? path.substring(extensionStrart + 1) : "";
      if (!validExtensions.contains(extension.toLowerCase())) {
        warning(
            "File extension '"
                + extension
                + "' is not supported. Provide any of: "
                + String.join(", ", validExtensions),
            param,
            Literals.ATTR_PARM__VALUE);
        return;
      }

      // Check file location
      var iconLocation = FileUtil.locateFile(path, reactor.eResource());
      if (iconLocation == null) {
        warning("Cannot locate icon file.", param, Literals.ATTR_PARM__VALUE);
      }
      if (("file".equals(iconLocation.getScheme()) || iconLocation.getScheme() == null)
          && !(new File(iconLocation.getPath()).exists())) {
        warning("Icon does not exist.", param, Literals.ATTR_PARM__VALUE);
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkInitialMode(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      long initialModesCount = reactor.getModes().stream().filter(m -> m.isInitial()).count();
      if (initialModesCount == 0) {
        error("Every modal reactor requires one initial mode.", Literals.REACTOR__MODES, 0);
      } else if (initialModesCount > 1) {
        reactor.getModes().stream()
            .filter(m -> m.isInitial())
            .skip(1)
            .forEach(
                m -> {
                  error(
                      "A modal reactor can only have one initial mode.",
                      Literals.REACTOR__MODES,
                      reactor.getModes().indexOf(m));
                });
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkModeStateNamespace(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      var names = new ArrayList<String>();
      reactor.getStateVars().stream().map(it -> it.getName()).forEach(it -> names.add(it));
      for (var mode : reactor.getModes()) {
        for (var stateVar : mode.getStateVars()) {
          if (names.contains(stateVar.getName())) {
            error(
                String.format(
                    "Duplicate state variable '%s'. (State variables are currently scoped on"
                        + " reactor level not modes)",
                    stateVar.getName()),
                stateVar,
                Literals.STATE_VAR__NAME);
          }
          names.add(stateVar.getName());
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkModeTimerNamespace(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      var names = new ArrayList<String>();
      reactor.getTimers().stream().map(it -> it.getName()).forEach(it -> names.add(it));
      for (var mode : reactor.getModes()) {
        for (var timer : mode.getTimers()) {
          if (names.contains(timer.getName())) {
            error(
                String.format(
                    "Duplicate Timer '%s'. (Timers are currently scoped on reactor level not"
                        + " modes)",
                    timer.getName()),
                timer,
                Literals.VARIABLE__NAME);
          }
          names.add(timer.getName());
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkModeActionNamespace(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      var names = new ArrayList<String>();
      reactor.getActions().stream().map(it -> it.getName()).forEach(it -> names.add(it));
      for (var mode : reactor.getModes()) {
        for (var action : mode.getActions()) {
          if (names.contains(action.getName())) {
            error(
                String.format(
                    "Duplicate Action '%s'. (Actions are currently scoped on reactor level not"
                        + " modes)",
                    action.getName()),
                action,
                Literals.VARIABLE__NAME);
          }
          names.add(action.getName());
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkModeInstanceNamespace(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      var names = new ArrayList<String>();
      reactor.getActions().stream().map(it -> it.getName()).forEach(it -> names.add(it));
      for (var mode : reactor.getModes()) {
        for (var instantiation : mode.getInstantiations()) {
          if (names.contains(instantiation.getName())) {
            error(
                String.format(
                    "Duplicate Instantiation '%s'. (Instantiations are currently scoped on reactor"
                        + " level not modes)",
                    instantiation.getName()),
                instantiation,
                Literals.INSTANTIATION__NAME);
          }
          names.add(instantiation.getName());
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkMissingStateResetInMode(Reactor reactor) {
    if (!reactor.getModes().isEmpty()) {
      var resetModes = new HashSet<Mode>();
      // Collect all modes that may be reset
      for (var m : reactor.getModes()) {
        for (var r : m.getReactions()) {
          for (var e : r.getEffects()) {
            if (e.getVariable() instanceof Mode && e.getTransition() != ModeTransition.HISTORY) {
              resetModes.add((Mode) e.getVariable());
            }
          }
        }
      }
      for (var m : resetModes) {
        // Check state variables in this mode
        if (!m.getStateVars().isEmpty()) {
          var hasResetReaction =
              m.getReactions().stream()
                  .anyMatch(
                      r ->
                          r.getTriggers().stream()
                              .anyMatch(
                                  t ->
                                      (t instanceof BuiltinTriggerRef
                                          && ((BuiltinTriggerRef) t).getType()
                                              == BuiltinTrigger.RESET)));
          if (!hasResetReaction) {
            for (var s : m.getStateVars()) {
              if (!s.isReset()) {
                error(
                    "State variable is not reset upon mode entry. It is neither marked for"
                        + " automatic reset nor is there a reset reaction.",
                    m,
                    Literals.MODE__STATE_VARS,
                    m.getStateVars().indexOf(s));
              }
            }
          }
        }
        // Check state variables in instantiated reactors
        if (!m.getInstantiations().isEmpty()) {
          for (var i : m.getInstantiations()) {
            var error = new LinkedHashSet<StateVar>();
            var checked = new HashSet<Reactor>();
            var toCheck = new LinkedList<Reactor>();
            toCheck.add((Reactor) i.getReactorClass());
            while (!toCheck.isEmpty()) {
              var check = toCheck.pop();
              checked.add(check);
              if (!check.getStateVars().isEmpty()) {
                var hasResetReaction =
                    check.getReactions().stream()
                        .anyMatch(
                            r ->
                                r.getTriggers().stream()
                                    .anyMatch(
                                        t ->
                                            (t instanceof BuiltinTriggerRef
                                                && ((BuiltinTriggerRef) t).getType()
                                                    == BuiltinTrigger.RESET)));
                if (!hasResetReaction) {
                  // Add state vars that are not self-resetting to the error
                  check.getStateVars().stream()
                      .filter(s -> !s.isReset())
                      .forEachOrdered(error::add);
                }
              }
              // continue with inner
              for (var innerInstance : check.getInstantiations()) {
                var next = (Reactor) innerInstance.getReactorClass();
                if (!checked.contains(next)) {
                  toCheck.push(next);
                }
              }
            }
            if (!error.isEmpty()) {
              error(
                  "This reactor contains state variables that are not reset upon mode entry: "
                      + error.stream()
                          .map(
                              e -> e.getName() + " in " + ASTUtils.getEnclosingReactor(e).getName())
                          .collect(Collectors.joining(", "))
                      + ".\n"
                      + "The state variables are neither marked for automatic reset nor have a"
                      + " dedicated reset reaction. It is unsafe to instantiate this reactor inside"
                      + " a mode entered with reset.",
                  m,
                  Literals.MODE__INSTANTIATIONS,
                  m.getInstantiations().indexOf(i));
            }
          }
        }
      }
    }
  }

  @Check(CheckType.FAST)
  public void checkStateResetWithoutInitialValue(StateVar state) {
    if (state.isReset() && (state.getInit() == null || state.getInit().getExprs().isEmpty())) {
      error(
          "The state variable can not be automatically reset without an initial value.",
          state,
          Literals.STATE_VAR__RESET);
    }
  }

  @Check(CheckType.FAST)
  public void checkUnspecifiedTransitionType(Reaction reaction) {
    for (var effect : reaction.getEffects()) {
      var variable = effect.getVariable();
      if (variable instanceof Mode) {
        // The transition type is always set to default by Xtext.
        // Hence, check if there is an explicit node for the transition type in the AST.
        var transitionAssignment =
            NodeModelUtils.findNodesForFeature((EObject) effect, Literals.VAR_REF__TRANSITION);
        if (transitionAssignment.isEmpty()) { // Transition type not explicitly specified.
          var mode = (Mode) variable;
          // Check if reset or history transition would make a difference.
          var makesDifference =
              !mode.getStateVars().isEmpty()
                  || !mode.getTimers().isEmpty()
                  || !mode.getActions().isEmpty()
                  || mode.getConnections().stream().anyMatch(c -> c.getDelay() != null);
          if (!makesDifference && !mode.getInstantiations().isEmpty()) {
            // Also check instantiated reactors
            for (var i : mode.getInstantiations()) {
              var checked = new HashSet<Reactor>();
              var toCheck = new LinkedList<Reactor>();
              toCheck.add((Reactor) i.getReactorClass());
              while (!toCheck.isEmpty() && !makesDifference) {
                var check = toCheck.pop();
                checked.add(check);

                makesDifference |=
                    !check.getModes().isEmpty()
                        || !ASTUtils.allStateVars(check).isEmpty()
                        || !ASTUtils.allTimers(check).isEmpty()
                        || !ASTUtils.allActions(check).isEmpty()
                        || ASTUtils.allConnections(check).stream()
                            .anyMatch(c -> c.getDelay() != null);

                // continue with inner
                for (var innerInstance : check.getInstantiations()) {
                  var next = (Reactor) innerInstance.getReactorClass();
                  if (!checked.contains(next)) {
                    toCheck.push(next);
                  }
                }
              }
            }
          }
          if (makesDifference) {
            warning(
                "You should specify a transition type! "
                    + "Reset and history transitions have different effects on this target mode. "
                    + "Currently, a reset type is implicitly assumed.",
                reaction,
                Literals.REACTION__EFFECTS,
                reaction.getEffects().indexOf(effect));
          }
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////
  //// Public methods.

  /** Return the error reporter for this validator. */
  public ValidatorMessageReporter getErrorReporter() {
    return this.errorReporter;
  }

  /** Implementation required by xtext to report validation errors. */
  @Override
  public ValidationMessageAcceptor getMessageAcceptor() {
    return messageAcceptor == null ? this : messageAcceptor;
  }

  /** Report an error on the value of a target property */
  public void reportTargetPropertyError(String message) {
    this.targetPropertyErrors.add(message);
  }

  //////////////////////////////////////////////////////////////
  //// Protected methods.

  /** Generate an error message for an AST node. */
  @Override
  protected void error(java.lang.String message, org.eclipse.emf.ecore.EStructuralFeature feature) {
    super.error(message, feature);
  }

  //////////////////////////////////////////////////////////////
  //// Private methods.

  /**
   * For each input, report a conflict if: 1) the input exists and the type doesn't match; or 2) the
   * input has a name clash with variable that is not an input.
   *
   * @param superVars List of typed variables of a particular kind (i.e., inputs, outputs, or
   *     actions), found in a super class.
   * @param sameKind Typed variables of the same kind, found in the subclass.
   * @param allOwn Accumulator of non-conflicting variables incorporated in the subclass.
   * @param conflicts Set of variables that are in conflict, to be used by this function to report
   *     conflicts.
   */
  private <T extends TypedVariable> void checkConflict(
      EList<T> superVars, EList<T> sameKind, List<Variable> allOwn, HashSet<Variable> conflicts) {
    for (T superVar : superVars) {
      T match = null;
      for (T it : sameKind) {
        if (it.getName().equals(superVar.getName())) {
          match = it;
          break;
        }
      }
      List<Variable> rest = new ArrayList<>(allOwn);
      rest.removeIf(it -> sameKind.contains(it));

      if ((match != null && superVar.getType() != match.getType())
          || hasNameConflict(superVar, rest)) {
        conflicts.add(superVar);
      } else {
        allOwn.add(superVar);
      }
    }
  }

  /**
   * Check the name of a feature for illegal substrings such as reserved identifiers and names with
   * double leading underscores.
   *
   * @param name The name.
   * @param feature The feature containing the name (for error reporting).
   */
  private void checkName(String name, EStructuralFeature feature) {

    // Raises an error if the string starts with two underscores.
    if (name.length() >= 2 && name.substring(0, 2).equals("__")) {
      error(UNDERSCORE_MESSAGE + name, feature);
    }

    if (this.target.isReservedIdent(name)) {
      error(RESERVED_MESSAGE + name, feature);
    }

    if (this.target == Target.TS) {
      // "actions" is a reserved word within a TS reaction
      if (name.equals("actions")) {
        error(ACTIONS_MESSAGE + name, feature);
      }
    }
  }

  /**
   * Check that the initializer is compatible with the type. Note that if the type is inferred it
   * will necessarily be compatible so this method is not harmful.
   */
  public void typeCheck(Initializer init, InferredType type, EStructuralFeature feature) {
    if (init == null) {
      return;
    }

    // TODO:
    //  type is list => init is list
    //  type is fixed with size n => init is fixed with size n
    // Specifically for C: list can only be literal or time lists

    if (type.isTime) {
      if (type.isList) {
        // list of times
        var exprs = init.getExprs();
        if (exprs.isEmpty()) {
          error("Expected at least one time value.", feature);
          return;
        }
        if (exprs.size() == 1 && exprs.get(0) instanceof BracedListExpression) {
          exprs = ((BracedListExpression) exprs.get(0)).getItems();
        }
        for (var component : exprs) {
          checkExpressionIsTime(component, feature);
        }
      } else {
        checkExpressionIsTime(init, feature);
      }
    }
  }

  private void checkExpressionIsTime(Initializer init, EStructuralFeature feature) {
    if (init == null) {
      return;
    }

    if (init.getExprs().size() != 1) {
      error("Expected exactly one time value.", feature);
    } else {
      checkExpressionIsTime(ASTUtils.asSingleExpr(init), feature);
    }
  }

  private void checkExpressionIsTime(Expression value, EStructuralFeature feature) {
    if (value == null || value instanceof Time) {
      return;
    }

    if (value instanceof ParameterReference) {
      if (!ASTUtils.isOfTimeType(((ParameterReference) value).getParameter())
          && target.requiresTypes) {
        error("Referenced parameter is not of time type.", feature);
      }
      return;
    } else if (value instanceof Literal) {
      if (ASTUtils.isZero(((Literal) value).getLiteral())) {
        return;
      }

      if (ASTUtils.isInteger(((Literal) value).getLiteral())) {
        error("Missing time unit.", feature);
        return;
      }
      // fallthrough
    }

    error("Invalid time value.", feature);
  }

  /**
   * Return the number of main or federated reactors declared.
   *
   * @param iter An iterator over all objects in the resource.
   */
  private int countMainOrFederated(TreeIterator<EObject> iter) {
    int nMain = 0;
    while (iter.hasNext()) {
      EObject obj = iter.next();
      if (!(obj instanceof Reactor)) {
        continue;
      }
      Reactor r = (Reactor) obj;
      if (r.isMain() || r.isFederated()) {
        nMain++;
      }
    }
    return nMain;
  }

  /**
   * Report whether a given reactor has dependencies on a cyclic instantiation pattern. This means
   * the reactor has an instantiation in it -- directly or in one of its contained reactors -- that
   * is self-referential.
   *
   * @param reactor The reactor definition to find out whether it has any dependencies on cyclic
   *     instantiations.
   * @param cycleSet The set of all reactors that are part of an instantiation cycle.
   * @param visited The set of nodes already visited in this graph traversal.
   */
  private boolean dependsOnCycle(Reactor reactor, Set<Reactor> cycleSet, Set<Reactor> visited) {
    Set<Reactor> origins = info.instantiationGraph.getUpstreamAdjacentNodes(reactor);
    if (visited.contains(reactor)) {
      return false;
    } else {
      visited.add(reactor);
      for (Reactor it : origins) {
        if (cycleSet.contains(it) || dependsOnCycle(it, cycleSet, visited)) {
          // Reached a cycle.
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Report whether the name of the given element matches any variable in the ones to check against.
   *
   * @param element The element to compare against all variables in the given iterable.
   * @param toCheckAgainst Iterable variables to compare the given element against.
   */
  private boolean hasNameConflict(Variable element, Iterable<Variable> toCheckAgainst) {
    int numNameConflicts = 0;
    for (Variable it : toCheckAgainst) {
      if (it.getName().equals(element.getName())) {
        numNameConflicts++;
      }
    }
    return numNameConflicts > 0;
  }

  /** Return true if target is C or a C-based target like CCpp. */
  private boolean isCBasedTarget() {
    return (this.target == Target.C || this.target == Target.CCPP);
  }

  /**
   * Report whether a given imported reactor is used in this resource or not.
   *
   * @param reactor The imported reactor to check whether it is used.
   */
  private boolean isUnused(ImportedReactor reactor) {
    TreeIterator<EObject> instantiations = reactor.eResource().getAllContents();
    TreeIterator<EObject> subclasses = reactor.eResource().getAllContents();

    boolean instantiationsCheck = true;
    while (instantiations.hasNext() && instantiationsCheck) {
      EObject obj = instantiations.next();
      if (!(obj instanceof Instantiation)) {
        continue;
      }
      Instantiation inst = (Instantiation) obj;
      instantiationsCheck &=
          (inst.getReactorClass() != reactor
              && inst.getReactorClass() != reactor.getReactorClass());
    }

    boolean subclassesCheck = true;
    while (subclasses.hasNext() && subclassesCheck) {
      EObject obj = subclasses.next();
      if (!(obj instanceof Reactor)) {
        continue;
      }
      Reactor subclass = (Reactor) obj;
      for (ReactorDecl decl : subclass.getSuperClasses()) {
        subclassesCheck &= (decl != reactor && decl != reactor.getReactorClass());
      }
    }
    return instantiationsCheck && subclassesCheck;
  }

  /**
   * Return true if the two types match. Unfortunately, xtext does not seem to create a suitable
   * equals() method for Type, so we have to do this manually.
   */
  private boolean sameType(Type type1, Type type2) {
    if (type1 == null) {
      return type2 == null;
    }
    if (type2 == null) {
      return type1 == null;
    }
    // Most common case first.
    if (type1.getId() != null) {
      if (type1.getStars() != null) {
        if (type2.getStars() == null) {
          return false;
        }
        if (type1.getStars().size() != type2.getStars().size()) {
          return false;
        }
      }
      return (type1.getId().equals(type2.getId()));
    }

    // Type specification in the grammar is:
    // (time?='time' (arraySpec=ArraySpec)?) | ((id=(DottedName) (stars+='*')* ('<'
    // typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')? (arraySpec=ArraySpec)?) | code=Code);
    if (type1.isTime()) {
      if (!type2.isTime()) {
        return false;
      }
      // Ignore the arraySpec because that is checked when connection
      // is checked for balance.
      return true;
    }
    // Type must be given in a code body
    return type1.getCode().getBody().equals(type2.getCode().getBody());
  }

  //////////////////////////////////////////////////////////////
  //// Private fields.

  /** The error reporter. */
  private ValidatorMessageReporter errorReporter =
      new ValidatorMessageReporter(getMessageAcceptor(), new ValidatorStateAccess());

  /** Helper class containing information about the model. */
  private ModelInfo info = new ModelInfo();

  @Inject(optional = true)
  private ValidationMessageAcceptor messageAcceptor;

  /** The declared target. */
  private Target target;

  private List<String> targetPropertyErrors = new ArrayList<>();

  private List<String> targetPropertyWarnings = new ArrayList<>();

  //////////////////////////////////////////////////////////////
  //// Private static constants.

  private static String ACTIONS_MESSAGE =
      "\"actions\" is a reserved word for the TypeScript target for objects "
          + "(inputs, outputs, actions, timers, parameters, state, reactor definitions, "
          + "and reactor instantiation): ";

  private static String HOST_OR_FQN_REGEX =
      "^([a-z0-9]+(-[a-z0-9]+)*)|(([a-z0-9]+(-[a-z0-9]+)*\\.)+[a-z]{2,})$";

  /** Regular expression to check the validity of IPV4 addresses (due to David M. Syzdek). */
  private static String IPV4_REGEX =
      "((25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])\\.){3,3}"
          + "(25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])";

  /**
   * Regular expression to check the validity of IPV6 addresses (due to David M. Syzdek), with minor
   * adjustment to allow up to six IPV6 segments (without truncation) in front of an embedded
   * IPv4-address.
   */
  private static String IPV6_REGEX =
      "(([0-9a-fA-F]{1,4}:){7,7}[0-9a-fA-F]{1,4}|"
          + "([0-9a-fA-F]{1,4}:){1,7}:|"
          + "([0-9a-fA-F]{1,4}:){1,6}:[0-9a-fA-F]{1,4}|"
          + "([0-9a-fA-F]{1,4}:){1,5}(:[0-9a-fA-F]{1,4}){1,2}|"
          + "([0-9a-fA-F]{1,4}:){1,4}(:[0-9a-fA-F]{1,4}){1,3}|"
          + "([0-9a-fA-F]{1,4}:){1,3}(:[0-9a-fA-F]{1,4}){1,4}|"
          + "([0-9a-fA-F]{1,4}:){1,2}(:[0-9a-fA-F]{1,4}){1,5}|"
          + "[0-9a-fA-F]{1,4}:((:[0-9a-fA-F]{1,4}){1,6})|"
          + ":((:[0-9a-fA-F]{1,4}){1,7}|:)|"
          + "fe80:(:[0-9a-fA-F]{0,4}){0,4}%[0-9a-zA-Z]{1,}|"
          + "::(ffff(:0{1,4}){0,1}:){0,1}"
          + IPV4_REGEX
          + "|"
          + "([0-9a-fA-F]{1,4}:){1,4}:"
          + IPV4_REGEX
          + "|"
          + "([0-9a-fA-F]{1,4}:){1,6}"
          + IPV4_REGEX
          + ")";

  private static String RESERVED_MESSAGE =
      "Reserved words in the target language are not allowed for objects (inputs, outputs, actions,"
          + " timers, parameters, state, reactor definitions, and reactor instantiation): ";

  private static List<String> SPACING_VIOLATION_POLICIES = List.of("defer", "drop", "replace");

  private static String UNDERSCORE_MESSAGE =
      "Names of objects (inputs, outputs, actions, timers, parameters, "
          + "state, reactor definitions, and reactor instantiation) may not start with \"__\": ";

  private static String USERNAME_REGEX = "^[a-z_]([a-z0-9_-]{0,31}|[a-z0-9_-]{0,30}\\$)$";
}
