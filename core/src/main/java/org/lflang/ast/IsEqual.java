package org.lflang.ast;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.EObject;
import org.lflang.TimeUnit;
import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.Assignment;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.CStyleArraySpec;
import org.lflang.lf.Code;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Element;
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
import org.lflang.lf.Literal;
import org.lflang.lf.Method;
import org.lflang.lf.MethodArgument;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.NamedHost;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.ParenthesisListExpression;
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
import org.lflang.lf.TypeParm;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.lf.util.LfSwitch;

/**
 * Switch class that checks if subtrees of the AST are semantically equivalent to each other. Return
 * {@code false} if they are not equivalent; return {@code true} or {@code false} (but preferably
 * {@code true}) if they are equivalent.
 */
public class IsEqual extends LfSwitch<Boolean> {

  private final EObject otherObject;

  public IsEqual(EObject other) {
    this.otherObject = other;
  }

  @Override
  public Boolean doSwitch(EObject eObject) {
    if (otherObject == eObject) return true;
    if (eObject == null) return false;
    return super.doSwitch(eObject);
  }

  @Override
  public Boolean caseModel(Model object) {
    return new ComparisonMachine<>(object, Model.class)
        .equivalent(Model::getTarget)
        .listsEquivalent(Model::getImports)
        .listsEquivalent(Model::getPreambles)
        .listsEquivalent(Model::getReactors)
        .conclusion;
  }

  @Override
  public Boolean caseImport(Import object) {
    return new ComparisonMachine<>(object, Import.class)
        .equalAsObjects(Import::getImportURI)
        .listsEquivalent(Import::getReactorClasses)
        .conclusion;
  }

  @Override
  public Boolean caseReactorDecl(ReactorDecl object) {
    return new ComparisonMachine<>(object, ReactorDecl.class)
        .equalAsObjects(ReactorDecl::getName)
        .conclusion;
  }

  @Override
  public Boolean caseImportedReactor(ImportedReactor object) {
    return new ComparisonMachine<>(object, ImportedReactor.class)
        .equalAsObjects(ImportedReactor::getName)
        .equivalent(ImportedReactor::getReactorClass)
        .conclusion;
  }

  @Override
  public Boolean caseReactor(Reactor object) {
    return new ComparisonMachine<>(object, Reactor.class)
        .listsEquivalent(Reactor::getAttributes)
        .equalAsObjects(Reactor::isFederated)
        .equalAsObjects(Reactor::isRealtime)
        .equalAsObjects(Reactor::isMain)
        .equalAsObjects(Reactor::getName)
        .listsEquivalent(Reactor::getTypeParms)
        .listsEquivalent(Reactor::getParameters)
        .equivalent(Reactor::getHost)
        .listsEquivalent(Reactor::getSuperClasses)
        .listsEquivalent(Reactor::getPreambles)
        .listsEquivalent(Reactor::getInputs)
        .listsEquivalent(Reactor::getOutputs)
        .listsEquivalent(Reactor::getTimers)
        .listsEquivalent(Reactor::getActions)
        .listsEquivalent(Reactor::getInstantiations)
        .listsEquivalent(Reactor::getConnections)
        .listsEquivalent(Reactor::getStateVars)
        .listsEquivalent(Reactor::getReactions)
        .listsEquivalent(Reactor::getMethods)
        .listsEquivalent(Reactor::getModes)
        .conclusion;
  }

  @Override
  public Boolean caseTypeParm(TypeParm object) {
    return new ComparisonMachine<>(object, TypeParm.class)
        .equalAsObjects(TypeParm::getLiteral)
        .equivalent(TypeParm::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseTargetDecl(TargetDecl object) {
    return new ComparisonMachine<>(object, TargetDecl.class)
        .equalAsObjects(TargetDecl::getName)
        .equivalentModulo(
            TargetDecl::getConfig,
            (KeyValuePairs it) -> it != null && it.getPairs().isEmpty() ? null : it)
        .conclusion;
  }

  @Override
  public Boolean caseStateVar(StateVar object) {
    return new ComparisonMachine<>(object, StateVar.class)
        .listsEquivalent(StateVar::getAttributes)
        .equalAsObjects(StateVar::getName)
        .equivalent(StateVar::getType)
        .equivalent(StateVar::getInit)
        .conclusion;
  }

  @Override
  public Boolean caseInitializer(Initializer object) {
    // Empty braces are not equivalent to no init.
    return new ComparisonMachine<>(object, Initializer.class)
        .equalAsObjects(Initializer::isAssign)
        .equivalent(Initializer::getExpr)
        .conclusion;
  }

  @Override
  public Boolean caseMethod(Method object) {
    return new ComparisonMachine<>(object, Method.class)
        .equalAsObjects(Method::isConst)
        .equalAsObjects(Method::getName)
        .listsEquivalent(Method::getArguments)
        .equivalent(Method::getReturn)
        .equivalent(Method::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseMethodArgument(MethodArgument object) {
    return new ComparisonMachine<>(object, MethodArgument.class)
        .equalAsObjects(MethodArgument::getName)
        .equivalent(MethodArgument::getType)
        .conclusion;
  }

  @Override
  public Boolean caseInput(Input object) {
    return new ComparisonMachine<>(object, Input.class)
        .listsEquivalent(Input::getAttributes)
        .equalAsObjects(Input::isMutable)
        .equivalent(Input::getWidthSpec)
        .equivalent(Input::getType)
        .conclusion;
  }

  @Override
  public Boolean caseOutput(Output object) {
    return new ComparisonMachine<>(object, Output.class)
        .listsEquivalent(Output::getAttributes)
        .equivalent(Output::getWidthSpec)
        .equalAsObjects(Output::getName)
        .equivalent(Output::getType)
        .conclusion;
  }

  @Override
  public Boolean caseTimer(Timer object) {
    return new ComparisonMachine<>(object, Timer.class)
        .listsEquivalent(Timer::getAttributes)
        .equalAsObjects(Timer::getName)
        .equivalent(Timer::getOffset)
        .equivalent(Timer::getPeriod)
        .conclusion;
  }

  @Override
  public Boolean caseMode(Mode object) {
    return new ComparisonMachine<>(object, Mode.class)
        .equalAsObjects(Mode::isInitial)
        .equalAsObjects(Mode::getName)
        .listsEquivalent(Mode::getStateVars)
        .listsEquivalent(Mode::getTimers)
        .listsEquivalent(Mode::getActions)
        .listsEquivalent(Mode::getInstantiations)
        .listsEquivalent(Mode::getConnections)
        .listsEquivalent(Mode::getReactions)
        .conclusion;
  }

  @Override
  public Boolean caseAction(Action object) {
    return new ComparisonMachine<>(object, Action.class)
        .listsEquivalent(Action::getAttributes)
        .equalAsObjects(Action::getOrigin) // This is an enum
        .equalAsObjects(Action::getName)
        .equivalent(Action::getMinDelay)
        .equivalent(Action::getMinSpacing)
        .equalAsObjects(Action::getPolicy)
        .equivalent(Action::getType)
        .conclusion;
  }

  @Override
  public Boolean caseAttribute(Attribute object) {
    return new ComparisonMachine<>(object, Attribute.class)
        .equalAsObjects(Attribute::getAttrName)
        .listsEquivalent(Attribute::getAttrParms)
        .conclusion;
  }

  @Override
  public Boolean caseAttrParm(AttrParm object) {
    return new ComparisonMachine<>(object, AttrParm.class)
        .equalAsObjects(AttrParm::getName)
        .equalAsObjects(AttrParm::getValue)
        .conclusion;
  }

  @Override
  public Boolean caseReaction(Reaction object) {
    return new ComparisonMachine<>(object, Reaction.class)
        .listsEquivalent(Reaction::getAttributes)
        .listsEquivalent(Reaction::getTriggers)
        .listsEquivalent(Reaction::getSources)
        .listsEquivalent(Reaction::getEffects)
        .equalAsObjects(Reaction::isMutation)
        .equalAsObjects(Reaction::getName)
        .equivalent(Reaction::getCode)
        .equivalent(Reaction::getStp)
        .equivalent(Reaction::getDeadline)
        .conclusion;
  }

  @Override
  public Boolean caseTriggerRef(TriggerRef object) {
    throw thereIsAMoreSpecificCase(TriggerRef.class, BuiltinTriggerRef.class, VarRef.class);
  }

  @Override
  public Boolean caseBuiltinTriggerRef(BuiltinTriggerRef object) {
    return new ComparisonMachine<>(object, BuiltinTriggerRef.class)
        .equalAsObjects(BuiltinTriggerRef::getType) // This is an enum
        .conclusion;
  }

  @Override
  public Boolean caseDeadline(Deadline object) {
    return new ComparisonMachine<>(object, Deadline.class)
        .equivalent(Deadline::getDelay)
        .equivalent(Deadline::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseSTP(STP object) {
    return new ComparisonMachine<>(object, STP.class)
        .equivalent(STP::getValue)
        .equivalent(STP::getCode)
        .conclusion;
  }

  @Override
  public Boolean casePreamble(Preamble object) {
    return new ComparisonMachine<>(object, Preamble.class)
        .equalAsObjects(Preamble::getVisibility) // This is an enum
        .equivalent(Preamble::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseInstantiation(Instantiation object) {
    return new ComparisonMachine<>(object, Instantiation.class)
        .equalAsObjects(Instantiation::getName)
        .equivalent(Instantiation::getWidthSpec)
        .equivalent(Instantiation::getReactorClass)
        .listsEquivalent(Instantiation::getTypeArgs)
        .listsEquivalent(Instantiation::getParameters)
        .equivalent(Instantiation::getHost)
        .conclusion;
  }

  @Override
  public Boolean caseConnection(Connection object) {
    return new ComparisonMachine<>(object, Connection.class)
        .listsEquivalent(Connection::getLeftPorts)
        .equalAsObjects(Connection::isIterated)
        .equalAsObjects(Connection::isPhysical)
        .listsEquivalent(Connection::getRightPorts)
        .equivalent(Connection::getDelay)
        .equivalent(Connection::getSerializer)
        .conclusion;
  }

  @Override
  public Boolean caseSerializer(Serializer object) {
    return new ComparisonMachine<>(object, Serializer.class)
        .equalAsObjects(Serializer::getType)
        .conclusion;
  }

  @Override
  public Boolean caseKeyValuePairs(KeyValuePairs object) {
    return new ComparisonMachine<>(object, KeyValuePairs.class)
        .listsEquivalent(KeyValuePairs::getPairs)
        .conclusion;
  }

  @Override
  public Boolean caseKeyValuePair(KeyValuePair object) {
    return new ComparisonMachine<>(object, KeyValuePair.class)
        .equalAsObjects(KeyValuePair::getName)
        .equivalent(KeyValuePair::getValue)
        .conclusion;
  }

  @Override
  public Boolean caseArray(Array object) {
    return new ComparisonMachine<>(object, Array.class)
        .listsEquivalent(Array::getElements)
        .conclusion;
  }

  @Override
  public Boolean caseElement(Element object) {
    return new ComparisonMachine<>(object, Element.class)
        .equivalent(Element::getKeyvalue)
        .equivalent(Element::getArray)
        .equalAsObjects(Element::getLiteral)
        .equalAsObjects(Element::getId)
        .equalAsObjects(Element::getUnit)
        .conclusion;
  }

  @Override
  public Boolean caseTypedVariable(TypedVariable object) {
    throw thereIsAMoreSpecificCase(TypedVariable.class, Port.class, Action.class);
  }

  @Override
  public Boolean caseVariable(Variable object) {
    throw thereIsAMoreSpecificCase(Variable.class, TypedVariable.class, Timer.class, Mode.class);
  }

  @Override
  public Boolean caseVarRef(VarRef object) {
    return new ComparisonMachine<>(object, VarRef.class)
        .equalAsObjects(
            varRef -> varRef.getVariable() instanceof Mode ? null : varRef.getVariable().getName())
        .equivalent(varRef -> varRef.getVariable() instanceof Mode ? null : varRef.getVariable())
        .equalAsObjects(
            varRef -> varRef.getContainer() == null ? null : varRef.getContainer().getName())
        .equalAsObjects(VarRef::isInterleaved)
        .equalAsObjects(VarRef::getTransition)
        .conclusion;
  }

  @Override
  public Boolean caseAssignment(Assignment object) {
    return new ComparisonMachine<>(object, Assignment.class)
        .equivalent(Assignment::getLhs)
        .equivalent(Assignment::getRhs)
        .conclusion;
  }

  @Override
  public Boolean caseParameter(Parameter object) {
    return new ComparisonMachine<>(object, Parameter.class)
        .listsEquivalent(Parameter::getAttributes)
        .equalAsObjects(Parameter::getName)
        .equivalent(Parameter::getType)
        .equivalent(Parameter::getInit)
        .conclusion;
  }

  @Override
  public Boolean caseExpression(Expression object) {
    throw thereIsAMoreSpecificCase(
        Expression.class,
        Literal.class,
        Time.class,
        ParameterReference.class,
        Code.class,
        BracedListExpression.class,
        BracketListExpression.class,
        ParenthesisListExpression.class);
  }

  @Override
  public Boolean caseBracedListExpression(BracedListExpression object) {
    return new ComparisonMachine<>(object, BracedListExpression.class)
        .listsEquivalent(BracedListExpression::getItems)
        .conclusion;
  }

  @Override
  public Boolean caseBracketListExpression(BracketListExpression object) {
    return new ComparisonMachine<>(object, BracketListExpression.class)
        .listsEquivalent(BracketListExpression::getItems)
        .conclusion;
  }

  @Override
  public Boolean caseParenthesisListExpression(ParenthesisListExpression object) {
    return new ComparisonMachine<>(object, ParenthesisListExpression.class)
        .listsEquivalent(ParenthesisListExpression::getItems)
        .conclusion;
  }

  @Override
  public Boolean caseParameterReference(ParameterReference object) {
    return new ComparisonMachine<>(object, ParameterReference.class)
        .equivalent(ParameterReference::getParameter)
        .conclusion;
  }

  @Override
  public Boolean caseTime(Time object) {
    return new ComparisonMachine<>(object, Time.class)
        .equalAsObjects(Time::getInterval)
        .equalAsObjectsModulo(
            Time::getUnit,
            ((Function<TimeUnit, String>) TimeUnit::getCanonicalName).compose(TimeUnit::fromName))
        .conclusion;
  }

  @Override
  public Boolean casePort(Port object) {
    throw thereIsAMoreSpecificCase(Port.class, Input.class, Output.class);
  }

  @Override
  public Boolean caseType(Type object) {
    return new ComparisonMachine<>(object, Type.class)
        .equivalent(Type::getCode)
        .equalAsObjects(Type::isTime)
        .equivalent(Type::getCStyleArraySpec)
        .equalAsObjects(Type::getId)
        .listsEquivalent(Type::getTypeArgs)
        .listsEqualAsObjects(Type::getStars)
        .equivalent(Type::getCStyleArraySpec)
        .equivalent(Type::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseCStyleArraySpec(CStyleArraySpec object) {
    return new ComparisonMachine<>(object, CStyleArraySpec.class)
        .equalAsObjects(CStyleArraySpec::isOfVariableLength)
        .equalAsObjects(CStyleArraySpec::getLength)
        .conclusion;
  }

  @Override
  public Boolean caseWatchdog(Watchdog object) {
    return new ComparisonMachine<>(object, Watchdog.class)
        .equalAsObjects(Watchdog::getName)
        .equivalent(Watchdog::getTimeout)
        .listsEquivalent(Watchdog::getEffects)
        .equivalent(Watchdog::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseWidthSpec(WidthSpec object) {
    return new ComparisonMachine<>(object, WidthSpec.class)
        .equalAsObjects(WidthSpec::isOfVariableLength)
        .listsEquivalent(WidthSpec::getTerms)
        .conclusion;
  }

  @Override
  public Boolean caseWidthTerm(WidthTerm object) {
    return new ComparisonMachine<>(object, WidthTerm.class)
        .equalAsObjects(WidthTerm::getWidth)
        .equivalent(WidthTerm::getParameter)
        .equivalent(WidthTerm::getPort)
        .equivalent(WidthTerm::getCode)
        .conclusion;
  }

  @Override
  public Boolean caseIPV4Host(IPV4Host object) {
    return caseHost(object);
  }

  @Override
  public Boolean caseIPV6Host(IPV6Host object) {
    return caseHost(object);
  }

  @Override
  public Boolean caseNamedHost(NamedHost object) {
    return caseHost(object);
  }

  @Override
  public Boolean caseHost(Host object) {
    return new ComparisonMachine<>(object, Host.class)
        .equalAsObjects(Host::getUser)
        .equalAsObjects(Host::getAddr)
        .equalAsObjects(Host::getPort)
        .conclusion;
  }

  @Override
  public Boolean caseCodeExpr(CodeExpr object) {
    return new ComparisonMachine<>(object, CodeExpr.class).equivalent(CodeExpr::getCode).conclusion;
  }

  @Override
  public Boolean caseCode(Code object) {
    return new ComparisonMachine<>(object, Code.class)
        .equalAsObjectsModulo(Code::getBody, s -> s == null ? null : s.strip().stripIndent())
        .conclusion;
  }

  @Override
  public Boolean caseLiteral(Literal object) {
    return new ComparisonMachine<>(object, Literal.class)
        .equalAsObjects(Literal::getLiteral)
        .conclusion;
  }

  @Override
  public Boolean defaultCase(EObject object) {
    return super.defaultCase(object);
  }

  @SafeVarargs
  private UnsupportedOperationException thereIsAMoreSpecificCase(
      Class<? extends EObject> thisCase, Class<? extends EObject>... moreSpecificCases) {
    return new UnsupportedOperationException(
        String.format(
            "%ss are %s, so the methods "
                + "corresponding to those types should be invoked instead.",
            thisCase.getName(),
            Arrays.stream(moreSpecificCases)
                .map(Class::getName)
                .map(it -> it + (it.endsWith("s") ? "es" : "s"))
                .collect(Collectors.joining(" or "))));
  }

  private static <T> boolean listsEqualish(
      List<T> list0, List<T> list1, BiPredicate<T, T> equalish) {
    if (list0 == list1) return true; // e.g., they are both null
    if (list0.size() != list1.size()) return false;
    for (int i = 0; i < list0.size(); i++) {
      if (!equalish.test(list0.get(i), list1.get(i))) {
        return false;
      }
    }
    return true;
  }

  /** Fluently compare a pair of parse tree nodes for equivalence. */
  private class ComparisonMachine<E extends EObject> {
    private final E object;
    private final E other;
    private boolean conclusion;

    ComparisonMachine(E object, Class<E> clz) {
      this.object = object;
      this.conclusion = clz.isInstance(otherObject);
      this.other = conclusion ? clz.cast(otherObject) : null;
    }

    /** Conclude false if the two given Lists are different as EObject sequences. Order matters. */
    <T extends EObject> ComparisonMachine<E> listsEquivalent(Function<E, List<T>> listGetter) {
      if (conclusion)
        conclusion = listsEqualish(listGetter, (T a, T b) -> new IsEqual(a).doSwitch(b));
      return this;
    }

    /** Conclude false if the two given Lists are different as object sequences. Order matters. */
    <T> ComparisonMachine<E> listsEqualAsObjects(Function<E, List<T>> listGetter) {
      if (conclusion) conclusion = listsEqualish(listGetter, Objects::equals);
      return this;
    }

    <T> boolean listsEqualish(
        Function<E, ? extends List<T>> listGetter, BiPredicate<T, T> equalish) {
      if (!conclusion) return false;
      List<T> list0 = listGetter.apply(object);
      List<T> list1 = listGetter.apply(other);
      return IsEqual.listsEqualish(list0, list1, equalish);
    }

    /** Conclude false if the two properties are not equal as objects. */
    <T> ComparisonMachine<E> equalAsObjects(Function<E, T> propertyGetter) {
      return equalAsObjectsModulo(propertyGetter, Function.identity());
    }

    /**
     * Conclude false if the two properties are not equal as objects, given that {@code
     * projectionToClassRepresentatives} maps each object to some semantically equivalent object.
     */
    <T> ComparisonMachine<E> equalAsObjectsModulo(
        Function<E, T> propertyGetter, Function<T, T> projectionToClassRepresentatives) {
      if (propertyGetter.apply(object) instanceof EObject) {
        throw new IllegalArgumentException(
            "EObjects should be compared for semantic equivalence, not object equality.");
      }
      propertyGetter = projectionToClassRepresentatives.compose(propertyGetter);
      if (conclusion)
        conclusion = Objects.equals(propertyGetter.apply(object), propertyGetter.apply(other));
      return this;
    }

    /** Conclude false if the two properties are not semantically equivalent parse nodes. */
    <T extends EObject> ComparisonMachine<E> equivalent(Function<E, T> propertyGetter) {
      return equivalentModulo(propertyGetter, Function.identity());
    }

    /**
     * Conclude false if the two properties are not semantically equivalent parse nodes, given that
     * {@code projectionToClassRepresentatives} maps each parse node to some semantically equivalent
     * node.
     */
    <T extends EObject> ComparisonMachine<E> equivalentModulo(
        Function<E, T> propertyGetter, Function<T, T> projectionToClassRepresentatives) {
      propertyGetter = projectionToClassRepresentatives.compose(propertyGetter);
      if (conclusion)
        conclusion =
            new IsEqual(propertyGetter.apply(object)).doSwitch(propertyGetter.apply(other));
      return this;
    }
  }
}
