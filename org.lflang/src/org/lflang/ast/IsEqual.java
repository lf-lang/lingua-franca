package org.lflang.ast;

import java.util.Objects;
import java.util.function.Function;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;

import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.ArraySpec;
import org.lflang.lf.Assignment;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Code;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Element;
import org.lflang.lf.Expression;
import org.lflang.lf.Host;
import org.lflang.lf.IPV4Host;
import org.lflang.lf.IPV6Host;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.Literal;
import org.lflang.lf.Method;
import org.lflang.lf.MethodArgument;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Mutation;
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
import org.lflang.lf.TypeParm;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.lf.util.LfSwitch;

/**
 * Switch class that checks if subtrees of the AST are semantically equivalent
 * to each other. Return {@code false} if they are not equivalent; return
 * {@code true} or {@code false} (but preferably {@code true}) if they are
 * equivalent.
 */
public class IsEqual extends LfSwitch<Boolean> {

    private final EObject otherObject;

    public IsEqual(EObject other) {
        this.otherObject = other;
    }

    @Override
    protected Boolean doSwitch(int classifierID, EObject theEObject) {
        if (otherObject == theEObject) return true;
        return super.doSwitch(classifierID, theEObject);
    }

    @Override
    public Boolean caseModel(Model object) {
        return otherObject instanceof Model other
            && new ComparisonMachine<>(object, other)
            .equivalent(Model::getTarget)
            .listsEqual(Model::getImports)
            .listsEqual(Model::getPreambles)
            .listsEqual(Model::getReactors).conclusion;
    }

    @Override
    public Boolean caseImport(Import object) {
        return otherObject instanceof Import other
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(Import::getImportURI)
            .listsEqual(Import::getReactorClasses).conclusion;
    }

    @Override
    public Boolean caseReactorDecl(ReactorDecl object) {
        return otherObject instanceof ReactorDecl other
            && object.getName().equals(other.getName());
    }

    @Override
    public Boolean caseImportedReactor(ImportedReactor object) {
        return otherObject instanceof ImportedReactor other
            && object.getName().equals(other.getName())
            && new IsEqual(object.getReactorClass()).doSwitch(other.getReactorClass());
    }

    @Override
    public Boolean caseReactor(Reactor object) {
        return otherObject instanceof Reactor other
            && new ComparisonMachine<>(object, other)
            .identical(Reactor::isFederated)
            .identical(Reactor::isRealtime)
            .identical(Reactor::isMain)
            .equalAsObjects(Reactor::getName)
            .listsEqual(Reactor::getTypeParms)
            .listsEqual(Reactor::getParameters)
            .equivalent(Reactor::getHost)
            .listsEqual(Reactor::getSuperClasses)
            .listsEqual(Reactor::getPreambles)
            .listsEqual(Reactor::getInputs)
            .listsEqual(Reactor::getOutputs)
            .listsEqual(Reactor::getTimers)
            .listsEqual(Reactor::getActions)
            .listsEqual(Reactor::getInstantiations)
            .listsEqual(Reactor::getConnections)
            .listsEqual(Reactor::getStateVars)
            .listsEqual(Reactor::getReactions)
            .listsEqual(Reactor::getMethods)
            .listsEqual(Reactor::getMutations)
            .listsEqual(Reactor::getModes)
            .conclusion;
    }

    @Override
    public Boolean caseTypeParm(TypeParm object) {
        return otherObject instanceof TypeParm other
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(TypeParm::getLiteral)
            .equivalent(TypeParm::getCode)
            .conclusion;
    }

    @Override
    public Boolean caseTargetDecl(TargetDecl object) {
        return otherObject instanceof TargetDecl other
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(TargetDecl::getName)
            .equivalent(TargetDecl::getConfig)
            .conclusion;
    }

    @Override
    public Boolean caseStateVar(StateVar object) {
        return otherObject instanceof StateVar other
            && other.getBraces().size() == object.getBraces().size()
            && other.getParens().size() == object.getParens().size()
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(StateVar::getName)
            .equalAsObjects(StateVar::getType)
            .listsEqual(StateVar::getInit)
            .conclusion;
    }

    @Override
    public Boolean caseMethod(Method object) {
        return otherObject instanceof Method other
            && new ComparisonMachine<>(object, other)
            .identical(Method::isConst)
            .equalAsObjects(Method::getName)
            .listsEqual(Method::getArguments)
            .equivalent(Method::getReturn)
            .equivalent(Method::getCode)
            .conclusion;
    }

    @Override
    public Boolean caseMethodArgument(MethodArgument object) {
        return otherObject instanceof MethodArgument other
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(MethodArgument::getName)
            .equivalent(MethodArgument::getType)
            .conclusion;
    }

    @Override
    public Boolean caseInput(Input object) {
        return otherObject instanceof Input other
            && new ComparisonMachine<>(object, other)
            .identical(Input::isMutable)
            .equivalent(Input::getWidthSpec)
            .equivalent(Input::getType)
            .conclusion;
    }

    @Override
    public Boolean caseOutput(Output object) {
        return otherObject instanceof Output other
            && new ComparisonMachine<>(object, other)
            .equivalent(Output::getWidthSpec)
            .equalAsObjects(Output::getName)
            .equivalent(Output::getType)
            .conclusion;
    }

    @Override
    public Boolean caseTimer(Timer object) {
        return otherObject instanceof Timer other
            && new ComparisonMachine<>(object, other)
            .equalAsObjects(Timer::getName)
            .equivalent(Timer::getOffset)
            .equivalent(Timer::getPeriod)
            .conclusion;
    }

    @Override
    public Boolean caseMode(Mode object) {
        return otherObject instanceof Mode other
            && new ComparisonMachine<>(object, other)
            .identical(Mode::isInitial)
            .equalAsObjects(Mode::getName)
            .listsEqual(Mode::getStateVars)
            .listsEqual(Mode::getTimers)
            .listsEqual(Mode::getActions)
            .listsEqual(Mode::getInstantiations)
            .listsEqual(Mode::getConnections)
            .listsEqual(Mode::getReactions)
            .conclusion;
    }

    @Override
    public Boolean caseAction(Action object) {
        return otherObject instanceof Action other
            && new ComparisonMachine<>(object, other)
            .identical(Action::getOrigin) // This is an enum
            .equalAsObjects(Action::getName)
            .equivalent(Action::getMinDelay)
            .equivalent(Action::getMinSpacing)
            .equalAsObjects(Action::getPolicy)
            .equivalent(Action::getType)
            .conclusion;
    }

    @Override
    public Boolean caseReaction(Reaction object) {
        return otherObject instanceof Reaction other
            && new ComparisonMachine<>(object, other)
            .listsEqual(Reaction::getTriggers)
            .listsEqual(Reaction::getSources)
            .listsEqual(Reaction::getEffects)
            .equivalent(Reaction::getCode)
            .equivalent(Reaction::getStp)
            .equivalent(Reaction::getDeadline)
            .conclusion;
    }

    @Override
    public Boolean caseTriggerRef(TriggerRef object) {
        throw new UnsupportedOperationException(
            "TriggerRefs are BuiltinTriggerRefs or VarRefs, so the methods "
                + "corresponding to those types should be invoked instead.");
    }

    @Override
    public Boolean caseBuiltinTriggerRef(BuiltinTriggerRef object) {
        return otherObject instanceof BuiltinTriggerRef other
            && new ComparisonMachine<>(object, other)
            .identical(BuiltinTriggerRef::getType)  // This is an enum
            .conclusion;
    }

    @Override
    public Boolean caseDeadline(Deadline object) {
        return super.caseDeadline(object);
    }

    @Override
    public Boolean caseSTP(STP object) {
        return super.caseSTP(object);
    }

    @Override
    public Boolean caseMutation(Mutation object) {
        return super.caseMutation(object);
    }

    @Override
    public Boolean casePreamble(Preamble object) {
        return super.casePreamble(object);
    }

    @Override
    public Boolean caseInstantiation(Instantiation object) {
        return super.caseInstantiation(object);
    }

    @Override
    public Boolean caseConnection(Connection object) {
        return super.caseConnection(object);
    }

    @Override
    public Boolean caseSerializer(Serializer object) {
        return super.caseSerializer(object);
    }

    @Override
    public Boolean caseKeyValuePairs(KeyValuePairs object) {
        return super.caseKeyValuePairs(object);
    }

    @Override
    public Boolean caseKeyValuePair(KeyValuePair object) {
        return super.caseKeyValuePair(object);
    }

    @Override
    public Boolean caseArray(Array object) {
        return super.caseArray(object);
    }

    @Override
    public Boolean caseElement(Element object) {
        return super.caseElement(object);
    }

    @Override
    public Boolean caseTypedVariable(TypedVariable object) {
        return super.caseTypedVariable(object);
    }

    @Override
    public Boolean caseVariable(Variable object) {
        return super.caseVariable(object);
    }

    @Override
    public Boolean caseVarRef(VarRef object) {
        return super.caseVarRef(object);
    }

    @Override
    public Boolean caseAssignment(Assignment object) {
        return super.caseAssignment(object);
    }

    @Override
    public Boolean caseParameter(Parameter object) {
        return super.caseParameter(object);
    }

    @Override
    public Boolean caseExpression(Expression object) {
        return super.caseExpression(object);
    }

    @Override
    public Boolean caseParameterReference(ParameterReference object) {
        return super.caseParameterReference(object);
    }

    @Override
    public Boolean caseTime(Time object) {
        return super.caseTime(object);
    }

    @Override
    public Boolean casePort(Port object) {
        return super.casePort(object);
    }

    @Override
    public Boolean caseType(Type object) {
        return super.caseType(object);
    }

    @Override
    public Boolean caseArraySpec(ArraySpec object) {
        return super.caseArraySpec(object);
    }

    @Override
    public Boolean caseWidthSpec(WidthSpec object) {
        return super.caseWidthSpec(object);
    }

    @Override
    public Boolean caseWidthTerm(WidthTerm object) {
        return super.caseWidthTerm(object);
    }

    @Override
    public Boolean caseIPV4Host(IPV4Host object) {
        return super.caseIPV4Host(object);
    }

    @Override
    public Boolean caseIPV6Host(IPV6Host object) {
        return super.caseIPV6Host(object);
    }

    @Override
    public Boolean caseNamedHost(NamedHost object) {
        return super.caseNamedHost(object);
    }

    @Override
    public Boolean caseHost(Host object) {
        return super.caseHost(object);
    }

    @Override
    public Boolean caseCode(Code object) {
        return super.caseCode(object);
    }

    @Override
    public Boolean caseLiteral(Literal object) {
        return super.caseLiteral(object);
    }

    @Override
    public Boolean defaultCase(EObject object) {
        return super.defaultCase(object);
    }

    /** Fluently compare a pair of parse tree nodes for equivalence. */
    private static class ComparisonMachine<E extends EObject> {
        private final E object;
        private final E other;
        private boolean conclusion = true;

        ComparisonMachine(E object, E other) {
            this.object = object;
            this.other = other;
        }

        /** Conclude false if the two given ELists are different. Order matters. */
        <T extends EObject> ComparisonMachine<E> listsEqual(Function<E, EList<T>> listGetter) {
            if (!conclusion) return this;
            var list0 = listGetter.apply(object);
            var list1 = listGetter.apply(other);
            if (list0.size() != list1.size()) {
                conclusion = false;
                return this;
            }
            for (int i = 0; i < list0.size(); i++) {
                if (!new IsEqual(list0.get(i)).doSwitch(list1.get(i))) {
                    conclusion = false;
                    return this;
                }
            }
            return this;
        }

        /**
         * Conclude false if the two properties are not equal as pointers or
         * as primitives.
         */
        ComparisonMachine<E> identical(Function<E, ?> propertyGetter) {
            if (conclusion) conclusion = propertyGetter.apply(object) == propertyGetter.apply(other);
            return this;
        }

        /** Conclude false if the two properties are not equal as objects. */
        <T> ComparisonMachine<E> equalAsObjects(Function<E, T> propertyGetter) {
            if (conclusion) conclusion = Objects.equals(propertyGetter.apply(object), propertyGetter.apply(other));
            return this;
        }

        /**
         * Conclude false if the two properties are not semantically equivalent
         * parse nodes.
         */
        <T extends EObject> ComparisonMachine<E> equivalent(Function<E, T> propertyGetter) {
            if (conclusion) conclusion = new IsEqual(propertyGetter.apply(object))
                .doSwitch(propertyGetter.apply(other));
            return this;
        }
    }
}
