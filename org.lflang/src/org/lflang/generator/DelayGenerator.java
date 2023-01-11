package org.lflang.generator;

import java.util.HashSet;
import java.util.Set;

import org.eclipse.core.internal.resources.Resource;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.Target;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Assignment;
import org.lflang.lf.Connection;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.VarRef;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;

abstract public class DelayGenerator {

    /**
     * Constant that specifies how to name generated delay reactors.
     */
    public static String GEN_DELAY_CLASS_NAME = "_lf_GenDelay";


    private Set<Reactor> delayClasses = new HashSet<>();

    private final Target target;

    protected boolean useGenerics;

    protected boolean variableWidths;

    /**
     * @param target
     * @param genericType
     * @param variableWidth If this is true then a delay for a wide connection is left unspecified and is thus variable.
     *                      Otherwise, the width of bank of delay reactors will be inferred from the width of the connection.
     *
     */
    protected DelayGenerator(Target target, boolean genericType, boolean variableWidth) {
        this.target = target;
        this.useGenerics = genericType;
        this.variableWidths = variableWidth;
    }


    /**
     * Return the generic type to be used in the class definition
     * of a generated delay reactor.
     */
    protected String getClassGeneric() {
        return "T";
    }

    /**
     * Return the generic type to be used in the port definition
     * of a generated delay reactor.
     */
    protected String getPortGeneric() {
        return "T";
    }

    /**
     * Store the given reactor in the collection of generated delay classes
     * and insert it in the AST under the top-level reactor's node.
     */
    protected void addDelayClass(Reactor generatedDelay, Resource resource) {
        // Record this class, so it can be reused.
        delayClasses.add(generatedDelay);
        // And hook it into the AST.
        EObject node = IteratorExtensions.findFirst(resource.getAllContents(), Model.class::isInstance);
        ((Model) node).getReactors().add(generatedDelay);
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
     * @param connection The connection to create a delay instantiation foe
     *
     */
    public Instantiation createDelayInstantiation(Connection connection) {
        Type type = ((Port) connection.getRightPorts().get(0).getVariable()).getType();
        String generic = generator.getTargetTypes().getTargetType(InferredType.fromAST(type)); // FIXME: what is this

        var delay = connection.getDelay();
        var delayClass = getDelayClass(type, connection.eResource());
        var delayInst = ASTUtils.factory.createInstantiation();

        delayInst.setReactorClass(delayClass);
        if (useGenerics) {
            TypeParm typeParm = ASTUtils.factory.createTypeParm();
            typeParm.setLiteral(generic);
            delayInst.getTypeParms().add(typeParm);
        }
        if (ASTUtils.hasMultipleConnections(connection)) {
            WidthSpec widthSpec = ASTUtils.factory.createWidthSpec();
            if (!variableWidths) {
                // Add all left ports of the connection to the WidthSpec of the generated delay instance.
                // This allows the code generator to later infer the width from the involved ports.
                // We only consider the left ports here, as they could be part of a broadcast. In this case, we want
                // to delay the ports first, and then broadcast the output of the delays.
                for (VarRef port : connection.getLeftPorts()) {
                    WidthTerm term = ASTUtils.factory.createWidthTerm();
                    term.setPort(EcoreUtil.copy(port));
                    widthSpec.getTerms().add(term);
                }
            } else {
                widthSpec.setOfVariableLength(true);
            }
            delayInst.setWidthSpec(widthSpec);
        }
        Assignment assignment = ASTUtils.factory.createAssignment();
        assignment.setLhs(delayClass.getParameters().get(0));
        assignment.getRhs().add(delay);
        delayInst.getParameters().add(assignment);
        delayInst.setName("delay");  // This has to be overridden.
        return delayInst;
    }

    /**
     * Indicates whether delay banks generated from after delays should have a variable length width.
     *
     * If this is true, any delay reactors that are inserted for after delays on multiport connections
     * will have an unspecified variable length width. The code generator is then responsible for inferring the
     * correct width of the delay bank, which is only possible if the precise connection width is known at compile time.
     *
     * If this is false, the width specification of the generated bank will list all the ports listed on the right
     * side of the connection. This gives the code generator the information needed to infer the correct width at
     * runtime.
     */
    public boolean generateAfterDelaysWithVariableWidth() { return true; }

    /**
     * Return a synthesized AST node that represents the definition of a delay
     * reactor. Depending on whether the target supports generics, either this
     * method will synthesize a generic definition and keep returning it upon
     * subsequent calls, or otherwise, it will synthesize a new definition for
     * each new type it hasn't yet created a compatible delay reactor for.
     * @param type The type the delay class must be compatible with.
     * @param generic Whether to return a generic class.
     */
    protected Reactor getDelayClass(Type type, Resource resouce) {
        String className = getClassName(type);
        Reactor classDef = IterableExtensions.findFirst(delayClasses, it -> it.getName().equals(className));
        if (classDef != null) {
            return classDef;
        }
        return createReactor(type, className, resource);
    }

    protected Reactor createReactor(Type type, String className, Resource resource) {
        var delayClass = ASTUtils.factory.createReactor();
        var delayParameter = ASTUtils.factory.createParameter();
        var action = createAction(type, delayParameter);
        var input = createInput(type);
        var output = createOutput(type);

        // Configure the reactor.
        delayClass.setName(className);
        delayClass.getActions().add(action);
        delayClass.getInputs().add(input);
        delayClass.getOutputs().add(output);

        // Add the reactions to the newly created reactor class.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        delayClass.getReactions().add(createForwardReaction(action, output));
        delayClass.getReactions().add(createDelayReaction(input, action));

        // Add a type parameter if the target supports it.
        if (useGenerics) {
            TypeParm parm = ASTUtils.factory.createTypeParm();
            parm.setLiteral(getClassGeneric());
            delayClass.getTypeParms().add(parm);
        }

        delayClass.getParameters().add(delayParameter);
        addDelayClass(delayClass, resource);
        return delayClass;
    }

    protected Action createAction(Type type, Parameter delayParameter) {
        var action = ASTUtils.factory.createAction();
        var paramRef = ASTUtils.factory.createParameterReference();

        // Name the newly created action; set its delay and type.
        action.setName("act");

        delayParameter.setName("delay");
        delayParameter.setType(ASTUtils.factory.createType());
        delayParameter.getType().setId("time");
        delayParameter.getType().setTime(true);
        Time defaultTime = ASTUtils.factory.createTime();
        defaultTime.setUnit(null);
        defaultTime.setInterval(0);
        delayParameter.getInit().add(defaultTime);

        paramRef.setParameter(delayParameter);
        action.setMinDelay(paramRef);
        action.setOrigin(ActionOrigin.LOGICAL);
        action.setType(createType(type));

        return action;
    }

    protected Input createInput(Type type) {
        Input input = ASTUtils.factory.createInput();
        input.setName("inp");
        input.setType(EcoreUtil.copy(createType(type)));
        return input;
    }

    protected Output createOutput(Type type) {
        Output output = ASTUtils.factory.createOutput();
        output.setName("out");
        output.setType(EcoreUtil.copy(createType(type)));
        return output;
    }

    protected Reaction createDelayReaction(Input input, Action action) {
        Reaction reaction = ASTUtils.factory.createReaction();
        VarRef inRef = ASTUtils.factory.createVarRef();
        VarRef effectRef = ASTUtils.factory.createVarRef();

        inRef.setVariable(input);
        effectRef.setVariable(action);

        reaction.getTriggers().add(inRef);
        reaction.getEffects().add(effectRef);
        reaction.setCode(ASTUtils.factory.createCode());
        reaction.getCode().setBody(generateDelayBody(action, inRef));
        return reaction;
    }

    protected Reaction createForwardReaction(Action action, Output output) {
        var reaction = ASTUtils.factory.createReaction();
        var triggerRef = ASTUtils.factory.createVarRef();
        var outRef = ASTUtils.factory.createVarRef();
        triggerRef.setVariable(action);
        outRef.setVariable(output);
        // Configure the first reaction, which produces the output.
        reaction.getTriggers().add(triggerRef);
        reaction.getEffects().add(outRef);
        reaction.setCode(ASTUtils.factory.createCode());
        reaction.getCode().setBody(generateForwardBody(action, outRef));
        return reaction;
    }


    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param action the action that triggers the reaction
     * @param port the port to write to
     */
    public abstract String generateForwardBody(Action action, VarRef port);


    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action the action to schedule
     * @param port the port to read from
     */
    public abstract String generateDelayBody(Action action, VarRef port);


    protected Type createType(Type concrete) {
        Type type;
        if (useGenerics) {
            type = ASTUtils.factory.createType();
            type.setId("T");
        } else {
            type = EcoreUtil.copy(concrete);
        }
        return type;
    }

    protected String getClassName(Type type) {
        if (useGenerics) {
            return GEN_DELAY_CLASS_NAME;
        } else {
            String id = Integer.toHexString(InferredType.fromAST(type).toText().hashCode());
            return String.format("%s_%s", GEN_DELAY_CLASS_NAME, id);
        }
    }



}
