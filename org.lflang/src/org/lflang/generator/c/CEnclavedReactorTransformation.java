package org.lflang.generator.c;

import static org.lflang.AttributeUtils.isEnclave;
import static org.lflang.AttributeUtils.setEnclaveAttribute;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;

import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DelayBodyGenerator;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Code;
import org.lflang.lf.Connection;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.VarRef;
import org.lflang.util.IteratorUtil;

import com.google.common.collect.Iterables;

public class CEnclavedReactorTransformation implements AstTransformation {

    public static final LfFactory factory = ASTUtils.factory;

    private final Resource mainResource;
    public CEnclavedReactorTransformation(Resource mainResource) {
        this.mainResource = mainResource;
    }

    Reactor connectionReactor = null;
    Map<String, Reactor> connectionReactorDefs= new LinkedHashMap<>();

    // A hashmap mapping Reactor definitions which has enclaved instances to their wrapper definition
    // A hashmap mapping enclaved reactor instances to their wrapper instances
    public void applyTransformation(List<Reactor> reactors) {
        // This function performs the whole AST transformation consisting in
        // 1. Get all Enclave Reactors
        List <Instantiation> enclaveInsts = getEnclaveInsts(reactors);
        List <Reactor> enclaveDefs = enclaveInsts.stream().map(r -> ASTUtils.toDefinition(r.getReactorClass())).distinct().toList();

        // 2. create ReactorWrappers for all of them.
        Map<Reactor, Reactor> defMap = createEnclaveWrappers(enclaveDefs);

        // 2. Replace enclave Reactor instances with wrapper instances.
        Map<Instantiation, Instantiation> instMap =
            replaceEnclavesWithWrappers(enclaveInsts, defMap);

        // Find all enclaved connections and partition them out. E.g.
        // enclave1.out, normal1.out ->  enclave2.in, normal2.in
        // is split into two separate connections.
        isolateEnclavedConnections(reactors);

        // 3.
        connectWrappers(reactors, instMap);

        // 3. Create ConnectionReactors for connections between enclave and parent
        insertConnectionReactorsForEnclaveOutputs(reactors);

        setEnclaveWrapperParams(reactors);

        setFreeConnectionReactorParams(reactors);

    }
    // Get the reactor definitions of all the enclaves
    private List<Instantiation> getEnclaveInsts(List<Reactor> reactors) {
        List<Instantiation> enclaves = new ArrayList<>();
        for (Reactor container : reactors) {
            for (Instantiation inst : container.getInstantiations()) {
                if (isEnclave(inst)) {
                    enclaves.add(inst);
                }
            }
        }
        return enclaves;
    }

    // Side-effect: Fills the enclaveToWrapperMap with the newly created EnclaveWrappers
    private Map<Reactor, Reactor> createEnclaveWrappers(List<Reactor> enclaves) {
        Map<Reactor, Reactor> map= new LinkedHashMap<>();
        for (Reactor enclave: enclaves) {
            Reactor wrapper = createEnclaveWrapperClass(enclave);

            // Hook it into AST
            EObject node =
                IteratorExtensions.findFirst(mainResource.getAllContents(), Model.class::isInstance);
            ((Model) node).getReactors().add(wrapper);

            map.put(enclave, wrapper);
        }
        return map;
    }

    private Reactor createEnclaveWrapperClass(Reactor enclaveDef) {
        // TODO: Add the same parameters to wrapper
        // TODO: Copy wrapper parameters to the inst
        Reactor wrapper = factory.createReactor();
        wrapper.setName(wrapperClassName(enclaveDef.getName()));
        Instantiation wrappedEnclaveInst = ASTUtils.createInstantiation(enclaveDef);
        wrappedEnclaveInst.setName(wrappedInstanceName(enclaveDef.getName()));
        wrapper.getInstantiations().add(wrappedEnclaveInst);
        for (Input input: enclaveDef.getInputs()) {
            Input in = factory.createInput();
            Type type = input.getType();
            in.setName(input.getName());
            in.setType(EcoreUtil.copy(input.getType()));

            // Create Connection reactor def and inst
            Reactor connReactorDef = createEnclaveConnectionClass();
            Instantiation connReactorInst = factory.createInstantiation();
            connReactorInst.setReactorClass(connReactorDef);
            connReactorInst.setName(connReactorDef.getName() + input.getName());
            connReactorInst.getTypeArgs().add(EcoreUtil.copy(type));

            // Create the two actual connections between top-level, connection-reactor and wrapped enclave
            Connection conn1 = factory.createConnection();
            Connection conn2 = factory.createConnection();

            // Create var-refs fpr ports
            VarRef topInRef = factory.createVarRef();
            VarRef encInRef = factory.createVarRef();
            VarRef connInRef = factory.createVarRef();
            VarRef connOutRef = factory.createVarRef();

            // Tie the var-refs to their ports
            topInRef.setVariable(in);

            encInRef.setContainer(wrappedEnclaveInst);
            encInRef.setVariable(input);

            connInRef.setContainer(connReactorInst);
            connInRef.setVariable(connReactorDef.getInputs().get(0));

            connOutRef.setContainer(connReactorInst);
            connOutRef.setVariable(connReactorDef.getOutputs().get(0));

            // Connect var-refs and connections
            conn1.getLeftPorts().add(topInRef);
            conn1.getRightPorts().add(connInRef);

            conn2.getLeftPorts().add(connOutRef);
            conn2.getRightPorts().add(encInRef);

            // Add all objects to the wrapper class
            wrapper.getInputs().add(in);
            wrapper.getConnections().add(conn1);
            wrapper.getConnections().add(conn2);
            wrapper.getInstantiations().add(connReactorInst);
        }

        for (Output output: enclaveDef.getOutputs()) {
            Output out = factory.createOutput();
            Type type = output.getType();
            out.setName(output.getName());
            out.setType(EcoreUtil.copy(type));

            Reactor connReactorDef = createEnclaveConnectionClass();
            Instantiation connReactorInst = factory.createInstantiation();
            connReactorInst.setReactorClass(connReactorDef);
            connReactorInst.setName(connReactorDef.getName() + output.getName()); // FIXME: Separate the name into separate func?
            connReactorInst.getTypeArgs().add(EcoreUtil.copy(type));

            // Create the two actual connections between top-level, connection-reactor and wrapped enclave
            Connection conn1 = factory.createConnection();
            Connection conn2 = factory.createConnection();

            // Create var-refs fpr ports
            VarRef topOutRef = factory.createVarRef();
            VarRef encOutRef = factory.createVarRef();
            VarRef connInRef = factory.createVarRef();
            VarRef connOutRef = factory.createVarRef();

            // Tie the var-refs to their ports
            topOutRef.setVariable(out);

            encOutRef.setContainer(wrappedEnclaveInst);
            encOutRef.setVariable(output);

            connInRef.setContainer(connReactorInst);
            connInRef.setVariable(connReactorDef.getInputs().get(0));

            connOutRef.setContainer(connReactorInst);
            connOutRef.setVariable(connReactorDef.getOutputs().get(0));

            // Connect var-refs and connections
            conn1.getLeftPorts().add(encOutRef);
            conn1.getRightPorts().add(connInRef);

            conn2.getLeftPorts().add(connOutRef);
            conn2.getRightPorts().add(topOutRef);

            // Add all objects to the wrapper class
            wrapper.getOutputs().add(out);
            wrapper.getConnections().add(conn1);
            wrapper.getConnections().add(conn2);
            wrapper.getInstantiations().add(connReactorInst);
        }

        return wrapper;
    }
    private String wrapperClassName(String originalName) {
        return "_" + originalName + "Wrapper";
    }

    private String wrappedInstanceName(String originalName) {
        return "_" + originalName + "_wrapped";
    }

    private void connectWrapperThroughConnectionReactor(VarRef lhs, VarRef rhs, Instantiation connReactor) {

    }

    // Side effect: We overwrite the Reactor-list passed into the AST. We remove and add Reactors as
    // children into the hierarchy. This function should also remove all old connections between reactors
    // and original enclaves with new instances.
    private Map<Instantiation, Instantiation> replaceEnclavesWithWrappers(List<Instantiation> enclaveInsts, Map<Reactor, Reactor> defMap) {
        Map<Instantiation, Instantiation> instMap = new LinkedHashMap<>();

        for (Instantiation inst: enclaveInsts) {
            EObject parent = inst.eContainer();
            Reactor wrapperDef = defMap.get(ASTUtils.toDefinition(inst.getReactorClass()));
            Instantiation wrapperInst = ASTUtils.createInstantiation(wrapperDef);
            wrapperInst.setName("_wrapper_"+inst.getName());

            setEnclaveAttribute(wrapperInst);
            // TODO: Copy parameters from inst to wrapperInst

            if (parent instanceof Reactor) {
                ((Reactor) parent).getInstantiations().remove(inst);
                ((Reactor) parent).getInstantiations().add(wrapperInst);
            } else if (parent instanceof Mode) {
                ((Mode) parent).getInstantiations().remove(inst);
                ((Mode) parent).getInstantiations().add(wrapperInst);
            }

            instMap.put(inst, wrapperInst);
        }
        return instMap;
    }

    private void connectWrappers(List<Reactor> reactors, Map<Instantiation, Instantiation> instMap) {
        for (Reactor container : reactors) {
            for (Connection connection : ASTUtils.allConnections(container)) {
                replaceConnection(connection, instMap);
            }
        }
    }

    // TODO: Handle all connection patterns.
    private void replaceConnection(Connection conn, Map<Instantiation, Instantiation> instMap) {
       if (isEnclave2EnclaveConn(conn)) {
           replaceEnclave2EnclaveConn(conn, instMap);
       }
    }
    private boolean isEnclavePort(VarRef portRef) {
        return isEnclave(portRef.getContainer());
    }

    private boolean isEnclave2EnclaveConn(Connection conn) {
        VarRef lhs = conn.getLeftPorts().get(0);
        VarRef rhs = conn.getRightPorts().get(0);
        return isEnclavePort(lhs) && isEnclavePort(rhs);
    }

    private void replaceEnclave2EnclaveConn(Connection oldConn, Map<Instantiation, Instantiation> instMap) {
        Connection newConn  = factory.createConnection();
        VarRef wrapperSrcOutput = factory.createVarRef();
        VarRef wrapperDestInput = factory.createVarRef();

        VarRef oldOutput = oldConn.getLeftPorts().get(0);
        VarRef oldInput = oldConn.getRightPorts().get(0);
        Instantiation src = oldOutput.getContainer();
        Instantiation dest = oldInput.getContainer();
        Instantiation wrapperSrc = instMap.get(src);
        Instantiation wrapperDest = instMap.get(dest);

        wrapperSrcOutput.setContainer(wrapperSrc);
        wrapperSrcOutput.setVariable(getInstanceOutputPortByName(wrapperSrc, oldOutput.getVariable().getName()));
        wrapperDestInput.setContainer(wrapperDest);
        wrapperDestInput.setVariable(getInstanceInputPortByName(wrapperDest, oldInput.getVariable().getName()));

        newConn.getLeftPorts().add(wrapperSrcOutput);
        newConn.getRightPorts().add(wrapperDestInput);

        replaceConnInAST(oldConn, newConn);
    }

    private void replaceConnInAST(Connection oldConn, Connection newConn) {
        var container = oldConn.eContainer();
        if (container instanceof Reactor) {
            ((Reactor) container).getConnections().remove(oldConn);
            ((Reactor) container).getConnections().add(newConn);
        } else if (container instanceof Mode) {
            ((Mode) container).getConnections().remove(oldConn);
            ((Mode) container).getConnections().add(newConn);
        }
    }

    private Input getInstanceInputPortByName(Instantiation inst, String name) {
        for (Input in: ASTUtils.toDefinition(inst.getReactorClass()).getInputs()) {
            if (in.getName() == name) {
                return in;
            }
        }
        assert(false);
        return factory.createInput();
    }

    private Output getInstanceOutputPortByName(Instantiation inst, String name) {
        for (Output out: ASTUtils.toDefinition(inst.getReactorClass()).getOutputs()) {
            if (out.getName() == name) {
                return out;
            }
        }
        assert(false);
        return factory.createOutput();
    }

    private void isolateEnclavedConnections(List<Reactor> reactors) {

    }

    // In the case where the output port of an enclave is not connected to the top-level port of another enclave.
    // E.g. a contained enclave is connected to the parents output. Or it is connected to another childs input.
    // Or it is directly connected to a reaction in the parent (this requires some more thought)
    // In this case we must generate a ConnectionReactor and put it into the receiving environment. Which is the parent
    // in this case.

    private void insertConnectionReactorsForEnclaveOutputs(List<Reactor> reactors) {

    }

    // This sets the in_delay and is_physical parameters for all the enclaveWrappers.
    // It is based on the class and any delay on the connections to its ports.
    // The connections should be replaced with ordinary connections
    private void setEnclaveWrapperParams(List<Reactor> reactors) {

    }

    // This function finds all the free ConnectionReactors and sets their parameters.
    private void setFreeConnectionReactorParams(List<Reactor> reactors) {

    }


    // FIXME: Replace with library reactor. But couldnt figure out how to load it
    private Reactor createEnclaveConnectionClass() {
        if (connectionReactor != null) {
            return connectionReactor;
        }
        Type type = factory.createType();
        type.setId("T");

        TypeParm typeParam = factory.createTypeParm();
        typeParam.setLiteral("T");

        Preamble preamble = factory.createPreamble();
        preamble.setCode(factory.createCode());
        preamble.getCode().setBody(String.join("\n",
            "#include \"reactor_common.h\"",
            "#include <string.h>"
        ));

        String className = "EnclavecConnectionReactor";
        Reactor connReactor = factory.createReactor();
        connReactor.getTypeParms().add(typeParam);
        connReactor.getPreambles().add(preamble);
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
        Initializer init = factory.createInitializer();
        init.setParens(true);
        init.setBraces(false);
        init.getExprs().add(defaultTime);
        delayParameter.setInit(init);

        // Name the newly created action; set its delay and type.
        action.setName("act");
        var paramRef = factory.createParameterReference();
        paramRef.setParameter(delayParameter);
        action.setMinDelay(paramRef);
        action.setOrigin(ActionOrigin.LOGICAL);
        action.setType(EcoreUtil.copy(type));


        input.setName("in");
        input.setType(EcoreUtil.copy(type));

        output.setName("out");
        output.setType(EcoreUtil.copy(type));

        // Establish references to the involved ports.
        inRef.setVariable(input);
        outRef.setVariable(output);

        // Establish references to the action.
        triggerRef.setVariable(action);
        effectRef.setVariable(action);

        // Add the action to the reactor.
        connReactor.setName(className);
        connReactor.getActions().add(action);

        // Configure the second reaction, which reads the input.
        r1.getTriggers().add(inRef);
        r1.getEffects().add(effectRef);
        r1.setCode(factory.createCode());
        r1.getCode().setBody(enclavedConnectionDelayBody());

        // Configure the first reaction, which produces the output.
        r2.getTriggers().add(triggerRef);
        r2.getEffects().add(outRef);
        r2.setCode(factory.createCode());
        r2.getCode().setBody(enclavedConnectionForwardBody());

        // Add the reactions to the newly created reactor class.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        connReactor.getReactions().add(r2);
        connReactor.getReactions().add(r1);

        connReactor.getInputs().add(input);
        connReactor.getOutputs().add(output);
        connReactor.getParameters().add(delayParameter);

        // Hook it into AST
        EObject node =
            IteratorExtensions.findFirst(mainResource.getAllContents(), Model.class::isInstance);
        ((Model) node).getReactors().add(connReactor);

        connectionReactor = connReactor;

        return connReactor;
    }

    private String enclavedConnectionDelayBody() {
        CodeBuilder code = new CodeBuilder();
        code.pr("environment_t* src_env = in->_base.source_reactor->environment;");
        code.pr("environment_t* dest_env = self->base.environment;");
        code.pr("// Calculate the tag at which to schedule the event at the target");
        code.pr("tag_t target_tag = lf_delay_tag(src_env->current_tag, self->delay);");
        code.pr("int length = 1;");
        code.pr("if (in->token) length = in->length;");
        code.pr("token_template_t* template = (token_template_t*)act;");
        code.pr("lf_critical_section_enter(dest_env);");
        code.pr("lf_token_t* token = _lf_initialize_token(template, length);");
        code.pr("memcpy(token->value, &(in->value), template->type.element_size * length);");
        code.pr("// Schedule event to the destination environment.");
        code.pr("trigger_handle_t result = _lf_schedule_at_tag(dest_env, act->_base.trigger, target_tag, token);");
        code.pr("// Notify the main thread in case it is waiting for physical time to elapse");
        code.pr("lf_notify_of_event(dest_env);");
        code.pr("lf_critical_section_exit(dest_env);");
        return code.toString();
    }

    private String enclavedConnectionForwardBody() {
       CodeBuilder code = new CodeBuilder();
       code.pr("lf_set(out, act->value);");
       return code.toString();
    }
}
