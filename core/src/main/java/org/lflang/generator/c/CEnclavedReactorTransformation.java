package org.lflang.generator.c;

import static org.lflang.AttributeUtils.isEnclave;
import static org.lflang.AttributeUtils.setEnclaveAttribute;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Assignment;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.VarRef;

/**
 * This class implements an AST transformation enabling enclaved execution in the C target. The
 * challenge is to enable communication of data between the enclaves, which excute in different
 * environments. This is achieved through special connection Reactors which are inspired by the
 * DelayedConnection reactors. They reside in the environment of the target enclave, but is executed
 * by the scheduler of the source enclave. It implements the communication by scheduling events on
 * the event queue of the target enclave.
 *
 * <p>In order to put the connection Reactors inside the target environment, we create a wrapper
 * reactor around each enclave. This wrapper will contain the enclave as well as connection reactors
 * connected to all its inputs.
 */
public class CEnclavedReactorTransformation implements AstTransformation {

  public static final LfFactory factory = ASTUtils.factory;
  private final Resource mainResource;

  public CEnclavedReactorTransformation(Resource mainResource) {
    this.mainResource = mainResource;
  }

  // We only need a single ConnectionReactor since it uses generics.
  Reactor connectionReactor = null;

  public void applyTransformation(List<Reactor> reactors) {
    // This function performs the whole AST transformation consisting in
    // 1. Get all Enclave Reactors
    List<Instantiation> enclaveInsts = getEnclaveInsts(reactors);

    if (enclaveInsts.size() == 0) {
      return;
    }

    // Get reactor definitions of the enclaves
    List<Reactor> enclaveDefs =
        enclaveInsts.stream()
            .map(r -> ASTUtils.toDefinition(r.getReactorClass()))
            .distinct()
            .toList();

    // 2. create wrapper reactor definitions for for all of the reactors which have enclaved
    // instances.
    Map<Reactor, Reactor> defMap = createEnclaveWrappers(enclaveDefs);

    // 2. Replace enclave Reactor instances with wrapper instances.
    Map<Instantiation, Instantiation> instMap = replaceEnclavesWithWrappers(enclaveInsts, defMap);

    // 3. Disconnect the old instances and connect the new wrapper instances.
    //  also resolve the delay parameters needed for the ConnectionReactors
    connectWrappers(reactors, instMap);
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
    Map<Reactor, Reactor> map = new LinkedHashMap<>();
    for (Reactor enclave : enclaves) {
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
    // TODO: Support enclaves with parameters by duplicating the parameters in the wrapper class
    Reactor wrapper = factory.createReactor();
    wrapper.setName(wrapperClassName(enclaveDef.getName()));
    Instantiation wrappedEnclaveInst = ASTUtils.createInstantiation(enclaveDef);
    wrappedEnclaveInst.setName(wrappedInstanceName(enclaveDef.getName()));
    wrapper.getInstantiations().add(wrappedEnclaveInst);
    for (Input input : enclaveDef.getInputs()) {
      Input in = factory.createInput();
      Type type = input.getType();
      in.setName(input.getName());
      in.setType(EcoreUtil.copy(input.getType()));
      Parameter delayParam = createDelayParameter(input.getName() + "_delay");
      wrapper.getParameters().add(delayParam);
      ParameterReference delayParamRef = factory.createParameterReference();
      delayParamRef.setParameter(delayParam);

      // Create Connection reactor def and inst
      Reactor connReactorDef = createEnclaveConnectionClass();
      Instantiation connReactorInst = factory.createInstantiation();
      connReactorInst.setReactorClass(connReactorDef);
      connReactorInst.setName(connReactorDef.getName() + input.getName());
      connReactorInst.getTypeArgs().add(EcoreUtil.copy(type));

      // Set the delay parameter of the ConnectionRactor
      Assignment delayAssignment = factory.createAssignment();
      delayAssignment.setLhs(connReactorDef.getParameters().get(0));
      Initializer init = factory.createInitializer();
      init.getExprs().add(Objects.requireNonNull(delayParamRef));
      delayAssignment.setRhs(init);
      connReactorInst.getParameters().add(delayAssignment);

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

    for (Output output : enclaveDef.getOutputs()) {
      Output out = factory.createOutput();
      Type type = output.getType();
      out.setName(output.getName());
      out.setType(EcoreUtil.copy(type));

      // Create the two actual connections between top-level, connection-reactor and wrapped enclave
      Connection conn1 = factory.createConnection();

      // Create var-refs fpr ports
      VarRef topOutRef = factory.createVarRef();
      VarRef encOutRef = factory.createVarRef();

      // Tie the var-refs to their ports
      topOutRef.setVariable(out);

      encOutRef.setContainer(wrappedEnclaveInst);
      encOutRef.setVariable(output);

      // Connect var-refs and connections
      conn1.getLeftPorts().add(encOutRef);
      conn1.getRightPorts().add(topOutRef);

      // Add all objects to the wrapper class
      wrapper.getOutputs().add(out);
      wrapper.getConnections().add(conn1);
    }

    return wrapper;
  }

  private String wrapperClassName(String originalName) {
    return "_" + originalName + "Wrapper";
  }

  private String wrappedInstanceName(String originalName) {
    return "_" + originalName + "_wrapped";
  }

  private void connectWrapperThroughConnectionReactor(
      VarRef lhs, VarRef rhs, Instantiation connReactor) {}

  private Map<Instantiation, Instantiation> replaceEnclavesWithWrappers(
      List<Instantiation> enclaveInsts, Map<Reactor, Reactor> defMap) {
    Map<Instantiation, Instantiation> instMap = new LinkedHashMap<>();

    for (Instantiation inst : enclaveInsts) {
      EObject parent = inst.eContainer();
      Reactor wrapperDef = defMap.get(ASTUtils.toDefinition(inst.getReactorClass()));
      Instantiation wrapperInst = ASTUtils.createInstantiation(wrapperDef);
      wrapperInst.setName("_wrapper_" + inst.getName());

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

  // TODO: Handle all connection patterns. We are currently missing:
  //  1. Enclave -> Parent Output port
  //  2. Enclave -> Parent reaction
  //  3. Parent input port -> Enclave
  //  4. Parent reaction -> Enclave
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

  private void replaceEnclave2EnclaveConn(
      Connection oldConn, Map<Instantiation, Instantiation> instMap) {
    Connection newConn = factory.createConnection();
    VarRef wrapperSrcOutput = factory.createVarRef();
    VarRef wrapperDestInput = factory.createVarRef();

    VarRef oldOutput = oldConn.getLeftPorts().get(0);
    VarRef oldInput = oldConn.getRightPorts().get(0);
    Instantiation src = oldOutput.getContainer();
    Instantiation dest = oldInput.getContainer();
    Instantiation wrapperSrc = instMap.get(src);
    Instantiation wrapperDest = instMap.get(dest);

    // Set the delay parameter of the enclaved connection which is inside the wrapperDest
    Expression connDelay = oldConn.getDelay();
    if (connDelay != null) {
      Assignment delayAssignment = factory.createAssignment();
      delayAssignment.setLhs(
          getParameter(wrapperDest, oldInput.getVariable().getName() + "_delay"));
      Initializer init = factory.createInitializer();
      init.getExprs().add(connDelay);
      delayAssignment.setRhs(init);
      wrapperDest.getParameters().add(delayAssignment);
    }

    wrapperSrcOutput.setContainer(wrapperSrc);
    wrapperSrcOutput.setVariable(
        getInstanceOutputPortByName(wrapperSrc, oldOutput.getVariable().getName()));
    wrapperDestInput.setContainer(wrapperDest);
    wrapperDestInput.setVariable(
        getInstanceInputPortByName(wrapperDest, oldInput.getVariable().getName()));

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
    for (Input in : ASTUtils.toDefinition(inst.getReactorClass()).getInputs()) {
      if (in.getName() == name) {
        return in;
      }
    }
    throw new RuntimeException();
  }

  private Output getInstanceOutputPortByName(Instantiation inst, String name) {
    for (Output out : ASTUtils.toDefinition(inst.getReactorClass()).getOutputs()) {
      if (out.getName().equals(name)) {
        return out;
      }
    }
    throw new RuntimeException();
  }

  /** Create the EnclavedConnectionReactor definition. */
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
    preamble
        .getCode()
        .setBody(
            String.join(
                "\n",
                "#ifdef __cplusplus",
                "extern \"C\"",
                "{",
                "#endif",
                "#include \"reactor_common.h\"",
                "#include <string.h>",
                "#ifdef __cplusplus",
                "}",
                "#endif"));

    String className = "EnclaveConnectionReactor";
    Reactor connReactor = factory.createReactor();
    connReactor.getTypeParms().add(typeParam);
    connReactor.getPreambles().add(preamble);
    Parameter delayParameter = createDelayParameter("delay");
    var paramRef = factory.createParameterReference();
    paramRef.setParameter(delayParameter);

    Action action = factory.createAction();
    VarRef triggerRef = factory.createVarRef();
    VarRef effectRef = factory.createVarRef();
    Input input = factory.createInput();
    Output output = factory.createOutput();
    VarRef inRef = factory.createVarRef();
    VarRef outRef = factory.createVarRef();

    Reaction r1 = factory.createReaction();
    Reaction r2 = factory.createReaction();

    // Name the newly created action; set its delay and type.
    action.setName("act");
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
    code.pr("token_template_t* tmplate = (token_template_t*)act;");
    code.pr("lf_critical_section_enter(dest_env);");
    code.pr("lf_token_t* token = _lf_initialize_token(tmplate, length);");
    code.pr("memcpy(token->value, &(in->value), tmplate->type.element_size * length);");
    code.pr("// Schedule event to the destination environment.");
    code.pr(
        "int result = _lf_schedule_at_tag(dest_env, act->_base.trigger, target_tag, token);");
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

  /** Utility for creating a delay parameters initialized to 0 */
  private Parameter createDelayParameter(String name) {
    Parameter delayParameter = factory.createParameter();
    delayParameter.setName(name);
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

    return delayParameter;
  }

  /** Utility for getting a parameter by name. Exception is thrown if it does not exist */
  private Parameter getParameter(Instantiation inst, String name) {
    Reactor reactorDef = ASTUtils.toDefinition(inst.getReactorClass());
    for (Parameter p : reactorDef.getParameters()) {
      if (p.getName().equals(name)) {
        return p;
      }
    }
    throw new RuntimeException();
  }
}
