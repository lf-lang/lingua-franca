package org.lflang.generator.c;

import static org.lflang.AttributeUtils.copyEnclaveAttribute;
import static org.lflang.AttributeUtils.isEnclave;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;

import org.lflang.ErrorReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
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
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

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
  private final ErrorReporter errorReporter;

  protected CTypes types;

  public enum ConnectionType {
    ENCLAVE_TO_ENCLAVE,
    ENCLAVE_TO_PARENT,
    PARENT_TO_ENCLAVE,
    OTHER,
  }


  public CEnclavedReactorTransformation(Resource mainResource, ErrorReporter errorReporter, CTypes types) {
    this.mainResource = mainResource;
    this.errorReporter = errorReporter;
    this.types = types;
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

    // 2. create wrapper reactor definitions for all the reactors which have enclaved
    // instances.
    Map<Reactor, Reactor> defMap = createEnclaveWrappers(enclaveInsts);

    // 2. Replace enclave Reactor instances with wrapper instances.
    Map<Instantiation, Instantiation> instMap = replaceEnclavesWithWrappers(enclaveInsts, defMap);

    Map<Connection, ConnectionType> enclavedConnections = extractEnclaveConnections(reactors);
    updateConnections(enclavedConnections, instMap);
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

  // Side effect: Fills the enclaveToWrapperMap with the newly created EnclaveWrappers
  private Map<Reactor, Reactor> createEnclaveWrappers(List<Instantiation> enclaves) {
    Map<Reactor, Reactor> map = new LinkedHashMap<>();
    for (Instantiation enclaveInst: enclaves) {
      Reactor enclaveDef = ASTUtils.toDefinition(enclaveInst.getReactorClass());
      if (!map.containsKey(enclaveDef)) {
        Reactor wrapper = createEnclaveWrapperClass(enclaveInst);

        // Hook it into AST
        EObject node =
            IteratorExtensions.findFirst(mainResource.getAllContents(), Model.class::isInstance);
        ((Model) node).getReactors().add(wrapper);

        map.put(enclaveDef, wrapper);
      }
    }
    return map;
  }

  private Reactor createEnclaveWrapperClass(Instantiation enclaveInst) {
    // TODO: Support enclaves with parameters by duplicating the parameters in the wrapper class
    Reactor enclaveDef = ASTUtils.toDefinition(enclaveInst.getReactorClass());
    Reactor wrapper = factory.createReactor();
    wrapper.setName(wrapperClassName(enclaveDef.getName()));

    // Copy parameters from the enclave to the wrapper
    for (Parameter p: enclaveDef.getParameters()) {
      var pCpy = EcoreUtil.copy(p);
      wrapper.getParameters().add(pCpy);
    }

    // Create the wrapped enclave
    Instantiation wrappedEnclaveInst = ASTUtils.createInstantiation(enclaveDef);
    wrappedEnclaveInst.setName(enclaveInst.getName());

    // Forward the parameters of the wrapper to the wrapped enclave
    // FIXME: Is there a better way to do this? Is very tedious to work with the AST like this...
    for (int i = 0; i<wrapper.getParameters().size(); i++) {
      Assignment paramAssignment = factory.createAssignment();
      ParameterReference parentParamRef = factory.createParameterReference();
      parentParamRef.setParameter(wrapper.getParameters().get(i));
      Initializer init = factory.createInitializer();
      init.getExprs().add(parentParamRef);
      paramAssignment.setLhs(enclaveDef.getParameters().get(i));
      paramAssignment.setRhs(init);
      wrappedEnclaveInst.getParameters().add(paramAssignment);
    }

    // Add wrapped enclave to the instantiations of its wrapper
    wrapper.getInstantiations().add(wrappedEnclaveInst);

    for (Input input : enclaveDef.getInputs()) {
      Input in = factory.createInput();
      Type type = input.getType();
      String name = input.getName();
      in.setName(name);
      in.setType(EcoreUtil.copy(type));
      Parameter delayParam = createDelayParameter(name + "_delay");
      wrapper.getParameters().add(delayParam);
      ParameterReference delayParamRef = factory.createParameterReference();
      delayParamRef.setParameter(delayParam);

      Instantiation connReactorInst = createEnclavedConnectionInstance(name, type, delayParamRef);
      Reactor connReactorDef = ASTUtils.toDefinition(connReactorInst.getReactorClass());

      List<Connection> conns = connectEnclavedConnectionReactor(connReactorInst, in, null, Arrays.asList(input), Arrays.asList(wrappedEnclaveInst));

      // Add all objects to the wrapper class
      wrapper.getInputs().add(in);
      wrapper.getConnections().addAll(conns);
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
  private Map<Instantiation, Instantiation> replaceEnclavesWithWrappers(
      List<Instantiation> enclaveInsts, Map<Reactor, Reactor> defMap) {
    Map<Instantiation, Instantiation> instMap = new LinkedHashMap<>();

    for (Instantiation inst : enclaveInsts) {
      EObject parent = inst.eContainer();
      Reactor wrapperDef = defMap.get(ASTUtils.toDefinition(inst.getReactorClass()));
      Instantiation wrapperInst = ASTUtils.createInstantiation(wrapperDef);
      wrapperInst.setName("_wrapper_" + inst.getName());

      copyEnclaveAttribute(inst, wrapperInst);

      // Copy the parameter assignments from the enclave to the wrapper
      for (Assignment assignment: inst.getParameters()) {
        Assignment wrapperAssignment = EcoreUtil.copy(assignment);
        Parameter wrapperParameter = getParameter(wrapperInst, assignment.getLhs().getName());
        wrapperAssignment.setLhs(wrapperParameter);
        wrapperInst.getParameters().add(wrapperAssignment);
      }

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

  /**
   * @brief Extract and separate enclaved connections. So that they dont appear
   * together with any other connections.
   * e.g.
   * `e1.out, r1.out -> e2.in, r2.in`
   * Turns into
   * ```
   * e1.out -> r2.in
   * r1.out -> r2.in
   * ```
   * and so on.
   * @param reactors
   */
  private Map<Connection, ConnectionType> extractEnclaveConnections(List<Reactor> reactors) {
    Map<Connection, ConnectionType> res = new HashMap();
    for (Reactor container : reactors) {
      List<Connection> connectionsToAdd = new ArrayList<>();
      List<Connection> connectionsToRemove = new ArrayList<>();
      for (Connection connection : ASTUtils.allConnections(container)) {
        if (connection.isIterated()) {
          VarRef lhs = connection.getLeftPorts().get(0); // FIXME: Always a single lhs?
          if (isEnclavePort(lhs)) {
            // Enclave broadcasted to others. Split everything up
            for (VarRef rhs : connection.getRightPorts()) {
              var newRhs = copyVarRef(rhs);
              var newLhs = copyVarRef(lhs);
              var newConn = extractConnection(connection, newLhs, newRhs);
              if (isEnclavePort(rhs)) {
                res.put(newConn, ConnectionType.ENCLAVE_TO_ENCLAVE);
              } else {
                res.put(newConn, ConnectionType.ENCLAVE_TO_PARENT);
              }
              connectionsToAdd.add(newConn);
            }
            connectionsToRemove.add(connection);
          } else {
            List<VarRef> rhsPortsToRemove = new ArrayList();
            for (VarRef rhs : connection.getRightPorts()) {
              var newRhs = copyVarRef(rhs);
              var newLhs = copyVarRef(lhs);
              if (isEnclavePort(rhs)) {
                var newConn = extractConnection(connection, newLhs, newRhs);
                res.put(newConn, ConnectionType.PARENT_TO_ENCLAVE);
                connectionsToAdd.add(newConn);
                rhsPortsToRemove.add(rhs);
              }
            }
            // Remove the rhs ports extracted above
            rhsPortsToRemove.stream().forEach(v -> connection.getRightPorts().remove(v));
          }
        } else {
          List<VarRef> rhsToRemove = new ArrayList<>();
          List<VarRef> lhsToRemove = new ArrayList<>();

          for (int i = 0; i<connection.getRightPorts().size(); i++) {
            VarRef rhs = connection.getRightPorts().get(i);
            VarRef lhs = connection.getLeftPorts().get(i);

            if (isEnclavePort(lhs) || isEnclavePort(rhs)) {
              if (connection.getLeftPorts().size() > 1) {
                var newConn = extractConnection(connection, copyVarRef(lhs), copyVarRef(rhs));
                // Add connections to list of connections in the container reactor
                connectionsToAdd.add(newConn);

                // Schedule the VarRefs for deletion
                rhsToRemove.add(rhs);
                lhsToRemove.add(lhs);

                // Add to the result map of enclaved connections
                res.put(newConn, getConnectionType(newConn));
              } else {
                // Since we only have a single set of ports. Just add this connection to the result map
                res.put(connection, getConnectionType(connection));
              }
            }
          }

          // Remove the VarRefs that we extracted out in separate connections
          rhsToRemove.stream().forEach(r -> connection.getRightPorts().remove(r));
          lhsToRemove.stream().forEach(l -> connection.getLeftPorts().remove(l));

          // If we extracted all ports from this connection, just remove it
          if (connection.getLeftPorts().size() == 0 || connection.getRightPorts().size() == 0) {
            connectionsToRemove.add(connection);
          }
        }
      }

      // Add and remove connections for current container
      connectionsToAdd.stream().forEach(c -> container.getConnections().add(c));
      connectionsToRemove.stream().forEach(c -> container.getConnections().remove(c));
    }
    return res;
  }

  private Connection extractConnection(Connection connection, VarRef lhs, VarRef rhs) {
    // Create a new connection for the enclaves
    Connection copy = EcoreUtil.copy(connection);
    Connection newConn = factory.createConnection();

    newConn.setDelay(copy.getDelay());
    newConn.setPhysical(copy.isPhysical());
    newConn.setSerializer(copy.getSerializer());
    newConn.getRightPorts().add(rhs);
    newConn.getLeftPorts().add(lhs);

    return newConn;
  }
  private void updateConnections(Map<Connection, ConnectionType> enclavedConnections, Map<Instantiation, Instantiation> instMap) {
    // Accessing elements
    for (Map.Entry<Connection, ConnectionType> entry : enclavedConnections.entrySet()) {
      Connection conn = entry.getKey();
      ConnectionType connType = entry.getValue();
      switch (connType) {
      case ENCLAVE_TO_ENCLAVE:
        updateEnclave2EnclaveConn(conn, instMap);
        break;
      case ENCLAVE_TO_PARENT:
        updateEnclave2ParentConn(conn, instMap);
        break;
      case PARENT_TO_ENCLAVE:
        updateParent2EnclaveConn(conn, instMap);
        break;
      default:
        errorReporter.reportError("Found OTHER connection. Not implemented yet");
        break;
      }

    }
  }

  private void updateParent2EnclaveConn(
      Connection conn, Map<Instantiation, Instantiation> instMap) {

    VarRef parentPortRef = conn.getLeftPorts().get(0);
    VarRef enclaveInput = conn.getRightPorts().get(0);

    Instantiation enclaveDestInst = enclaveInput.getContainer();
    Instantiation enclaveWrapperDestInst = instMap.get(enclaveDestInst);

    // Set the delay parameter of the enclaved connection which is inside the wrapperDest
    Expression connDelay = conn.getDelay();
    if (connDelay != null) {
      Assignment delayAssignment = factory.createAssignment();
      delayAssignment.setLhs(
          getParameter(enclaveWrapperDestInst, enclaveInput.getVariable().getName() + "_delay"));
      Initializer init = factory.createInitializer();
      init.getExprs().add(connDelay);
      delayAssignment.setRhs(init);
      enclaveWrapperDestInst.getParameters().add(delayAssignment);
    }

    VarRef enclaveWrapperDestPortRef = factory.createVarRef();
    enclaveWrapperDestPortRef.setContainer(enclaveWrapperDestInst);
    enclaveWrapperDestPortRef.setVariable(
        getInstanceInputPortByName(enclaveWrapperDestInst, enclaveInput.getVariable().getName()));

    Connection newConn = factory.createConnection();
    newConn.getLeftPorts().add(parentPortRef);
    newConn.getRightPorts().add(enclaveWrapperDestPortRef);

    replaceConnInAST(conn, newConn);
  }

  private void updateEnclave2ParentConn(
      Connection conn, Map<Instantiation, Instantiation> instMap) {

    VarRef enclaveOutput = conn.getLeftPorts().get(0);
    List<VarRef> parentInputs = conn.getRightPorts();
    Port enclaveOutputPort = (Port) enclaveOutput.getVariable();
    Type enclaveOutputType = enclaveOutputPort.getType();

    Instantiation src = enclaveOutput.getContainer();
    Instantiation wrapperSrc = instMap.get(src);

    // Create Connection reactor def and inst
    Instantiation connReactorInst = createEnclavedConnectionInstance(
        getEnclaveToParentConnectionName(enclaveOutputPort, (Port) parentInputs.get(0).getVariable()),
        enclaveOutputType,
        conn.getDelay()
        );
    Reactor connReactorDef = ASTUtils.toDefinition(connReactorInst.getReactorClass());

    Variable lhs = getInstanceOutputPortByName(wrapperSrc, enclaveOutputPort.getName());
    List<Variable> rhss = parentInputs.stream().map(v -> v.getVariable()).toList();
    List<Instantiation> rhsParents = parentInputs.stream().map(v -> v.getContainer()).toList();
    List<Connection> conns = connectEnclavedConnectionReactor(
        connReactorInst,
        lhs,
        wrapperSrc,
        rhss,
        rhsParents
        );

    // Add the newly created ConnectionReactor to the appropriate instantiation
    EObject parent = conn.eContainer();
    insertInAST(connReactorInst, parent);

    // Remove old connection and add the new ones
    removeFromAST(conn, parent);
    conns.stream().forEach(c -> insertInAST(c, parent));
  }
  private void updateEnclave2EnclaveConn(
      Connection oldConn, Map<Instantiation, Instantiation> instMap) {

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
    VarRef wrapperSrcOutput = factory.createVarRef();
    VarRef wrapperDestInput = factory.createVarRef();

    wrapperSrcOutput.setContainer(wrapperSrc);
    wrapperSrcOutput.setVariable(
        getInstanceOutputPortByName(wrapperSrc, oldOutput.getVariable().getName()));
    wrapperDestInput.setContainer(wrapperDest);
    wrapperDestInput.setVariable(
        getInstanceInputPortByName(wrapperDest, oldInput.getVariable().getName()));

    Connection newConn = factory.createConnection();
    newConn.getLeftPorts().add(wrapperSrcOutput);
    newConn.getRightPorts().add(wrapperDestInput);

    replaceConnInAST(oldConn, newConn);
  }

  private void replaceConnInAST(Connection oldConn, Connection newConn) {
    var container = oldConn.eContainer();
    insertInAST(newConn, container);
    removeFromAST(oldConn, container);
  }

  private void insertInAST(Connection conn, EObject container) {
    if (container instanceof Reactor) {
      ((Reactor) container).getConnections().add(conn);
    } else if (container instanceof Mode) {
      ((Mode) container).getConnections().add(conn);
    } else {
      errorReporter.reportError("Tried inserting a connection into the AST with container=null");
    }
  }
  private void insertInAST(Instantiation inst, EObject container) {
    if (container instanceof Reactor) {
      ((Reactor) container).getInstantiations().add(inst);
    } else if (container instanceof Mode) {
      ((Mode) container).getInstantiations().add(inst);
    } else {
      errorReporter.reportError("Tried inserting a connection into the AST with container=null");
    }
  }



  private void removeFromAST(Connection conn, EObject container) {
    if (container instanceof Reactor) {
      ((Reactor) container).getConnections().remove(conn);
    } else if (container instanceof Mode) {
      ((Mode) container).getConnections().remove(conn);
    } else {
      errorReporter.reportError("Tried removing a connection into the AST with container=null");
    }
  }

  private Input getInstanceInputPortByName(Instantiation inst, String name) {
    for (Input in : ASTUtils.toDefinition(inst.getReactorClass()).getInputs()) {
      if (in.getName().equals(name)) {
        return in;
      }
    }
    errorReporter.reportError("getInstanceInputPortByName could not find port");
    return factory.createInput();
  }

  private Output getInstanceOutputPortByName(Instantiation inst, String name) {
    for (Output out : ASTUtils.toDefinition(inst.getReactorClass()).getOutputs()) {
      if (out.getName().equals(name)) {
        return out;
      }
    }
    errorReporter.reportError("getInstanceOutputPortByName could not find port");
    return factory.createOutput();
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
                "#include \"rti_local.h\"",
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
    r1.getCode().setBody(CReactionGenerator.generateEnclavedConnectionDelayBody());

    // Configure the first reaction, which produces the output.
    r2.getTriggers().add(triggerRef);
    r2.getEffects().add(outRef);
    r2.setCode(factory.createCode());
    r2.getCode().setBody(CReactionGenerator.generateEnclavedConnectionForwardBody());

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

  private ConnectionType getConnectionType(Connection conn) {
    if (conn.getRightPorts().size()==1 && conn.getLeftPorts().size()==1) {
      return getConnectionType(conn.getLeftPorts().get(0),conn.getRightPorts().get(0));
    } else {
      return ConnectionType.OTHER;
    }
  }

  private ConnectionType getConnectionType(VarRef lhs, VarRef rhs) {
    ConnectionType connType = ConnectionType.OTHER;
      if (isEnclavePort(rhs) && isEnclavePort(lhs)) {
        connType = ConnectionType.ENCLAVE_TO_ENCLAVE;
      } else if (isEnclavePort(rhs) && !isEnclavePort(lhs)) {
        connType = ConnectionType.PARENT_TO_ENCLAVE;
      } else if (!isEnclavePort(rhs) && isEnclavePort(lhs)) {
        connType = ConnectionType.ENCLAVE_TO_PARENT;
      }
    return connType;
  }

  private boolean isEnclavePort(VarRef portRef) {
    Instantiation container = portRef.getContainer();
    // If the container is null, then we are dealing with an internal connection. We are referring to a top-level port.
    // By convention, we say that this is port is not an enclave port. It is an internal connection.
    if (container == null) {
      return false;
    } else {
      return isEnclave(container);
    }
  }

  /**
   * Create an unique name for a ConnectionReactor between an enclave and a reactor in its parent
   * @param enclaveOutput
   * @param parentInput
   * @return
   */
  private String getEnclaveToParentConnectionName(Port enclaveOutput, Port parentInput) {
    return "enclave_connection_reactor_" + enclaveOutput.getName() + "_" + enclaveOutput.getName() + "_" + parentInput.getName();
  }

  private String getEnclaveToEnclaveConnectionName(Port wrapperIn) {
    return "enclave_connection_reactor_" + wrapperIn.getName();
  }

  private Instantiation createEnclavedConnectionInstance(String name, Type type, Expression delay) {

    // Create Connection reactor def and inst
    Reactor def = createEnclaveConnectionClass();
    Instantiation inst = factory.createInstantiation();
    inst.setReactorClass(def);
    inst.setName(name);
    inst.getTypeArgs().add(EcoreUtil.copy(type));

    // Set the delay parameter of the ConnectionRactor
    if (delay != null) {
      Assignment delayAssignment = factory.createAssignment();
      delayAssignment.setLhs(def.getParameters().get(0)); // FIXME: Abstract away magic number
      Initializer init = factory.createInitializer();
      init.getExprs().add(Objects.requireNonNull(delay));
      delayAssignment.setRhs(init);
      inst.getParameters().add(delayAssignment);
    }

    return inst;
  }

  // FIXME: Docs
  private List<Connection> connectEnclavedConnectionReactor(Instantiation connReactor, Variable lhs, Instantiation lhsParent, List<Variable> rhss, List<Instantiation> rhsParents) {
    Reactor connReactorDef = ASTUtils.toDefinition(connReactor.getReactorClass());

    // Create the two actual connections between top-level, connection-reactor and wrapped enclave
    Connection conn1 = factory.createConnection();
    Connection conn2 = factory.createConnection();

    // Create var-refs fpr ports
    VarRef lhsRef = factory.createVarRef();
    VarRef connInRef = factory.createVarRef();
    VarRef connOutRef = factory.createVarRef();

    // Tie the var-refs to their ports
    lhsRef.setVariable(lhs);
    if (lhsParent != null) {
      lhsRef.setContainer(lhsParent);
    }


    connInRef.setContainer(connReactor);
    connInRef.setVariable(connReactorDef.getInputs().get(0));

    connOutRef.setContainer(connReactor);
    connOutRef.setVariable(connReactorDef.getOutputs().get(0));

    // Connect var-refs and connections
    conn1.getLeftPorts().add(lhsRef);
    conn1.getRightPorts().add(connInRef);

    conn2.getLeftPorts().add(connOutRef);
    for (int i = 0; i<rhss.size(); i++) {
      Variable rhs = rhss.get(i);
      Instantiation rhsParent = rhsParents.get(i);
      VarRef rhsRef = factory.createVarRef();
      rhsRef.setVariable(rhs);
      if (rhsParent != null) {
        rhsRef.setContainer(rhsParent);
      }
      conn2.getRightPorts().add(rhsRef);
    }
    return Arrays.asList(conn1, conn2);
  }

  private VarRef copyVarRef(VarRef varRef) {
    var v = factory.createVarRef();
    v.setContainer(varRef.getContainer());
    v.setVariable(varRef.getVariable());
    v.setAlias(varRef.getAlias());
    v.setInterleaved(varRef.isInterleaved());
    v.setTransition(varRef.getTransition());
    return v;
  }
}
