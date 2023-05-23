package org.lflang.ast;

import static org.lflang.AttributeUtils.isEnclave;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.generator.DelayBodyGenerator;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.Connection;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Time;
import org.lflang.lf.VarRef;

/**
 * This class implements AST transformations for enclaved connections. There are three types of
 * enclaved connections: 1) Zero-delay connections 2) Delayed connections 3) Physical connections
 */
public class EnclavedConnectionTransformation implements AstTransformation {

  /** The Lingua Franca factory for creating new AST nodes. */
  public static final LfFactory factory = ASTUtils.factory;

  /** A code generator used to insert reaction bodies for the generated delay reactors. */
  private final DelayBodyGenerator generator;

  /**
   * A target type instance that is used during the transformation to manage target specific types
   */
  private final TargetTypes targetTypes;

  /** The Eclipse eCore view of the main LF file. */
  private final Resource mainResource;

  /** Collection of generated delay classes. */
  private final LinkedHashSet<Reactor> delayClasses = new LinkedHashSet<>();

  private final LinkedHashSet<Reactor> wrapperClasses = new LinkedHashSet<>();
  private final LinkedHashSet<Reactor> connectionClasses = new LinkedHashSet<>();

  /**
   * @param generator
   * @param targetTypes
   * @param mainResource
   */
  public EnclavedConnectionTransformation(
      DelayBodyGenerator generator, TargetTypes targetTypes, Resource mainResource) {
    this.generator = generator;
    this.targetTypes = targetTypes;
    this.mainResource = mainResource;
  }

  /** Transform all after delay connections by inserting generated delay reactors. */
  @Override
  public void applyTransformation(List<Reactor> reactors) {
    insertGeneratedEnclavedConnections(reactors);
  }

  /**
   * Find connections in the given resource that have a delay associated with them, and reroute them
   * via a generated delay reactor.
   *
   * @param reactors A list of reactors to apply the transformation to.
   */
  private void insertGeneratedEnclavedConnections(List<Reactor> reactors) {
    // The resulting changes to the AST are performed _after_ iterating
    // in order to avoid concurrent modification problems.
    List<Connection> oldConnections = new ArrayList<>();
    Map<EObject, List<Connection>> newConnections = new LinkedHashMap<>();
    Map<EObject, List<Instantiation>> delayInstances = new LinkedHashMap<>();

    Map<ReactorDecl, Reactor> enclaveWrappers = new LinkedHashMap<>();

    // Iterate over all reactor instances, and find any enclaves
    for (Reactor container : reactors) {
      for (Instantiation inst : container.getInstantiations()) {
        if (isEnclave(inst)) {
          Reactor r = ASTUtils.toDefinition(inst.getReactorClass());
          if (!enclaveWrappers.containsKey(r)) {}
        }
      }
    }
  }

  // Create an enclave wrapper class for a particular enclave
  private Reactor createEnclaveWrapperClass(Instantiation enclave) {

    Reactor enclaveClass = ASTUtils.toDefinition(enclave.getReactorClass());
    Reactor wrapperClass = factory.createReactor();
    for (int i = 0; i < enclaveClass.getInputs().size(); i++) {
      Input in = factory.createInput();
      VarRef topInRef = factory.createVarRef();
      VarRef connInRef = factory.createVarRef();
      VarRef connOutRef = factory.createVarRef();
      VarRef encInRef = factory.createVarRef();
      Reactor connClass = createEnclaveConnectionClass(enclaveClass.getInputs().get(i));
      Instantiation connInst = createEnclaveConnectionInstance(connClass);
      Connection topConn = factory.createConnection();
      Connection connEnc = factory.createConnection();

      // Connect top port with input port of ConnectionReactor
      topInRef.setVariable(in);
      connInRef.setContainer(connInst);
      connInRef.setVariable(connClass.getInputs().get(0));
      topConn.getLeftPorts().add(topInRef);
      topConn.getRightPorts().add(connInRef);

      // Connect output port of ConnectionReactor with the enclaved reactor
      encInRef.setContainer(enclave);
      encInRef.setVariable(enclaveClass.getInputs().get(i));
      connOutRef.setContainer(connInst);
      connOutRef.setVariable(connClass.getOutputs().get(0));
      connEnc.getLeftPorts().add(connOutRef);
      connEnc.getRightPorts().add(encInRef);

      // Create parameters for the delay on the connection
      Parameter topDelay =
          createDelayParameter(enclaveClass.getInputs().get(0).getName() + "_delay");
      ParameterReference paramRef = factory.createParameterReference();
      paramRef.setParameter(topDelay);
      //            connInst.getParameters().add(paramRef);

      topDelay.setName("delay");
      //            topConn.getLeftPorts().add(inRef);
      //            topConn.getRightPorts().add(connInst.get);

    }
    return enclaveClass;
  }

  private Reactor createEnclaveConnectionClass(Input input) {
    Reactor enclaveConn = factory.createReactor();
    Parameter delayParam = createDelayParameter("delay");
    return enclaveConn;
  }

  private Instantiation createEnclaveConnectionInstance(Reactor conn) {
    Instantiation inst = factory.createInstantiation();
    return inst;
  }

  private Parameter createDelayParameter(String name) {
    Parameter delay = factory.createParameter();
    delay.setType(factory.createType());
    delay.getType().setId("time");
    delay.getType().setTime(true);
    delay.setName(name);
    Time defaultTime = factory.createTime();
    defaultTime.setUnit(null);
    defaultTime.setInterval(0);
    Initializer init = factory.createInitializer();
    init.setParens(true);
    init.setBraces(false);
    init.getExprs().add(defaultTime);
    delay.setInit(init);
    return delay;
  }
}
