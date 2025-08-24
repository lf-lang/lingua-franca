/*************
 * Copyright (c) 2023, The Norwegian University of Science and Technology.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.generator.c;

import static org.lflang.AttributeUtils.isEnclave;
import static org.lflang.util.FileUtil.getResourceFromClassPath;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CEnclaveGraph.EnclaveConnection;
import org.lflang.graph.ConnectionGraph;
import org.lflang.lf.Assignment;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;
import org.lflang.lf.VarRef;
import org.lflang.util.Pair;

/**
 * This class implements the AST transformation enabling enclaved execution in the C target. This
 * transformation finds all connections involving an enclave and inserts a special connection
 * Reactor there. The connection Reactor is inspired by the after-delay reactors. They consist of an
 * action and two reactions. The first reaction, the `delay reaction`, schedules events received
 * onto the action. The other reaction, the `forward reaction` writes the scheduled event to its
 * output port.
 */
public class CEnclavedReactorTransformation implements AstTransformation {

  public static final LfFactory factory = ASTUtils.factory;
  private final Resource mainResource;
  protected CTypes types;

  // The following fields must match the parameter names in the EnclavedConnection library reactor.
  public static String sourceEnvParamName = "source_env";
  public static String destEnvParamName = "dest_env";
  public static String delayParamName = "delay";
  public static String isPhysicalParamName = "is_physical";
  public static String hasAfterDelayParamName = "has_after_delay";
  public static String enclaveConnectionLibraryPath = "/lib/c/EnclaveConnection.lf";

  public Instantiation PARENT = factory.createInstantiation();

  /**
   * @brief Set the environment pointers which are parameters to the enclave connection reactor.
   * @param conn The generated enclave connection reactor
   * @param source The upstream reactor
   * @param dest The downstream reactor
   */
  public void setEnvParams(ReactorInstance conn, ReactorInstance source, ReactorInstance dest) {
    ParameterInstance srcEnvParam = conn.getParameter(sourceEnvParamName);
    CodeExpr srcExpr = factory.createCodeExpr();
    srcExpr.setCode(factory.createCode());
    srcExpr.getCode().setBody(CUtil.getEnvironmentStructPtr(source));
    ASTUtils.overrideParameter(srcEnvParam, srcExpr);

    ParameterInstance dstEnvParam = conn.getParameter(destEnvParamName);
    CodeExpr dstExpr = factory.createCodeExpr();
    dstExpr.setCode(factory.createCode());
    dstExpr.getCode().setBody(CUtil.getEnvironmentStructPtr(dest));
    ASTUtils.overrideParameter(dstEnvParam, dstExpr);
  }

  /**
   * The AST transformation also collects information about the enclave connections. Note that this
   * is on the AST graph, containing Instantiations, not ReactorInstances. This graph is exposed and
   * used later to build the graph of ReactorInstances that are enclaves.
   */
  public final ConnectionGraph<Instantiation, EnclaveConnection> connGraph =
      new ConnectionGraph<>();

  /**
   * We also expose a map of Instantiations of EnclavedConnection reactors to their upstream and
   * downstream Instantiations
   */
  public final Map<Instantiation, Pair<Instantiation, Instantiation>> enclavedConnections =
      new HashMap<>();

  public CEnclavedReactorTransformation(Resource mainResource, CTypes types) {
    this.mainResource = mainResource;
    this.types = types;
  }

  /** We only need a single ConnectionReactor since it uses generics. */
  Reactor connectionReactor = null;

  /**
   * Part of the AstTransformation interface. Performs the transformation.
   *
   * @param reactors A list of Reactor`s to perform the AST transformation on.
   */
  public void applyTransformation(List<Reactor> reactors) {
    insertEnclavedConnections(reactors);
  }

  private void insertEnclavedConnections(List<Reactor> reactors) {
    List<Pair<Connection, List<Instantiation>>> toReroute = new ArrayList<>();

    // Iterate over the connections in the tree and find the ones to replace.
    for (Reactor container : reactors) {
      for (Connection connection : ASTUtils.allConnections(container)) {
        List<Instantiation> newInstantiations = new ArrayList<>();
        var lhsPorts = connection.getLeftPorts();
        var rhsPorts = connection.getRightPorts();
        var lhsIndex = 0;
        // The following will become true if any connection between enclaves is found.
        var isEnclaveConnection = false;
        for (var rhsIndex = 0; rhsIndex < rhsPorts.size(); rhsIndex++) {
          VarRef lhs = lhsPorts.get(lhsIndex++);
          VarRef rhs = rhsPorts.get(rhsIndex);

          if (lhsIndex >= connection.getLeftPorts().size()) {
            // Assume the LHS is iterated (checked by the validator, so don't need to check here).
            lhsIndex = 0;
          }

          if (isEnclavePort(lhs) || isEnclavePort(rhs)) {
            isEnclaveConnection = true;
            Type type = ((Port) lhs.getVariable()).getType();
            // Create an enclave connection instantiation and give it a unique name.
            Instantiation connInst =
                createEnclavedConnectionInstance(
                    type, EcoreUtil.copy(connection.getDelay()), connection.isPhysical());
            connInst.setName(ASTUtils.getUniqueIdentifier(container, "enclave_conn"));

            // Get delay info and whether it is physical
            TimeValue delay = TimeValue.NEVER;
            boolean hasAfterDelay = connection.getDelay() != null;
            boolean isPhysical = connection.isPhysical();

            if (hasAfterDelay) {
              delay = ASTUtils.getDelayAsTimeValue(connection.getDelay());
            }

            EnclaveConnection edge = new EnclaveConnection(delay, hasAfterDelay, isPhysical);

            // Store connection info for the code-generator to use later. If the enclave is
            // connected
            // to its parent. We store a placeholder instantiation.
            Instantiation src = lhs.getContainer();
            if (src == null) {
              src = PARENT;
            }
            Instantiation dst = rhs.getContainer();
            if (dst == null) {
              dst = PARENT;
            }

            // If this is a normal logical connection. Store this info for later code-generation.
            // If it is a physical connection, the runtime does not need to know about it.
            if (!isPhysical) {
              connGraph.addEdge(src, dst, edge);
            }
            // Store a mapping from the connection reactor to its upstream/downstream. To be used
            // in code-gen later.
            enclavedConnections.put(connInst, new Pair<>(src, dst));
            newInstantiations.add(connInst);
          } else {
            newInstantiations.add(null);
          }
        }
        if (isEnclaveConnection) {
          toReroute.add(new Pair<>(connection, newInstantiations));
        }
      }
    }
    // Reroute all the connections via the newly created enclave connection instantiations.
    rerouteViaInstance(toReroute);
  }

  /**
   * Reroute the given connections to go through their given list of instantiations. For each
   * instantiation in the list, create two connections to reroute specified connection to instead go
   * through the specified instantiation.
   *
   * <p>Note that this is somewhat similar to ASTUtils.rerouteViaInstance, but does not assume that
   * the replacement instantiation is a bank.
   *
   * @param conns A list of pairs of connections and lists of instantiations.
   */
  private static void rerouteViaInstance(List<Pair<Connection, List<Instantiation>>> conns) {
    for (var pair : conns) {
      Connection connection = pair.first();
      EObject parent = connection.eContainer();

      var leftPorts = connection.getLeftPorts();
      var rightPorts = connection.getRightPorts();
      var leftIndex = 0;
      var rightIndex = 0;
      for (var inst : pair.second()) {
        if (inst == null) {
          // This is not a connection between enclaves.
          // Replace with an ordinary connection bridging the original ports.
          Connection newConnection = factory.createConnection();
          newConnection.getLeftPorts().add(leftPorts.get(leftIndex));
          newConnection.getRightPorts().add(rightPorts.get(rightIndex));
          newConnection.setDelay(connection.getDelay());
          newConnection.setPhysical(connection.isPhysical());
          if (parent instanceof Reactor) {
            ((Reactor) parent).getConnections().add(newConnection);
          } else {
            // Parent must be a Mode.
            ((Mode) parent).getConnections().add(newConnection);
          }
        } else {
          // This is a connection between enclaves.
          // Create a connection reactor and two connections to reroute the original connection
          // to go through the new connection reactor.
          Connection upstream = factory.createConnection();
          Connection downstream = factory.createConnection();
          VarRef input = factory.createVarRef();
          VarRef output = factory.createVarRef();
          Reactor instClass = ASTUtils.toDefinition(inst.getReactorClass());
          // Establish references to the involved ports.
          input.setContainer(inst);
          input.setVariable(instClass.getInputs().get(0));
          output.setContainer(inst);
          output.setVariable(instClass.getOutputs().get(0));
          upstream.getLeftPorts().add(leftPorts.get(leftIndex));
          upstream.getRightPorts().add(input);
          downstream.getLeftPorts().add(output);
          downstream.getRightPorts().add(rightPorts.get(rightIndex));

          // Insert new connection reactor and connections into the parent.
          if (parent instanceof Reactor) {
            ((Reactor) parent).getInstantiations().add(inst);
            ((Reactor) parent).getConnections().add(upstream);
            ((Reactor) parent).getConnections().add(downstream);
          } else {
            // Parent must be a Mode.
            ((Mode) parent).getInstantiations().add(inst);
            ((Mode) parent).getConnections().add(upstream);
            ((Mode) parent).getConnections().add(downstream);
          }
        }
        leftIndex++;
        rightIndex++;
        if (leftIndex >= leftPorts.size()) {
          // Assume the connection is iterated.
          leftIndex = 0;
        }
      }
      // Remove old connection.
      if (parent instanceof Reactor) {
        ((Reactor) parent).getConnections().remove(connection);
      } else {
        // Parent must be a Mode.
        ((Mode) parent).getConnections().remove(connection);
      }
    }
  }

  /**
   * Creates a Reactor definition for an EnclaveConnection reactor. It fetches the reactor from the
   * JAR, and caches it for use multiple times.
   */
  private Reactor createEnclaveConnectionClass() {
    if (connectionReactor != null) {
      return connectionReactor;
    }
    Resource resource;
    try {
      resource =
          getResourceFromClassPath(
              this.mainResource.getResourceSet(), enclaveConnectionLibraryPath);
    } catch (Exception e) {
      throw new RuntimeException("Could not retrieve EnclaveConnection library reactor from JAR");
    }

    // Get the reactor.
    Iterable<EObject> nodes = IteratorExtensions.toIterable(resource.getAllContents());
    var connReactor = IterableExtensions.filter(nodes, Reactor.class).iterator().next();

    // Hook it into AST.
    EObject node =
        IteratorExtensions.findFirst(mainResource.getAllContents(), Model.class::isInstance);
    ((Model) node).getReactors().add(connReactor);
    connectionReactor = connReactor;
    return connectionReactor;
  }

  /**
   * Retrieve a parameter with the given name from a reactor definition.
   *
   * @param def The reactor definition.
   * @param name The name of the parameter.
   * @return The parameter.
   */
  private Parameter getParameter(Reactor def, String name) {
    for (Parameter p : def.getParameters()) {
      if (p.getName().equals(name)) {
        return p;
      }
    }
    throw new RuntimeException("Could not find paramter: " + name + " in reactor " + def.getName());
  }

  /**
   * Returns whether the given VarRef is to a port of an enclave.
   *
   * @param portRef The reference to the port.
   * @return Is the port of an enclave.
   */
  private boolean isEnclavePort(VarRef portRef) {
    Instantiation container = portRef.getContainer();
    // If the container is null, then we are dealing with an internal connection. We are referring
    // to a top-level port.
    // By convention, we say that this is port is not an enclave port. It is an internal connection.
    if (container == null) {
      return false;
    } else {
      return isEnclave(container);
    }
  }

  /**
   * Creates an instantiation of a Connection Reactor, initialize the parameters.
   *
   * @param type The data type of the connection.
   * @param delay The delay of the connection (null if no delay)
   * @return The newly created instantiation.
   */
  private Instantiation createEnclavedConnectionInstance(
      Type type, Expression delay, boolean isPhysical) {

    // Create Connection reactor def and inst.
    Reactor def = createEnclaveConnectionClass();
    Instantiation inst = factory.createInstantiation();
    inst.setReactorClass(def);
    inst.getTypeArgs().add(EcoreUtil.copy(type));

    Assignment hasAfterDelayAssignment = factory.createAssignment();
    hasAfterDelayAssignment.setLhs(getParameter(def, hasAfterDelayParamName));
    Initializer initHasAfter = factory.createInitializer();
    CodeExpr exprHasAfter = factory.createCodeExpr();
    exprHasAfter.setCode(factory.createCode());

    Assignment isPhysicalAssignment = factory.createAssignment();
    isPhysicalAssignment.setLhs(getParameter(def, isPhysicalParamName));
    Initializer isPhysicalInit = factory.createInitializer();
    CodeExpr isPhysicalExpr = factory.createCodeExpr();
    isPhysicalExpr.setCode(factory.createCode());

    if (delay != null) {
      Assignment delayAssignment = factory.createAssignment();
      delayAssignment.setLhs(getParameter(def, delayParamName));
      Initializer init = factory.createInitializer();
      init.setExpr(Objects.requireNonNull(delay));
      delayAssignment.setRhs(init);
      inst.getParameters().add(delayAssignment);
      exprHasAfter.getCode().setBody("true");
    } else {
      exprHasAfter.getCode().setBody("false");
    }

    if (isPhysical) {
      isPhysicalExpr.getCode().setBody("true");
    } else {
      isPhysicalExpr.getCode().setBody("false");
    }
    initHasAfter.setExpr(exprHasAfter);
    hasAfterDelayAssignment.setRhs(initHasAfter);
    inst.getParameters().add(hasAfterDelayAssignment);

    isPhysicalInit.setExpr(isPhysicalExpr);
    isPhysicalAssignment.setRhs(isPhysicalInit);
    inst.getParameters().add(isPhysicalAssignment);
    return inst;
  }
}
