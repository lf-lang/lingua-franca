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

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.MessageReporter;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CEnclaveInstance.EnclaveConnection;
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
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.VarRef;
import org.lflang.util.FileUtil;
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
  private final MessageReporter messageReporter;
  protected CTypes types;

  // The following fields must match the parameter names in the EnclavedConnection library reactor.
  public static String sourceEnvParamName = "source_env";
  public static String destEnvParamName = "dest_env";
  public static String delayParamName = "delay";
  public static String isPhysicalParamName = "is_physical";
  public static String hasAfterDelayParamName = "is_physical";
  public static String enclaveConnectionReactorName = "_EnclavedConnection";

  public static String enclaveConnectionLibraryPath = "/lib/c/EnclavedConnection.lf";

  public static Instantiation PARENT = factory.createInstantiation();

  // FIXME: docs
  public void setEnvParams(ReactorInstance conn, ReactorInstance source, ReactorInstance dest) {
    ParameterInstance srcEnvParam = conn.getParameter(sourceEnvParamName);
    CodeExpr srcExpr = factory.createCodeExpr();
    srcExpr.setCode(factory.createCode());
    srcExpr.getCode().setBody(CUtil.getEnvironmentStructPtr(source));
    srcEnvParam.override(srcExpr);

    ParameterInstance dstEnvParam = conn.getParameter(destEnvParamName);
    CodeExpr dstExpr = factory.createCodeExpr();
    dstExpr.setCode(factory.createCode());
    dstExpr.getCode().setBody(CUtil.getEnvironmentStructPtr(dest));
    dstEnvParam.override(dstExpr);
  }

  // The AST transformation also collects information about the enclave connections. Note that this
  // is on the AST graph, not the instance graph.
  public final ConnectionGraph<Instantiation, EnclaveConnection> connGraph =
      new ConnectionGraph<>();
  public final Map<Instantiation, Pair<Instantiation, Instantiation>> enclavedConnections =
      new HashMap<>();

  public CEnclavedReactorTransformation(
      Resource mainResource, MessageReporter messageReporter, CTypes types) {
    this.mainResource = mainResource;
    this.messageReporter = messageReporter;
    this.types = types;
  }

  // We only need a single ConnectionReactor since it uses generics.
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
    // The resulting changes to the AST are performed _after_ iterating
    // in order to avoid concurrent modification problems.
    List<Connection> oldConnections = new ArrayList<>();
    Map<EObject, List<Connection>> newConnections = new LinkedHashMap<>();
    Map<EObject, List<Instantiation>> enclaveConnInstances = new LinkedHashMap<>();

    // Iterate over the connections in the tree.
    for (Reactor container : reactors) {
      for (Connection connection : ASTUtils.allConnections(container)) {
        if (connection.isIterated()
            || connection.getLeftPorts().size() > 1
            || connection.getRightPorts().size() > 1) {
          continue;
        }
        VarRef lhs = connection.getLeftPorts().get(0);
        VarRef rhs = connection.getRightPorts().get(0);

        if (isEnclavePort(lhs) || isEnclavePort(rhs)) {
          EObject parent = connection.eContainer();
          Type type = ((Port) lhs.getVariable()).getType();
          Instantiation connInst =
              createEnclavedConnectionInstance(
                  type, EcoreUtil.copy(connection.getDelay()), connection.isPhysical());

          // FIXME: Docs
          TimeValue delay = TimeValue.NEVER;
          boolean hasAfterDelay = connection.getDelay() != null;
          boolean isPhysical = connection.isPhysical();

          if (hasAfterDelay) {
            Time delayExpr = (Time) connection.getDelay();
            delay = new TimeValue(delayExpr.getInterval(), TimeUnit.fromName(delayExpr.getUnit()));
          }
          // Store connection info for the code-generator to use later.
          Instantiation src = lhs.getContainer();
          if (src == null) {
            src = PARENT;
          }
          Instantiation dst = rhs.getContainer();
          if (dst == null) {
            dst = PARENT;
          }
          EnclaveConnection edge = new EnclaveConnection(delay, hasAfterDelay, isPhysical);
          connGraph.addEdge(src, dst, edge);

          List<Connection> connections =
              ASTUtils.convertToEmptyListIfNull(newConnections.get(parent));
          connections.addAll(ASTUtils.rerouteViaInstance(connection, connInst));
          newConnections.put(parent, connections);

          // Stage the original connection for deletion from the tree.
          oldConnections.add(connection);
          // Stage the newly created delay reactor instance for insertion
          List<Instantiation> instances =
              ASTUtils.convertToEmptyListIfNull(enclaveConnInstances.get(parent));
          instances.add(connInst);
          enclaveConnInstances.put(parent, instances);

          enclavedConnections.put(connInst, new Pair<>(src, dst));
        }
      }
    }

    // Remove old connections; insert new ones.
    oldConnections.forEach(
        connection -> {
          var container = connection.eContainer();
          if (container instanceof Reactor) {
            ((Reactor) container).getConnections().remove(connection);
          } else if (container instanceof Mode) {
            ((Mode) container).getConnections().remove(connection);
          }
        });
    newConnections.forEach(
        (container, connections) -> {
          if (container instanceof Reactor) {
            ((Reactor) container).getConnections().addAll(connections);
          } else if (container instanceof Mode) {
            ((Mode) container).getConnections().addAll(connections);
          }
        });
    // Finally, insert the instances and, before doing so, assign them a unique name.
    enclaveConnInstances.forEach(
        (container, instantiations) ->
            instantiations.forEach(
                instantiation -> {
                  if (container instanceof Reactor) {
                    instantiation.setName(
                        ASTUtils.getUniqueIdentifier((Reactor) container, "enclave_conn"));
                    ((Reactor) container).getInstantiations().add(instantiation);
                  } else if (container instanceof Mode) {
                    instantiation.setName(
                        ASTUtils.getUniqueIdentifier(
                            (Reactor) container.eContainer(), "enclave_conn"));
                    ((Mode) container).getInstantiations().add(instantiation);
                  }
                }));
  }

  private Reactor createEnclaveConnectionClass() {
    if (connectionReactor != null) {
      return connectionReactor;
    }
    Resource resource;
    try {
      resource = getResourceFromClassPath(this.mainResource.getResourceSet(), enclaveConnectionLibraryPath);
    } catch (Exception e) {
      throw new RuntimeException();
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
    throw new RuntimeException();
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
   * Creates an instantiation of a Connection Reactor.
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

    // FIXME: Modularize this a little. SO much repetition and tedious work....
    // FIXME: The parameter names should not be magic variables. Put them somewhere.
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
      init.getExprs().add(Objects.requireNonNull(delay));
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
    initHasAfter.getExprs().add(exprHasAfter);
    hasAfterDelayAssignment.setRhs(initHasAfter);
    inst.getParameters().add(hasAfterDelayAssignment);

    isPhysicalInit.getExprs().add(isPhysicalExpr);
    isPhysicalAssignment.setRhs(isPhysicalInit);
    inst.getParameters().add(isPhysicalAssignment);
    return inst;
  }
}
