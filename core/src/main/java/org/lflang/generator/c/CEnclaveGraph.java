package org.lflang.generator.c;

import static org.lflang.AttributeUtils.isEnclave;

import java.util.LinkedList;
import java.util.Queue;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CEnclaveInstance.EnclaveConnection;
import org.lflang.graph.ConnectionGraph;
import org.lflang.lf.Instantiation;

public class CEnclaveGraph {
  public ConnectionGraph<CEnclaveInstance, EnclaveConnection> graph = new ConnectionGraph<>();
  private CEnclavedReactorTransformation ast;

  public CEnclaveGraph(CEnclavedReactorTransformation ast) {
    this.ast = ast;
  }
  // FIXME: Look for ZDC

  // FIXME: docs
  public void build(ReactorInstance main, ReactorEnclaveMap enclaveMap) {
    Queue<ReactorInstance> queue = new LinkedList<>();
    queue.add(main);
    while (!queue.isEmpty()) {
      ReactorInstance current = queue.poll();
      for (ReactorInstance child : current.children) {
        Instantiation inst = child.getDefinition();
        if (isEnclave(inst)) {
          CEnclaveInstance enc = enclaveMap.get(child);
          var downStreamMap = ast.connGraph.getDownstreamOf(inst);
          for (Instantiation downstream : downStreamMap.keySet()) {
            if (downstream.equals(ast.PARENT)) {
              // If downstream was `null` then we have a connection to the parent.
              CEnclaveInstance downEnclave = enclaveMap.get(current.enclave);
              graph.addEdges(enc, downEnclave, downStreamMap.get(downstream));
            } else {
              ReactorInstance down =
                  current.children.stream()
                      .filter(i -> i.getDefinition() == downstream)
                      .findFirst()
                      .get();
              CEnclaveInstance downEnclave = enclaveMap.get(down);
              graph.addEdges(enc, downEnclave, downStreamMap.get(downstream));
            }
          }
          var upstreamMap = ast.connGraph.getUpstreamOf(inst);
          for (Instantiation upstream : upstreamMap.keySet()) {
            if (upstream.equals(ast.PARENT)) {
              // If upstream was `null` then we have a connection to the parent.
              CEnclaveInstance upEnclave = enclaveMap.get(current.enclave);
              graph.addEdges(upEnclave, enc, upstreamMap.get(upstream));
            } else {
              ReactorInstance up =
                  current.children.stream()
                      .filter(i -> i.getDefinition() == upstream)
                      .findFirst()
                      .get();
              CEnclaveInstance upEnclave = enclaveMap.get(up);
              graph.addEdges(upEnclave, enc, upstreamMap.get(upstream));
            }
          }
        } else if (ast.enclavedConnections.containsKey(inst)) {
          ReactorInstance source = null;
          ReactorInstance dest = null;
          var envs = ast.enclavedConnections.get(inst);
          if (envs.first().equals(ast.PARENT)) {
            source = current;
          }
          if (envs.second().equals(ast.PARENT)) {
            dest = current;
          }

          for (ReactorInstance c : current.children) {
            if (envs.first().equals(c.getDefinition())) {
              source = c;
            }
            if (envs.second().equals(c.getDefinition())) {
              dest = c;
            }
          }
          ast.setEnvParams(child, source, dest);
        }
        queue.add(child);
      }
    }
  }
}
