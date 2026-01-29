package org.lflang.pretvm;

import java.util.List;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.instruction.Instruction;

/**
 * A PretVM object file: instruction sequences per worker, plus the state space diagram and
 * partitioned DAG for linking.
 */
public class PretVmObjectFile extends PretVmExecutable {

  private StateSpaceDiagram diagram;
  private Dag dagPartitioned;

  public PretVmObjectFile(
      List<List<Instruction>> instructions, StateSpaceDiagram diagram, Dag dagPartitioned) {
    super(instructions);
    this.diagram = diagram;
    this.dagPartitioned = dagPartitioned;
  }

  public StateSpaceDiagram getDiagram() {
    return diagram;
  }

  public Dag getDag() {
    return dagPartitioned;
  }

  /** Pretty print all worker instruction sequences. */
  public void display() {
    List<List<Instruction>> instructions = this.getContent();
    for (int i = 0; i < instructions.size(); i++) {
      List<Instruction> schedule = instructions.get(i);
      System.out.println("Worker " + i + ":");
      for (int j = 0; j < schedule.size(); j++) {
        System.out.println(schedule.get(j));
      }
    }
  }
}
