package org.lflang.analyses.pretvm;

import java.util.List;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.statespace.StateSpaceFragment;

/**
 * A PretVM Object File is a list of list of instructions, each list of which is for a worker. The
 * object file also contains a state space fragment and a partitioned DAG for this fragment.
 *
 * @author Shaokai Lin
 */
public class PretVmObjectFile extends PretVmExecutable {

  private StateSpaceFragment fragment; // Useful for linking.
  private Dag dagParitioned;

  public PretVmObjectFile(
      List<List<Instruction>> instructions, StateSpaceFragment fragment, Dag dagParitioned) {
    super(instructions);
    this.fragment = fragment;
    this.dagParitioned = dagParitioned;
  }

  public StateSpaceFragment getFragment() {
    return fragment;
  }

  public Dag getDag() {
    return dagParitioned;
  }

  /** Pretty printing instructions */
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
