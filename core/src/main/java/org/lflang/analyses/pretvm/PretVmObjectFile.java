package org.lflang.analyses.pretvm;

import java.util.List;
import org.lflang.analyses.statespace.StateSpaceFragment;

/**
 * A PRET VM Object File is a list of list of instructions and a hyperiod. Each list of instructions is
 * for a worker.
 */
public class PretVmObjectFile extends PretVmExecutable {

  private StateSpaceFragment fragment; // Useful for linking.

  public PretVmObjectFile(List<List<Instruction>> instructions, StateSpaceFragment fragment) {
    super(instructions);
    this.fragment = fragment;
  }

  public StateSpaceFragment getFragment() {
    return fragment;
  }
}
