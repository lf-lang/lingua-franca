package org.lflang.analyses.evm;

import java.util.List;
import org.lflang.analyses.statespace.StateSpaceFragment;

/**
 * An EVM Object File is a list of list of instructions and a hyperiod. Each list of instructions is
 * for a worker.
 */
public class EvmObjectFile extends EvmExecutable {

  private StateSpaceFragment fragment; // Useful for linking.

  public EvmObjectFile(List<List<Instruction>> instructions, StateSpaceFragment fragment) {
    super(instructions);
    this.fragment = fragment;
  }

  public StateSpaceFragment getFragment() {
    return fragment;
  }

}
