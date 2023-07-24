package org.lflang.analyses.evm;

import java.util.List;

public class EvmExecutable {

  private List<List<Instruction>> content;

  public EvmExecutable(List<List<Instruction>> instructions) {
    this.content = instructions;
  }

  public List<List<Instruction>> getContent() {
    return content;
  }
}
