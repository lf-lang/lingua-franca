package org.lflang.analyses.evm;

import java.util.List;

public class EvmExecutable {

  private List<List<Instruction>> content;
  private Long hyperperiod;

  public EvmExecutable(List<List<Instruction>> instructions, Long hyperperiod) {
    this.content = instructions;
    this.hyperperiod = hyperperiod;
  }

  public List<List<Instruction>> getContent() {
    return content;
  }

  public Long getHyperperiod() {
    return hyperperiod;
  }
}
