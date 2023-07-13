package org.lflang.analyses.evm;

import java.util.List;

/**
 * An EVM Object File is a list of list of instructions and a hyperiod. Each list of instructions is
 * for a worker.
 */
public class EvmObjectFile {

  private List<List<Instruction>> content;
  private Long hyperperiod;

  public EvmObjectFile(List<List<Instruction>> instructions, Long hyperperiod) {
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
