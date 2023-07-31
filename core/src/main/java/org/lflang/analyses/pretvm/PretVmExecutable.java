package org.lflang.analyses.pretvm;

import java.util.List;

public class PretVmExecutable {

  private List<List<Instruction>> content;

  public PretVmExecutable(List<List<Instruction>> instructions) {
    this.content = instructions;
  }

  public List<List<Instruction>> getContent() {
    return content;
  }
}
