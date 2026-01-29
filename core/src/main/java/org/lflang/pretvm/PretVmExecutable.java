package org.lflang.pretvm;

import java.util.List;
import org.lflang.pretvm.instruction.Instruction;

/** A PRET VM executable: a list of instruction sequences, one per worker. */
public class PretVmExecutable {

  private List<List<Instruction>> content;

  public PretVmExecutable(List<List<Instruction>> instructions) {
    this.content = instructions;
  }

  public List<List<Instruction>> getContent() {
    return content;
  }

  public void setContent(List<List<Instruction>> updatedContent) {
    this.content = updatedContent;
  }
}
