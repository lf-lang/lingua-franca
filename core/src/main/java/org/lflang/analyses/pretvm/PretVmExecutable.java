package org.lflang.analyses.pretvm;

import java.util.List;

/**
 * Class defining a PRET VM executable
 *
 * @author Shaokai Lin
 */
public class PretVmExecutable {

  /**
   * Content is a list of list of instructions, where the inner list is a sequence of instructions
   * for a worker, and the outer list is a list of instruction sequences, one for each worker.
   */
  private List<List<Instruction>> content;

  /** Constructor */
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
