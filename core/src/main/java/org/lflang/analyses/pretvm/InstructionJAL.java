package org.lflang.analyses.pretvm;

import java.util.Objects;

/**
 * Class defining the JAL instruction
 * 
 * @FIXME: Make the second parameter PretVmLabel type instead of an Object type.
 *         Currently, the second parameter needs to be wrapped in a call to
 *         PretVmLabel.getPhaseLabel() before being passed in, which is highly
 *         error-prone. This issue also exists for all branch instructions!
 *
 * @author Shaokai Lin
 */
public class InstructionJAL extends Instruction<Register,Object,Integer> {

  /** Constructor */
  public InstructionJAL(Register retAddr, Object targetLabel) {
    if (targetLabel instanceof String || targetLabel instanceof PretVmLabel) {
      this.opcode = Opcode.JAL;
      this.operand1 = retAddr; // A register to store the return address
      this.operand2 = targetLabel; // A target label to jump to
    }
    else throw new RuntimeException(
      "TargetLabel must be either String or PretVmLabel. Label must be either Phase or PretVmLabel. targetLabel: " + targetLabel.getClass().getName());
  }
  
  public InstructionJAL(Register retAddr, Object targetLabel, Integer offset) {
    if (targetLabel instanceof String || targetLabel instanceof PretVmLabel) {
      this.opcode = Opcode.JAL;
      this.operand1 = retAddr; // A register to store the return address
      this.operand2 = targetLabel; // A target label to jump to
      this.operand3 = offset; // An additional offset
    }
    else throw new RuntimeException(
      "TargetLabel must be either String or PretVmLabel. Label must be either Phase or PretVmLabel. targetLabel: " + targetLabel.getClass().getName());
  }

  @Override
  public String toString() {
    return "JAL: " + "store return address in " + this.operand1 + " and jump to " + this.operand2 + (this.operand3 == null ? "" : " + " + this.operand3);
  }

  @Override
  public Instruction<Register,Object,Integer> clone() {
    return new InstructionJAL(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionJAL that) {
      if (Objects.equals(this.operand1, that.operand1)
        && Objects.equals(this.operand2, that.operand2)
        && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}
