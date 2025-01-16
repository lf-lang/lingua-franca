package org.lflang.pretvm.instruction;

/**
 * Class defining the STP instruction
 *
 * <p>STP : Stop the execution.
 *
 * @author Shaokai J. Lin
 */
public class STP extends Instruction<Object, Object, Object> {

  @Override
  public Instruction<Object, Object, Object> clone() {
    return new STP();
  }

  @Override
  public String toString() {
    return "STP";
  }
}
