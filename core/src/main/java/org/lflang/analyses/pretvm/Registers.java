package org.lflang.analyses.pretvm;

import java.util.ArrayList;
import java.util.List;

/**
 * PretVM registers
 *
 * <p>FIXME: Should this be a record instead?
 */
public class Registers {
  public final Register registerStartTime = Register.START_TIME;
  public final Register registerOffset = Register.OFFSET;
  public final Register registerOffsetInc = Register.OFFSET_INC;
  public final Register registerOne = Register.ONE;
  public final Register registerTimeout = Register.TIMEOUT;
  public final Register registerZero = Register.ZERO;
  public List<Register> registerBinarySemas = new ArrayList<>();
  public List<Register> registerCounters = new ArrayList<>();
  public List<Register> registerReturnAddrs = new ArrayList<>();
  public List<Register> runtimeRegisters = new ArrayList<>();

  /**
   * A utility function that checks if a runtime register is already created. If so, it returns the
   * instantiated register. Otherwise, it instantiates the register and adds it to the
   * runtimeRegisters list.
   *
   * @param regString The C pointer address for which the register is created
   * @return a runtime register
   */
  public Register getRuntimeRegister(String regString) {
    Register temp = Register.createRuntimeRegister(regString);
    int index = runtimeRegisters.indexOf(temp);
    if (index == -1) {
      // Not found in the list of already instantiated runtime registers.
      // So add to the list.
      runtimeRegisters.add(temp);
      return temp;
    } else {
      // Found in the list. Simply return the register in list.
      return runtimeRegisters.get(index);
    }
  }
}
