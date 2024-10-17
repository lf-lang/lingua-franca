package org.lflang.analyses.pretvm;

import java.util.ArrayList;
import java.util.List;

/**
 * PretVM registers
 *
 * <p>FIXME: Should this be a record instead?
 */
public class Registers {
  public final Register startTime = new Register(RegisterType.START_TIME);
  public final Register offset = new Register(RegisterType.OFFSET);
  public final Register offsetInc = new Register(RegisterType.OFFSET_INC);
  public final Register one = new Register(RegisterType.ONE);
  public final Register timeout = new Register(RegisterType.TIMEOUT);
  public final Register zero = new Register(RegisterType.ZERO);
  public List<Register> binarySemas = new ArrayList<>();
  public List<Register> counters = new ArrayList<>();
  public List<Register> returnAddrs = new ArrayList<>();
  public List<Register> runtime = new ArrayList<>();
  public List<Register> temp0 = new ArrayList<>();
  public List<Register> temp1 = new ArrayList<>();

  // Abstract worker registers whose owner needs to be defined later.
  public static final Register ABSTRACT_WORKER_RETURN_ADDR = new Register(RegisterType.RETURN_ADDR);

  /**
   * A utility function that checks if a runtime register is already created. If so, it returns the
   * instantiated register. Otherwise, it instantiates the register and adds it to the runtime list.
   *
   * @param regString The C pointer address for which the register is created
   * @return a runtime register
   */
  public Register getRuntimeRegister(String regString) {
    Register temp = Register.createRuntimeRegister(regString);
    int index = runtime.indexOf(temp);
    if (index == -1) {
      // Not found in the list of already instantiated runtime registers.
      // So add to the list.
      runtime.add(temp);
      return temp;
    } else {
      // Found in the list. Simply return the register in list.
      return runtime.get(index);
    }
  }
}
