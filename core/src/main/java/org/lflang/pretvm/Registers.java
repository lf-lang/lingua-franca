package org.lflang.pretvm;

import java.util.ArrayList;
import java.util.List;
import org.lflang.pretvm.register.*;

/** A set of registers used by the PretVM */
public final class Registers {
  public final Register startTime;
  public final Register offset;
  public final Register offsetInc;
  public final Register one;
  public final Register timeout;
  public final Register zero;
  public final List<Register> binarySemas = new ArrayList<>();
  public final List<Register> progressIndices = new ArrayList<>();
  public final List<Register> returnAddrs = new ArrayList<>();
  public final List<Register> runtimeVars = new ArrayList<>();
  public final List<Register> temp0 = new ArrayList<>();
  public final List<Register> temp1 = new ArrayList<>();

  /** Constructor */
  public Registers(int workers) {

    // Instantiate global registers.
    startTime = new StartTime();
    offset = new Offset();
    offsetInc = new OffsetInc();
    one = new One();
    timeout = new Timeout();
    zero = new Zero();

    // Instantiate worker registers.
    for (int w = 0; w < workers; w++) {
      binarySemas.add(new BinarySema(w));
      progressIndices.add(new ProgressIndex(w));
      returnAddrs.add(new ReturnAddr(w));
      temp0.add(new Temp0(w));
      temp1.add(new Temp1(w));
    }
  }

  /**
   * A utility function that checks if a runtimeVars register is already created. If so, it returns
   * the instantiated register. Otherwise, it instantiates the register and adds it to the
   * runtimeVars list.
   *
   * @param regString The C pointer address for which the register is created
   * @return a runtimeVars register
   */
  public Register getRuntimeVar(String regString) {
    Register temp = new RuntimeVar(regString);
    int index = runtimeVars.indexOf(temp);
    if (index == -1) {
      // Not found in the list of already instantiated runtimeVars registers.
      // So add to the list.
      runtimeVars.add(temp);
      return temp;
    } else {
      // Found in the list. Simply return the register in list.
      return runtimeVars.get(index);
    }
  }
}
