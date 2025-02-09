package org.lflang.pretvm;

import java.util.ArrayList;
import java.util.List;

import org.lflang.pretvm.register.*;

/**
 * A set of registers used by the PretVM
 */
public final class Registers {
  public final Register startTime;
  public final Register offset;
  public final Register offsetInc;
  public final Register one;
  public final Register timeout;
  public final Register zero;
  public final List<Register> binarySemas = new ArrayList<>();
  public final List<Register> counters    = new ArrayList<>();
  public final List<Register> returnAddrs = new ArrayList<>();
  public final List<Register> runtime     = new ArrayList<>();
  public final List<Register> temp0       = new ArrayList<>();
  public final List<Register> temp1       = new ArrayList<>();

  /** Constructor */
  public Registers(int workers) {
    
    // Instantiate global registers.
    startTime = new StartTime();
    offset    = new Offset();
    offsetInc = new OffsetInc();
    one       = new One();
    timeout   = new Timeout();
    zero      = new Zero();

    // Instantiate worker registers.
    for (int w = 0; w < workers; w++) {
      binarySemas.add(new BinarySema(w));
      counters.add(new ProgressIndex(w));
      returnAddrs.add(new ReturnAddr(w));
      temp0.add(new Temp0(w));
      temp1.add(new Temp1(w));
    }
  }
}