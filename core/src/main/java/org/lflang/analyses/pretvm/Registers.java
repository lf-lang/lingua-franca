package org.lflang.analyses.pretvm;

import java.util.ArrayList;
import java.util.List;

/**
 * PretVM registers
 * 
 * FIXME: Should this be a record instead?
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
}
