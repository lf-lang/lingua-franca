package org.lflang.pretvm.register;

/**
 * An amount to increment the offset by (usually the current hyperperiod). This is global because
 * worker 0 applies the increment to all workers' offsets.
 */
public class OffsetInc extends GlobalRegister {
  public OffsetInc() {
    super();
  }
}
