package org.lflang.pretvm.opt;

import java.util.List;
import org.lflang.pretvm.instruction.Instruction;

/** Base class for PretVM optimizers. */
public abstract class PretVMOptimizer {
  public static void optimize(List<Instruction> instructions) {
    throw new UnsupportedOperationException("Unimplemented method 'optimize'");
  }
}
