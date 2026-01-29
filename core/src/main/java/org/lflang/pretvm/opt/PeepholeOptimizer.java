package org.lflang.pretvm.opt;

import java.util.ArrayList;
import java.util.List;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.pretvm.instruction.WU;

/** Peephole optimizer that applies local optimizations over a sliding window of instructions. */
public class PeepholeOptimizer {

  public static void optimize(List<Instruction> instructions) {
    int maxWindowSize = 2;
    int i = 0;
    while (i < instructions.size()) {
      boolean changed = false;
      for (int windowSize = 2; windowSize <= maxWindowSize; windowSize++) {
        if (i + windowSize >= instructions.size()) {
          break;
        }
        List<Instruction> window = instructions.subList(i, i + windowSize);
        List<Instruction> optimizedWindow = optimizeWindow(window);
        if (!optimizedWindow.equals(window)) {
          changed = true;
          instructions.subList(i, i + windowSize).clear();
          instructions.addAll(i, optimizedWindow);
        }
      }
      if (!changed) i++;
    }
  }

  public static List<Instruction> optimizeWindow(List<Instruction> window) {
    List<Instruction> optimized = new ArrayList<>(window);
    if (window.size() >= 2) {
      removeRedundantWU(window, optimized);
    }
    return optimized;
  }

  /**
   * When there are two consecutive WU instructions, keep the one waiting on a larger release value.
   */
  public static void removeRedundantWU(List<Instruction> original, List<Instruction> optimized) {
    if (optimized.size() == 2) {
      Instruction first = optimized.get(0);
      Instruction second = optimized.get(1);
      Instruction removed;
      if (first instanceof WU firstWU && second instanceof WU secondWU) {
        if (firstWU.getOperand2() < secondWU.getOperand2()) {
          removed = optimized.remove(0);
        } else {
          removed = optimized.remove(1);
        }
        if (removed.getLabels() != null) optimized.get(0).addLabels(removed.getLabels());
      }
    }
  }
}
