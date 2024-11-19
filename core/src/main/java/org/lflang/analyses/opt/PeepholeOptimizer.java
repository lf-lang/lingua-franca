package org.lflang.analyses.opt;

import java.util.ArrayList;
import java.util.List;
import org.lflang.analyses.pretvm.instructions.Instruction;
import org.lflang.analyses.pretvm.instructions.InstructionWU;

public class PeepholeOptimizer {

  public static void optimize(List<Instruction> instructions) {

    // Move the sliding window down.
    int maxWindowSize = 2; // Currently 2, but could be extended if larger windows are needed.
    int i = 0;
    while (i < instructions.size()) {
      boolean changed = false;
      for (int windowSize = 2; windowSize <= maxWindowSize; windowSize++) {
        if (i + windowSize >= instructions.size()) {
          break; // Avoid out-of-bound error.
        }
        List<Instruction> window = instructions.subList(i, i + windowSize);
        List<Instruction> optimizedWindow = optimizeWindow(window);
        if (!optimizedWindow.equals(window)) {
          changed = true;
          instructions.subList(i, i + windowSize).clear();
          instructions.addAll(i, optimizedWindow);
        }
      }
      // Only slide the window when nothing is changed.
      // If the code within a window change, apply optimizations again.
      if (!changed) i++;
    }
  }

  public static List<Instruction> optimizeWindow(List<Instruction> window) {
    // Here, window size could vary from 2 to 5 based on incoming patterns
    // This method is called by the main optimize function with different sized windows
    List<Instruction> optimized = new ArrayList<>(window);

    // Invoke optimizations for size >= 2.
    if (window.size() >= 2) {
      // Optimize away redundant WUs.
      removeRedundantWU(window, optimized);
    }
    return optimized;
  }

  /**
   * When there are two consecutive WU instructions, keep the one waiting on a larger release value.
   * The window size is 2.
   *
   * @param original
   * @param optimized
   */
  public static void removeRedundantWU(List<Instruction> original, List<Instruction> optimized) {
    if (optimized.size() == 2) {
      Instruction first = optimized.get(0);
      Instruction second = optimized.get(1);
      Instruction removed;
      if (first instanceof InstructionWU firstWU && second instanceof InstructionWU secondWU) {
        if (firstWU.getOperand2() < secondWU.getOperand2()) {
          removed = optimized.remove(0);
        } else {
          removed = optimized.remove(1);
        }
        // At this point, one WU has been removed.
        // Transfer the labels from the removed WU to the survived WU.
        if (removed.getLabelList() != null) optimized.get(0).addLabels(removed.getLabelList());
      }
    }
  }
}
