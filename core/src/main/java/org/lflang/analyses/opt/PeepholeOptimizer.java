package org.lflang.analyses.opt;

import java.util.ArrayList;
import java.util.List;

import org.lflang.analyses.pretvm.Instruction;
import org.lflang.analyses.pretvm.InstructionWU;

public class PeepholeOptimizer {
    
    public static List<Instruction> optimize(List<Instruction> instructions) {

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
        return instructions;
    }

    public static List<Instruction> optimizeWindow(List<Instruction> window) {
        // Here, window size could vary from 2 to 5 based on incoming patterns
        // This method is called by the main optimize function with different sized windows
        List<Instruction> optimized = new ArrayList<>(window);

        // Invoke optimizations for size >= 2.
        if (window.size() >= 2) {
            removeSmallerWU(window, optimized);
        }
        return optimized;
    }

    /**
     * When there are two consecutive WU instructions, keep the one waiting on a
     * larger release value. The window size is 2.
     * @param original
     * @param optimized
     */
    public static void removeSmallerWU(List<Instruction> original, List<Instruction> optimized) {
        if (original.size() == 2) {
            Instruction first = original.get(0);
            Instruction second = original.get(1);
            if (first instanceof InstructionWU firstWU && second instanceof InstructionWU secondWU) {
                if (firstWU.releaseValue < secondWU.releaseValue) {
                    optimized.remove(0);
                } else {
                    optimized.remove(1);
                }
            }
        }
    }
}
