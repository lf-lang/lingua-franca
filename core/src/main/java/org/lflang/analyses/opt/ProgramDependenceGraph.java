package org.lflang.analyses.opt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.lflang.analyses.pretvm.Instruction;

class ProgramDependenceGraph {
    Map<Instruction, List<Instruction>> controlDependencies = new HashMap<>();
    Map<Instruction, List<Instruction>> dataDependencies = new HashMap<>();

    void addControlDependency(Instruction from, Instruction to) {
        controlDependencies.computeIfAbsent(from, k -> new ArrayList<>()).add(to);
    }

    void addDataDependency(Instruction from, Instruction to) {
        dataDependencies.computeIfAbsent(from, k -> new ArrayList<>()).add(to);
    }
}
