// package org.lflang.analyses.opt;

// import java.util.ArrayList;
// import java.util.HashMap;
// import java.util.Map;

// import org.lflang.analyses.pretvm.Instruction;
// import org.lflang.analyses.pretvm.InstructionADD;
// import org.lflang.analyses.pretvm.InstructionADDI;
// import org.lflang.analyses.pretvm.InstructionBEQ;
// import org.lflang.analyses.pretvm.InstructionJAL;
// import org.lflang.analyses.pretvm.InstructionJALR;

// public class PDGBuilder {
//     public static ProgramDependenceGraph buildPDG(ArrayList<Instruction> instructions) {
//         ProgramDependenceGraph pdg = new ProgramDependenceGraph();
//         Instruction lastControlInstruction = null;
//         Map<String, Instruction> lastWriter = new HashMap<>();

//         for (Instruction instruction : instructions) {
//             if (instruction instanceof InstructionADD) {
//                 InstructionADD inst = (InstructionADD) instruction;
//                 if (lastWriter.containsKey(inst.src1)) pdg.addDataDependency(lastWriter.get(inst.src1), inst);
//                 if (lastWriter.containsKey(inst.src2)) pdg.addDataDependency(lastWriter.get(inst.src2), inst);
//                 lastWriter.put(inst.dest, inst);
//             } else if (instruction instanceof InstructionADDI) {
//                 InstructionADDI inst = (InstructionADDI) instruction;
//                 if (lastWriter.containsKey(inst.src)) pdg.addDataDependency(lastWriter.get(inst.src), inst);
//                 lastWriter.put(inst.dest, inst);
//             }
//             // Add handling for other types similarly

//             if (instruction instanceof InstructionBEQ || instruction instanceof InstructionJAL || instruction instanceof InstructionJALR) {
//                 if (lastControlInstruction != null) pdg.addControlDependency(lastControlInstruction, instruction);
//                 lastControlInstruction = instruction;
//             }
//         }

//         return pdg;
//     }
// }