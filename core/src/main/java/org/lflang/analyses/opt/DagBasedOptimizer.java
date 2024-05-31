package org.lflang.analyses.opt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.analyses.pretvm.Instruction;
import org.lflang.analyses.pretvm.InstructionJAL;
import org.lflang.analyses.pretvm.InstructionJALR;
import org.lflang.analyses.pretvm.PretVmLabel;
import org.lflang.analyses.pretvm.PretVmObjectFile;
import org.lflang.analyses.pretvm.Registers;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

public class DagBasedOptimizer extends PretVMOptimizer {
    
    // FIXME: Store the number of workers set in the DAG, instead of passing in
    // from the outside separately. 
    public static void optimize(PretVmObjectFile objectFile, int workers, Registers registers) {
        // A list of list of nodes, where each list of nodes can be factored
        // into a procedure. The index of the list is procedure index.
        List<List<DagNode>> equivalenceClasses = new ArrayList<>();

        // For a quick lookup, map each node to its procedure index.
        Map<DagNode, Integer> nodeToProcedureIndexMap = new HashMap<>();

        // Populate the above data structures.
        populateEquivalenceClasses(objectFile, equivalenceClasses, nodeToProcedureIndexMap);

        // Factor out procedures.
        factorOutProcedures(objectFile, registers, equivalenceClasses, nodeToProcedureIndexMap, workers);
    }

    /**
     * Finds equivalence classes in the bytecode, i.e., lists of nodes with
     * identical generated instructions. 
     */
    private static void populateEquivalenceClasses(
        PretVmObjectFile objectFile,
        List<List<DagNode>> equivalenceClasses,
        Map<DagNode, Integer> nodeToProcedureIndexMap
    ) {
        // Iterate over the topological sort of the DAG and find nodes that
        // produce the same instructions.
        Dag dag = objectFile.getDag();
        for (DagNode node : dag.getTopologicalSort()) {
            // Only consider reaction nodes because they generate instructions.
            // if (node.nodeType != dagNodeType.REACTION) continue;
            // System.out.println("Current node instructions: " + node.getInstructions());
            boolean matched = false;
            for (int i = 0; i < equivalenceClasses.size(); i++) {
                List<DagNode> list = equivalenceClasses.get(i);
                DagNode listHead = list.get(0);
                if (node.hasSameInstructionsAs(listHead)) {
                    list.add(node);
                    matched = true;
                    nodeToProcedureIndexMap.put(node, i);
                }
                // else {
                //     System.out.println("DO NOT MATCH: " + node + " , " + listHead);
                // }
            }
            // If a node does not match with any existing nodes,
            // start a new list.
            if (!matched) {
                equivalenceClasses.add(new ArrayList<>(List.of(node)));
                nodeToProcedureIndexMap.put(node, equivalenceClasses.size() - 1);
            }
        }
        System.out.println(equivalenceClasses);
        System.out.println(nodeToProcedureIndexMap);
    }

    private static void factorOutProcedures(
        PretVmObjectFile objectFile,
        Registers registers,
        List<List<DagNode>> equivalenceClasses,
        Map<DagNode, Integer> nodeToProcedureIndexMap,
        int workers
    ) {
        // Get the partitioned DAG and Phase.
        Dag dag = objectFile.getDag();
        Phase phase = objectFile.getFragment().getPhase();

        // Instantiate a list of updated instructions
        List<List<Instruction>> updatedInstructions = new ArrayList<>();
        for (int w = 0; w < workers; w++) {
            updatedInstructions.add(new ArrayList<>());
        }
        
        // Record the procedures used by each worker. The index of the outer
        // list matches a worker number, and the inner list contains procedure
        // indices used by this worker.
        List<Set<Integer>> proceduresUsedByWorkers = new ArrayList<>();
        for (int i = 0; i < workers; i++) {
            proceduresUsedByWorkers.add(new HashSet<>());
        }
        
        // Update proceduresUsedByWorkers.
        for (DagNode node : dag.getTopologicalSort()) {
            // Look up the procedure index
            Integer procedureIndex = nodeToProcedureIndexMap.get(node);
            if (node.nodeType == dagNodeType.REACTION) {
                // Add the procedure index to proceduresUsedByWorkers.
                int worker = node.getWorker();
                proceduresUsedByWorkers.get(worker).add(procedureIndex);
            }
        }

        // Generate procedures first.
        for (int w = 0; w < workers; w++) {
            Set<Integer> procedureIndices = proceduresUsedByWorkers.get(w);
            for (Integer procedureIndex : procedureIndices) {
                // Look up the instructions in the first node in the equivalence class list.
                List<Instruction> procedureCode = equivalenceClasses.get(procedureIndex).get(0).getInstructions();
                
                // Remove any phase labels from the procedure code.
                // We need to do this because new phase labels will be
                // added later in this optimizer pass.
                if (procedureCode.get(0).hasLabel()) { // Only check the first instruction.
                    List<PretVmLabel> labels = procedureCode.get(0).getLabelList();
                    for (int i = 0; i < labels.size(); i++) {
                        try {
                            // Check if label is a phase.
                            Enum.valueOf(Phase.class, labels.get(i).toString());
                            // If so, remove it.
                            labels.remove(i);
                            break;
                        } catch (IllegalArgumentException e) {
                            // Otherwise an error is raised, do nothing.
                        }
                    }
                }

                // Set / append a procedure label.
                procedureCode.get(0).setLabel(phase + "_PROCEDURE_" + procedureIndex);

                System.out.println("Procedure code to be added: ");
                for (var inst: procedureCode) {
                    System.out.println(inst);
                }

                // Add instructions to the worker instruction list.
                // FIXME: We likely need a clone here if there are multiple workers.
                updatedInstructions.get(w).addAll(procedureCode);

                // Jump back to the call site.
                updatedInstructions.get(w).add(new InstructionJALR(registers.registerZero, registers.registerReturnAddrs.get(w), 0L));
            }
        }

        // Store locations to set a phase label for the optimized object code.
        int[] phaseLabelLoc = new int[workers];
        for (int w = 0; w < workers; w++) {
            phaseLabelLoc[w] = updatedInstructions.get(w).size();
        }

        // Generate code in the next topological sort.
        for (DagNode node : dag.getTopologicalSort()) {
            if (node.nodeType == dagNodeType.REACTION) {
                // Generate code for jumping to the procedure index.
                int w = node.getWorker();
                Integer procedureIndex = nodeToProcedureIndexMap.get(node);
                updatedInstructions.get(w).add(new InstructionJAL(registers.registerReturnAddrs.get(w), phase + "_PROCEDURE_" + procedureIndex));
            }
            else if (node == dag.tail) {
                // If the node is a tail node, simply copy the code.
                // FIXME: We cannot do a jump to procedure here because the tail
                // node also jumps to SYNC_BLOCK, which can be considered as
                // another procedure call. There currently isn't a method
                // for nesting procedures calls. One strategy is to temporarily use
                // a register to save the outer return address. Then, when the
                // inner procedure call returns, update the return address
                // variable from the temp register.
                for (int w = 0; w < workers; w++) {
                    // Add instructions from this node.
                    updatedInstructions.get(w).addAll(node.getInstructions(w));
                }
            }
        }

        // Add a label to the first instruction using the exploration phase
        // (INIT, PERIODIC, SHUTDOWN_TIMEOUT, etc.).
        for (int w = 0; w < workers; w++) {
            updatedInstructions.get(w).get(phaseLabelLoc[w]).setLabel(phase.toString());
        }

        // Update the object file.
        objectFile.setContent(updatedInstructions);

        for (int w = 0; w < workers; w++) {
            System.out.println("Worker " + w + ": ");
            for (Instruction inst : updatedInstructions.get(w)) {
                System.out.println(inst);
            }
        }
    }

    // FIXME: Check if a procedure is reused.
    // private static boolean procedureIsReused() {

    // }
}
