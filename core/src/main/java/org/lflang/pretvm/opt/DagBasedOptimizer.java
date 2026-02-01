package org.lflang.pretvm.opt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.lflang.pretvm.ExecutionPhase;
import org.lflang.pretvm.Label;
import org.lflang.pretvm.PartialSchedule;
import org.lflang.pretvm.Registers;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagNode;
import org.lflang.pretvm.dag.JobNode;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.pretvm.instruction.JAL;
import org.lflang.pretvm.instruction.JALR;

/**
 * Optimizer that identifies equivalent reaction-node instruction sequences in the DAG and factors
 * them into shared procedures, replacing duplicates with JAL/JALR call/return pairs.
 */
public class DagBasedOptimizer extends PretVMOptimizer {

  public static void optimize(PartialSchedule schedule, int workers, Registers registers) {
    List<List<DagNode>> equivalenceClasses = new ArrayList<>();
    Map<DagNode, Integer> nodeToProcedureIndexMap = new HashMap<>();

    populateEquivalenceClasses(schedule, equivalenceClasses, nodeToProcedureIndexMap);
    factorOutProcedures(schedule, registers, equivalenceClasses, nodeToProcedureIndexMap, workers);
  }

  /** Finds equivalence classes: lists of nodes with identical generated instructions. */
  private static void populateEquivalenceClasses(
      PartialSchedule schedule,
      List<List<DagNode>> equivalenceClasses,
      Map<DagNode, Integer> nodeToProcedureIndexMap) {
    Dag dag = schedule.getDag();
    for (DagNode node : dag.getTopologicalSort()) {
      if (!(node instanceof JobNode jobNode)) continue;
      int worker = jobNode.getWorker();
      List<Instruction> workerInstructions = schedule.getInstructions().get(worker);
      boolean matched = false;
      for (int i = 0; i < equivalenceClasses.size(); i++) {
        List<DagNode> list = equivalenceClasses.get(i);
        DagNode listHead = list.get(0);
        if (node.filterInstructions(workerInstructions)
            .equals(listHead.filterInstructions(workerInstructions))) {
          list.add(node);
          matched = true;
          nodeToProcedureIndexMap.put(node, i);
          break;
        }
      }
      if (!matched) {
        equivalenceClasses.add(new ArrayList<>(List.of(node)));
        nodeToProcedureIndexMap.put(node, equivalenceClasses.size() - 1);
      }
    }
  }

  /** Factor out each procedure. Works at the level of partial schedules (phases). */
  private static void factorOutProcedures(
      PartialSchedule schedule,
      Registers registers,
      List<List<DagNode>> equivalenceClasses,
      Map<DagNode, Integer> nodeToProcedureIndexMap,
      int workers) {
    Dag dag = schedule.getDag();
    ExecutionPhase phase = schedule.getPhase();

    List<List<Instruction>> updatedInstructions = new ArrayList<>();
    for (int w = 0; w < workers; w++) {
      updatedInstructions.add(new ArrayList<>());
    }

    // Record procedures used by each worker.
    List<Set<Integer>> proceduresUsedByWorkers = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      proceduresUsedByWorkers.add(new HashSet<>());
    }
    for (DagNode node : dag.getTopologicalSort()) {
      Integer procedureIndex = nodeToProcedureIndexMap.get(node);
      if (node instanceof JobNode jobNode) {
        proceduresUsedByWorkers.get(jobNode.getWorker()).add(procedureIndex);
      }
    }

    // Generate procedures first.
    for (int w = 0; w < workers; w++) {
      Set<Integer> procedureIndices = proceduresUsedByWorkers.get(w);
      for (Integer procedureIndex : procedureIndices) {
        List<Instruction> workerInstructions = schedule.getInstructions().get(w);
        DagNode listHead = equivalenceClasses.get(procedureIndex).get(0);
        List<Instruction> procedureCode = listHead.filterInstructions(workerInstructions);

        // Remove any phase labels from the first instruction of the procedure code.
        if (procedureCode.get(0).hasLabel()) {
          List<Label> labels = procedureCode.get(0).getLabels();
          for (int i = 0; i < labels.size(); i++) {
            try {
              Enum.valueOf(ExecutionPhase.class, labels.get(i).toString());
              labels.remove(i);
              break;
            } catch (IllegalArgumentException e) {
              // Not a phase label, skip.
            }
          }
        }

        // Set a procedure label.
        procedureCode.get(0).addLabel(new Label(phase + "_PROCEDURE_" + procedureIndex));

        updatedInstructions.get(w).addAll(procedureCode);

        // Jump back to the call site.
        updatedInstructions.get(w).add(new JALR(registers.zero, registers.returnAddrs.get(w), 0L));
      }
    }

    // Store locations to set a phase label for the optimized object code.
    int[] phaseLabelLoc = new int[workers];
    for (int w = 0; w < workers; w++) {
      phaseLabelLoc[w] = updatedInstructions.get(w).size();
    }

    // Generate code in the topological sort order.
    for (DagNode node : dag.getTopologicalSort()) {
      if (node instanceof JobNode jobNode) {
        int w = jobNode.getWorker();
        Integer procedureIndex = nodeToProcedureIndexMap.get(node);
        updatedInstructions
            .get(w)
            .add(
                new JAL(
                    registers.returnAddrs.get(w),
                    new Label(phase + "_PROCEDURE_" + procedureIndex)));
      } else if (node == dag.end) {
        // End node: copy instructions directly for all workers.
        for (int w = 0; w < workers; w++) {
          List<Instruction> workerInstructions = schedule.getInstructions().get(w);
          updatedInstructions.get(w).addAll(node.filterInstructions(workerInstructions));
        }
      }
    }

    // Add phase label to the first non-procedure instruction per worker.
    for (int w = 0; w < workers; w++) {
      updatedInstructions.get(w).get(phaseLabelLoc[w]).addLabel(new Label(phase.toString()));
    }

    schedule.setInstructions(updatedInstructions);
  }
}
