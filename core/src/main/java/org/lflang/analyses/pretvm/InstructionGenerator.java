package org.lflang.analyses.pretvm;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.UUID;
import java.util.stream.Collectors;

import org.lflang.FileConfig;
import org.lflang.TimeValue;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.analyses.statespace.StateSpaceFragment;
import org.lflang.analyses.statespace.StateSpaceUtils;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.DashProperty;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.TimeOutProperty;

/**
 * A generator that generates PRET VM programs from DAGs. It also acts as a linker that piece
 * together multiple PRET VM object files.
 *
 * @author Shaokai Lin
 */
public class InstructionGenerator {

  /** File configuration */
  FileConfig fileConfig;

  /** Target configuration */
  TargetConfig targetConfig;

  /** Main reactor instance */
  protected ReactorInstance main;

  /** A list of reactor instances in the program */
  List<ReactorInstance> reactors;

  /** A list of reaction instances in the program */
  List<ReactionInstance> reactions;

  /** A list of trigger instances in the program */
  List<TriggerInstance> triggers;

  /** Number of workers */
  int workers;

  /** 
   * A mapping remembering where to fill in the placeholders
   * Each element of the list corresponds to a worker. The PretVmLabel marks the
   * line to be updated. The list of String is the set of variables to be
   * written into the instruction during deferred initialization. The index of
   * List<String> corresponds to the list of operands to be replaced in
   * sequential order for a particular instruction.
   */
  private List<Map<PretVmLabel, List<String>>> placeholderMaps = new ArrayList<>(); 

  /**
   * A nested map that maps a source port to a C function name, which updates a
   * priority queue holding tokens in a delayed connection. Each input can
   * identify a unique connection because no more than one connection can feed
   * into an input port.
   */
  private Map<PortInstance, String> preConnectionHelperFunctionNameMap  = new HashMap<>();
  private Map<PortInstance, String> postConnectionHelperFunctionNameMap = new HashMap<>();

  /**
   * A map that maps a trigger to a list of (BEQ) instructions where this trigger's
   * presence is tested.
   */
  private Map<TriggerInstance, List<Instruction>> triggerPresenceTestMap = new HashMap<>();

  /**
   * PretVM registers
   */
  private Registers registers;

  /** Constructor */
  public InstructionGenerator(
      FileConfig fileConfig,
      TargetConfig targetConfig,
      int workers,
      ReactorInstance main,
      List<ReactorInstance> reactors,
      List<ReactionInstance> reactions,
      List<TriggerInstance> triggers,
      Registers registers) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.workers = workers;
    this.main = main;
    this.reactors = reactors;
    this.reactions = reactions;
    this.triggers = triggers;
    this.registers = registers;
    for (int i = 0; i < this.workers; i++) {
      placeholderMaps.add(new HashMap<>());
      registers.registerBinarySemas.add(new Register(GlobalVarType.WORKER_BINARY_SEMA, i));
      registers.registerCounters.add(new Register(GlobalVarType.WORKER_COUNTER, i));
      registers.registerReturnAddrs.add(new Register(GlobalVarType.WORKER_RETURN_ADDR, i));
    }
  }

  /** Topologically sort the dag nodes and assign release values to DAG nodes for counting locks. */
  public void assignReleaseValues(Dag dagParitioned) {
    // Initialize a reaction index array to keep track of the latest counting
    // lock value for each worker.
    Long[] releaseValues = new Long[workers];
    Arrays.fill(releaseValues, 0L); // Initialize all elements to 0

    // Iterate over a topologically sorted list of dag nodes.
    for (DagNode current : dagParitioned.getTopologicalSort()) {
      if (current.nodeType == dagNodeType.REACTION) {
        releaseValues[current.getWorker()] += 1;
        current.setReleaseValue(releaseValues[current.getWorker()]);
      }
    }
  }

  /** Traverse the DAG from head to tail using Khan's algorithm (topological sort). */
  public PretVmObjectFile generateInstructions(Dag dagParitioned, StateSpaceFragment fragment) {
    // Map from a reactor to its latest associated SYNC node.
    // This is used to determine when ADVIs and DUs should be generated without
    // duplicating them for each reaction node in the same reactor.
    Map<ReactorInstance, DagNode> reactorToLastSeenSyncNodeMap = new HashMap<>();

    // Map an output port to its last seen EXE instruction at the current
    // tag. When we know for sure that no other reactions can modify a port, we then
    // go back to the last seen reaction-invoking EXE that can modify this port and
    // _insert_ a connection helper right after the last seen EXE in the schedule.
    // All the key value pairs in this map are waiting to be handled,
    // since all the output port values must be written to the buffers at the
    // end of the tag.
    Map<PortInstance, Instruction> portToUnhandledReactionExeMap = new HashMap<>();

    // Map a reaction to its last seen invocation, which is a DagNode.
    // If two invocations are mapped to different workers, a WU needs to
    // be generated to prevent race condition.
    // This map is used to check whether the WU needs to be generated.
    Map<ReactionInstance, DagNode> reactionToLastSeenInvocationMap = new HashMap<>();

    // Assign release values for the reaction nodes.
    assignReleaseValues(dagParitioned);

    // Instructions for all workers
    List<List<Instruction>> instructions = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }

    // Iterate over a topologically sorted list of dag nodes.
    for (DagNode current : dagParitioned.getTopologicalSort()) {
      // Get the upstream reaction nodes.
      List<DagNode> upstreamReactionNodes =
          dagParitioned.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n.nodeType == dagNodeType.REACTION)
              .toList();

      if (current.nodeType == dagNodeType.REACTION) {
        // Find the worker assigned to the REACTION node, 
        // the reactor, and the reaction.
        int worker = current.getWorker();
        ReactorInstance reactor = current.getReaction().getParent();
        ReactionInstance reaction = current.getReaction();

        // Current worker schedule
        List<Instruction> currentSchedule = instructions.get(worker);

        // Get the nearest upstream sync node.
        DagNode associatedSyncNode = current.getAssociatedSyncNode();

        // WU Case 1:
        // If the reaction depends on upstream reactions owned by other
        // workers, generate WU instructions to resolve the dependencies.
        // FIXME: The current implementation generates multiple unnecessary WUs
        // for simplicity. How to only generate WU when necessary?
        for (DagNode n : upstreamReactionNodes) {
          int upstreamOwner = n.getWorker();
          if (upstreamOwner != worker) {
            addInstructionForWorker(instructions, current.getWorker(), current, null, new InstructionWU(
              registers.registerCounters.get(upstreamOwner), n.getReleaseValue()));
          }
        }

        // WU Case 2:
        // FIXME: Is there a way to implement this using waitUntilDependencies
        // in the Dag class?
        // If the reaction has an _earlier_ invocation and is mapped to a
        // _different_ worker, then a WU needs to be generated to prevent from
        // processing of these two invocations of the same reaction in parallel.
        // If they are processed in parallel, the shared logical time field in
        // the reactor could get concurrent updates, resulting in incorrect
        // execution. 
        // Most often, there is not an edge between these two nodes,
        // making this a trickier case to handle.
        // The strategy here is to use a variable to remember the last seen
        // invocation of the same reaction instance.
        DagNode lastSeen = reactionToLastSeenInvocationMap.get(reaction);
        if (lastSeen != null && lastSeen.getWorker() != current.getWorker()) {
          addInstructionForWorker(instructions, current.getWorker(), current, null, new InstructionWU(
            registers.registerCounters.get(lastSeen.getWorker()), lastSeen.getReleaseValue()));
          if (current.getAssociatedSyncNode().timeStep.isEarlierThan(lastSeen.getAssociatedSyncNode().timeStep)) {
            System.out.println("FATAL ERROR: The current node is earlier than the lastSeen node. This case should not be possible and this strategy needs to be revised.");
            System.exit(1);
          }
        }
        reactionToLastSeenInvocationMap.put(reaction, current);

        // WU Case 3:
        // (Reference: Philosophers.lf using EGS with 2 workers)
        // If the node has an upstream dependency based on connection, but the
        // upstream is mapped to a different worker. Generate a WU.
        List<DagNode> upstreamsFromConnection = dagParitioned.waitUntilDependencies.get(current);
        if (upstreamsFromConnection != null && upstreamsFromConnection.size() > 0) {
          for (DagNode us : upstreamsFromConnection) {
            if (us.getWorker() != current.getWorker()) {
              addInstructionForWorker(instructions, current.getWorker(), current, null, new InstructionWU(
                registers.registerCounters.get(us.getWorker()), us.getReleaseValue()));
            }
          }
        }

        // When the new associated sync node _differs_ from the last associated sync
        // node of the reactor, this means that the current node's reactor needs
        // to advance to a new tag. The code should update the associated sync
        // node in the reactorToLastSeenSyncNodeMap map. And if
        // associatedSyncNode is not the head, generate ADVI and DU instructions. 
        if (associatedSyncNode != reactorToLastSeenSyncNodeMap.get(reactor)) {
          // Update the mapping.
          reactorToLastSeenSyncNodeMap.put(reactor, associatedSyncNode);

          // If the reaction depends on a single SYNC node,
          // advance to the LOGICAL time of the SYNC node first,
          // as well as delay until the PHYSICAL time indicated by the SYNC node.
          // Skip if it is the head node since this is done in the sync block.
          // FIXME: Here we have an implicit assumption "logical time is
          // physical time." We need to find a way to relax this assumption.
          // FIXME: One way to relax this is that "logical time is physical time
          // only when executing real-time reactions, otherwise fast mode for
          // non-real-time reactions."
          if (associatedSyncNode != dagParitioned.head) {
            
            // A pre-connection helper for an output port cannot be inserted
            // until we are sure that all reactions that can modify this port
            // at this tag has been invoked. At this point, since we have
            // detected time advancement, this condition is satisfied.
            // Iterate over all the ports of this reactor. We know at
            // this point that the EXE instruction stored in
            // portToUnhandledReactionExeMap is that the very last reaction
            // invocation that can modify these ports. So we can insert
            // pre-connection helpers after that reaction invocation.
            for (PortInstance output : reactor.outputs) {
              // Only generate for delayed connections.
              if (outputToDelayedConnection(output)) {
                Instruction lastPortModifyingReactionExe = portToUnhandledReactionExeMap.get(output);
                if (lastPortModifyingReactionExe != null) {
                  int exeWorker = lastPortModifyingReactionExe.getWorker();
                  int indexToInsert = instructions.get(exeWorker).indexOf(lastPortModifyingReactionExe) + 1;
                  generatePreConnectionHelper(output, instructions, exeWorker, indexToInsert, lastPortModifyingReactionExe.getDagNode());
                  // Remove the entry since this port is handled.
                  portToUnhandledReactionExeMap.remove(output);
                }
              }
            }

            // Generate an ADVI instruction.
            // FIXME: Factor out in a separate function.
            var advi = new InstructionADVI(
                        current.getReaction().getParent(),
                        registers.registerOffset,
                        associatedSyncNode.timeStep.toNanoSeconds());
            var uuid = generateShortUUID();
            advi.setLabel("ADVANCE_TAG_FOR_" + reactor.getFullNameWithJoiner("_") + "_" + uuid);
            placeholderMaps.get(current.getWorker()).put(
              advi.getLabel(),
              List.of(getReactorFromEnv(main, reactor)));
            addInstructionForWorker(instructions, worker, current, null, advi);
            // There are two cases for not generating a DU within a
            // hyperperiod: 1. if fast is on, 2. if dash is on and the parent
            // reactor is not realtime. 
            // Generate a DU instruction if neither case holds.
            if (!(targetConfig.get(FastProperty.INSTANCE)
                || (targetConfig.get(DashProperty.INSTANCE)
                && !reaction.getParent().reactorDefinition.isRealtime()))) {
              addInstructionForWorker(instructions, worker, current, null,
                new InstructionDU(associatedSyncNode.timeStep));
            }
          }
        }

        // Generate an EXE instruction for the current reaction.
        // FIXME: Handle a reaction triggered by both timers and ports.
        // Create an EXE instruction that invokes the reaction.
        // This instruction requires delayed instantiation.
        Instruction exe = new InstructionEXE(getPlaceHolderMacroString(), getPlaceHolderMacroString(), reaction.index);
        exe.setLabel("EXECUTE_" + reaction.getFullNameWithJoiner("_") + "_" + generateShortUUID());
        placeholderMaps.get(current.getWorker()).put(
            exe.getLabel(),
            List.of(
              getReactionFromEnv(main, reaction) + "->function",
              getReactorFromEnv(main, reaction.getParent())
            ));
        // Check if the reaction has BEQ guards or not.
        boolean hasGuards = false;
        // Create BEQ instructions for checking triggers.
        for (var trigger : reaction.triggers) {
          if (trigger instanceof PortInstance port && port.isInput()) {
            hasGuards = true;
            var beq = new InstructionBEQ(getPlaceHolderMacroRegister(), getPlaceHolderMacroRegister(), exe.getLabel());
            beq.setLabel("TEST_TRIGGER_" + port.getFullNameWithJoiner("_") + "_" + generateShortUUID());
                        
            // If connection has delay, check the connection buffer to see if
            // the earliest event matches the reactor's current logical time.
            if (inputFromDelayedConnection(port)) {
              placeholderMaps.get(current.getWorker()).put(
                beq.getLabel(),
                List.of(
                  "&" + getPqueueHeadFromEnv(main, port) + "->time",
                  "&" + getReactorFromEnv(main, reactor) + "->tag.time"
              ));
            }
            // Otherwise, if the connection has zero delay, check for the presence of the
            // downstream port.
            else {
              placeholderMaps.get(current.getWorker()).put(
                beq.getLabel(),
                List.of(
                  "&" + getTriggerIsPresentFromEnv(main, trigger), // The is_present field
                  getVarName(registers.registerOne, true) // is_present == 1
              ));
            }
            addInstructionForWorker(instructions, current.getWorker(), current, null, beq);
            // Update triggerPresenceTestMap.
            if (triggerPresenceTestMap.get(port) == null)
              triggerPresenceTestMap.put(port, new LinkedList<>());
            triggerPresenceTestMap.get(port).add(beq);
          }
        }

        // If none of the guards are activated, jump to one line after the
        // EXE instruction. 
        if (hasGuards) 
          addInstructionForWorker(instructions, worker, current, null,
            new InstructionJAL(registers.registerZero, exe.getLabel(), 1));

        // Add the reaction-invoking EXE to the schedule.
        addInstructionForWorker(instructions, current.getWorker(), current, null, exe);

        // Add the post-connection helper to the schedule, in case this reaction
        // is triggered by an input port, which is connected to a connection
        // buffer.
        // Reaction invocations can be skipped,
        // and we don't want the connection management to be skipped.
        // FIXME: This does not seem to support the case when an input port
        // triggers multiple reactions. We only want to add a post connection
        // helper after the last reaction triggered by this port.
        int indexToInsert = currentSchedule.indexOf(exe) + 1;
        generatePostConnectionHelpers(reaction, instructions, worker, indexToInsert, exe.getDagNode());
        
        // Add this reaction invoking EXE to the output-port-to-EXE map,
        // so that we know when to insert pre-connection helpers.
        for (TriggerInstance effect : reaction.effects) {
          if (effect instanceof PortInstance output) {
            portToUnhandledReactionExeMap.put(output, exe);
          }
        }

        // Increment the counter of the worker.
        // IMPORTANT: This ADDI has to be last because executing it releases
        // downstream workers. If this ADDI is executed before
        // connection management, then there is a race condition between
        // upstream pushing events into connection buffers and downstream
        // reading connection buffers.
        // Instantiate an ADDI to be executed after EXE, releasing the counting locks.
        var addi = new InstructionADDI(
                    registers.registerCounters.get(current.getWorker()),
                    registers.registerCounters.get(current.getWorker()),
                    1L);
        addInstructionForWorker(instructions, worker, current, null, addi);

      } else if (current.nodeType == dagNodeType.SYNC) {
        if (current == dagParitioned.tail) {
          // At this point, we know for sure that all reactors are done with
          // its current tag and are ready to advance time. We now insert a
          // connection helper after each port's last reaction's ADDI
          // (indicating the reaction is handled).
          for (var entry : portToUnhandledReactionExeMap.entrySet()) {
            PortInstance output = entry.getKey();
            // Only generate for delayed connections.
            if (outputToDelayedConnection(output)) {
              Instruction lastReactionExe = entry.getValue();
              int exeWorker = lastReactionExe.getWorker();
              int indexToInsert = instructions.get(exeWorker).indexOf(lastReactionExe) + 1;
              generatePreConnectionHelper(output, instructions, exeWorker, indexToInsert, lastReactionExe.getDagNode());
            }
          }
          portToUnhandledReactionExeMap.clear();

          // When the timeStep = TimeValue.MAX_VALUE in a SYNC node,
          // this means that the DAG is acyclic and can end without
          // real-time constraints, hence we do not genereate DU and ADDI.
          if (current.timeStep != TimeValue.MAX_VALUE) {
            for (int worker = 0; worker < workers; worker++) {
              // Add a DU instruction if the fast mode is off.
              // Turning on the dash mode does not affect this DU. The
              // hyperperiod is still real-time.
              // ALTERNATIVE DESIGN: remove the DU here and let the head node,
              // instead of the tail node, handle DU. This potentially allows
              // breaking the hyperperiod boundary.
              if (!targetConfig.get(FastProperty.INSTANCE))
                addInstructionForWorker(instructions, worker, current, null,
                  new InstructionDU(current.timeStep));
              // [Only Worker 0] Update the time increment register.
              if (worker == 0) {
                addInstructionForWorker(instructions, worker, current, null,
                  new InstructionADDI(
                    registers.registerOffsetInc,
                    registers.registerZero,
                    current.timeStep.toNanoSeconds()));
              }
              // Let all workers go to SYNC_BLOCK after finishing PREAMBLE.
              addInstructionForWorker(instructions, worker, current, null,
                new InstructionJAL(registers.registerReturnAddrs.get(worker), Phase.SYNC_BLOCK));
            }
          }
        }
      }
    }
    return new PretVmObjectFile(instructions, fragment, dagParitioned);
  }

  /**
   * Helper function for adding an instruction to a worker schedule.
   * This function is not meant to be called in the code generation logic above,
   * because a node needs to be associated with each instruction added.
   * 
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param inst The instruction to be added
   * @param index The index at which to insert the instruction
   */
  private void _addInstructionForWorker(
    List<List<Instruction>> instructions, int worker, Integer index, Instruction inst) {
    if (index == null) {
      // Add instruction to the instruction list.
      instructions.get(worker).add(inst);
    } else {
      // Insert instruction to the instruction list at the specified index.
      instructions.get(worker).add(index, inst);
    }
    // Remember the worker at the instruction level.
    inst.setWorker(worker);
  }

  /**
   * Helper function for adding an instruction to a worker schedule
   * 
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param node The DAG node for which this instruction is added
   * @param inst The instruction to be added
   */
  private void addInstructionForWorker(
    List<List<Instruction>> instructions, int worker, DagNode node, Integer index, Instruction inst) {
    // Add an instruction to the instruction list.
    _addInstructionForWorker(instructions, worker, index, inst);
    // Store the reference to the instruction in the DAG node.
    node.addInstruction(inst);
    // Store the reference to the DAG node in the instruction.
    inst.setDagNode(node);
  }

  /**
   * Helper function for adding an instruction to a worker schedule
   * 
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param nodes The DAG nodes for which this instruction is added
   * @param inst The instruction to be added
   */
  private void addInstructionForWorker(
    List<List<Instruction>> instructions, int worker, List<DagNode> nodes, Integer index, Instruction inst) {
    // Add an instruction to the instruction list.
    _addInstructionForWorker(instructions, worker, index, inst);
    for (DagNode node : nodes) {
      // Store the reference to the instruction in the DAG node.
      node.addInstruction(inst);
      // Store the reference to the DAG node in the instruction.
      inst.setDagNode(node);
    }
  }

  /** Generate C code from the instructions list. */
  public void generateCode(PretVmExecutable executable) {
    List<List<Instruction>> instructions = executable.getContent();

    // Instantiate a code builder.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("static_schedule.c");
    CodeBuilder code = new CodeBuilder();

    // Generate a block comment.
    code.pr(
        String.join(
            "\n",
            "/**",
            " * An auto-generated schedule file for the STATIC scheduler.",
            " * ",
            " * reactor array:",
            " * " + this.reactors,
            " * ",
            " * reaction array:",
            " * " + this.reactions,
            " */"));

    // Header files
    code.pr(
        String.join(
            "\n",
            "#include <stdint.h>",
            "#include <stddef.h> // size_t",
            "#include <limits.h> // ULLONG_MAX",
            "#include \"core/environment.h\"",
            "#include \"core/threaded/scheduler_instance.h\"",
            "#include \"core/threaded/scheduler_static_functions.h\"",
            "#include " + "\"" + fileConfig.name + ".h" + "\""));

    // Include reactor header files.
    List<TypeParameterizedReactor> tprs = this.reactors.stream().map(it -> it.tpr).toList();
    Set<String> headerNames = CUtil.getNames(tprs);
    for (var name : headerNames) {
      code.pr("#include " + "\"" + name + ".h" + "\"");
    }

    // Generate label macros.
    // Future FIXME: Make sure that label strings are formatted properly and are
    // unique, when the user is allowed to define custom labels. Currently,
    // all Phase enums are formatted properly.
    for (int i = 0; i < instructions.size(); i++) {
      var schedule = instructions.get(i);
      for (int j = 0; j < schedule.size(); j++) {
        if (schedule.get(j).hasLabel()) {
          code.pr("#define " + getWorkerLabelString(schedule.get(j).getLabel(), i) + " " + j);
        }
      }
    }
    code.pr("#define " + getPlaceHolderMacroString() + " " + "NULL");

    // Extern variables
    code.pr("// Extern variables");
    code.pr("extern environment_t envs[_num_enclaves];");
    code.pr("extern instant_t " + getVarName(registers.registerStartTime, false) + ";");

    // Runtime variables
    code.pr("// Runtime variables");
    if (targetConfig.isSet(TimeOutProperty.INSTANCE))
      // FIXME: Why is timeout volatile?
      code.pr(
          "volatile uint64_t "
              + getVarName(registers.registerTimeout, false)
              + " = "
              + targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()
              + "LL"
              + ";");
    code.pr("const size_t num_counters = " + workers + ";"); // FIXME: Seems unnecessary.
    code.pr("volatile reg_t " + getVarName(registers.registerOffset, false) + " = 0ULL;");
    code.pr("volatile reg_t " + getVarName(registers.registerOffsetInc, false) + " = 0ULL;");
    code.pr("const uint64_t " + getVarName(registers.registerZero, false) + " = 0ULL;");
    code.pr("const uint64_t " + getVarName(registers.registerOne, false) + " = 1ULL;");
    code.pr(
      "volatile uint64_t "
        + getVarName(GlobalVarType.WORKER_COUNTER)
        + "[" + workers + "]"
        + " = {0ULL};"); // Must be uint64_t, otherwise writing a long long to it could cause
    // buffer overflow.
    code.pr(
      "volatile reg_t "
        + getVarName(GlobalVarType.WORKER_RETURN_ADDR)
        + "[" + workers + "]"
        + " = {0ULL};");
    code.pr(
      "volatile reg_t " 
        + getVarName(GlobalVarType.WORKER_BINARY_SEMA)
        + "[" + workers + "]"
        + " = {0ULL};");

    // Generate function prototypes (forward declaration).
    // FIXME: Factor it out.
    for (ReactorInstance reactor : this.reactors) {
      for (PortInstance output : reactor.outputs) {
        // For each output port, iterate over each destination port.
        for (SendRange srcRange : output.getDependentPorts()) {
          for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
            // Can be used to identify a connection.
            PortInstance input = dstRange.instance;
            // Only generate pre-connection helper if it is delayed.
            if (outputToDelayedConnection(output)) {
              code.pr("void " + preConnectionHelperFunctionNameMap.get(input) + "();");
            }
            code.pr("void " + postConnectionHelperFunctionNameMap.get(input) + "();");
          }
        }
      }
    }

    // Generate static schedules. Iterate over the workers (i.e., the size
    // of the instruction list).
    for (int worker = 0; worker < instructions.size(); worker++) {
      var schedule = instructions.get(worker);
      code.pr("inst_t schedule_" + worker + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);

        // If there is a label attached to the instruction, generate a comment.
        if (inst.hasLabel()) code.pr("// " + getWorkerLabelString(inst.getLabel(), worker) + ":");

        // Generate code based on opcode
        switch (inst.getOpcode()) {
          case ADD:
            {
              InstructionADD add = (InstructionADD) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + add.getOpcode()
                      + ", "
                      + ".opcode="
                      + add.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(add.target, true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarName(add.source, true)
                      + ", "
                      + ".op3.reg="
                      + "(reg_t*)"
                      + getVarName(add.source2, true)
                      + "}"
                      + ",");
              break;
            }
          case ADDI:
            {
              InstructionADDI addi = (InstructionADDI) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + addi.getOpcode()
                      + ", "
                      + ".opcode="
                      + addi.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(addi.target, true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarName(addi.source, true)
                      + ", "
                      + ".op3.imm="
                      + addi.immediate
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case ADV:
            {
              ReactorInstance reactor = ((InstructionADV) inst).reactor;
              Register baseTime = ((InstructionADV) inst).baseTime;
              Register increment = ((InstructionADV) inst).increment;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.imm="
                      + reactors.indexOf(reactor)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarName(baseTime, true)
                      + ", "
                      + ".op3.reg="
                      + "(reg_t*)"
                      + getVarName(increment, true)
                      + "}"
                      + ",");
              break;
            }
          case ADVI:
            {
              Register baseTime = ((InstructionADVI) inst).baseTime;
              Long increment = ((InstructionADVI) inst).increment;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getPlaceHolderMacroString()
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarName(baseTime, true)
                      + ", "
                      + ".op3.imm="
                      + increment
                      + "LL" // FIXME: Why longlong should be ULL for our type?
                      + "}"
                      + ",");
              break;
            }
          case BEQ:
            {
              InstructionBEQ instBEQ = (InstructionBEQ) inst;
              String rs1Str = getVarName(instBEQ.rs1, true);
              String rs2Str = getVarName(instBEQ.rs2, true);
              Object label = instBEQ.label;
              String labelString = getWorkerLabelString(label, worker);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + instBEQ);
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BGE:
            {
              InstructionBGE instBGE = (InstructionBGE) inst;
              String rs1Str = getVarName(instBGE.rs1, true);
              String rs2Str = getVarName(instBGE.rs2, true);
              Object label = instBGE.label;
              String labelString = getWorkerLabelString(label, worker);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " >= "
                      + rs2Str);
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BLT:
            {
              InstructionBLT instBLT = (InstructionBLT) inst;
              String rs1Str = getVarName(instBLT.rs1, true);
              String rs2Str = getVarName(instBLT.rs2, true);
              Object label = instBLT.label;
              String labelString = getWorkerLabelString(label, worker);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " < "
                      + rs2Str);
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BNE:
            {
              InstructionBNE instBNE = (InstructionBNE) inst;
              String rs1Str = getVarName(instBNE.rs1, true);
              String rs2Str = getVarName(instBNE.rs2, true);
              Object label = instBNE.label;
              String labelString = getWorkerLabelString(label, worker);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " != "
                      + rs2Str);
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case DU:
            {
              TimeValue releaseTime = ((InstructionDU) inst).releaseTime;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Delay Until the variable offset plus "
                      + releaseTime
                      + " is reached.");
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(registers.registerOffset, true)
                      + ", "
                      + ".op2.imm="
                      + releaseTime.toNanoSeconds()
                      + "LL" // FIXME: LL vs ULL. Since we are giving time in signed ints. Why not
                      // use signed int as our basic data type not, unsigned?
                      + "}"
                      + ",");
              break;
            }
          case EXE:
            {
              String functionPointer = ((InstructionEXE) inst).functionPointer;
              String functionArgumentPointer = ((InstructionEXE) inst).functionArgumentPointer;
              Integer reactionNumber = ((InstructionEXE) inst).reactionNumber;
              code.pr("// Line " + j + ": " + "Execute function " + functionPointer);
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + functionPointer
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + functionArgumentPointer
                      + ", "
                      + ".op3.imm="
                      + (reactionNumber == null ? "ULLONG_MAX" : reactionNumber)
                      + "}"
                      + ",");
              break;
            }
          case JAL:
            {
              Register retAddr = ((InstructionJAL) inst).retAddr;
              var targetLabel = ((InstructionJAL) inst).targetLabel;
              Integer offset = ((InstructionJAL) inst).offset;
              String targetFullLabel = getWorkerLabelString(targetLabel, worker);
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(retAddr, true)
                      + ", "
                      + ".op2.imm="
                      + targetFullLabel + (offset == null ? "" : " + " + offset)
                      + "}"
                      + ",");
              break;
            }
          case JALR:
            {
              Register destination = ((InstructionJALR) inst).destination;
              Register baseAddr = ((InstructionJALR) inst).baseAddr;
              Long immediate = ((InstructionJALR) inst).immediate;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(destination, true)
                      + ", "
                      + ".op2.reg=" // FIXME: This does not seem right op2 seems to be used as an
                      // immediate...
                      + "(reg_t*)"
                      + getVarName(baseAddr, true)
                      + ", "
                      + ".op3.imm="
                      + immediate
                      + "}"
                      + ",");
              break;
            }
          case STP:
            {
              code.pr("// Line " + j + ": " + "Stop the execution");
              code.pr(
                "{" + ".func="
                  + "execute_inst_" + inst.getOpcode()
                  + ", "
                  + ".opcode="
                  + inst.getOpcode() + "}" + ",");
              break;
            }
          case WLT:
            {
              Register register = ((InstructionWLT) inst).register;
              Long releaseValue = ((InstructionWLT) inst).releaseValue;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(register, true)
                      + ", "
                      + ".op2.imm="
                      + releaseValue
                      + "}"
                      + ",");
              break;
            }
          case WU:
            {
              Register register = ((InstructionWU) inst).register;
              Long releaseValue = ((InstructionWU) inst).releaseValue;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{" + ".func="
                      + "execute_inst_" + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarName(register, true)
                      + ", "
                      + ".op2.imm="
                      + releaseValue
                      + "}"
                      + ",");
              break;
            }
          default:
            throw new RuntimeException("UNREACHABLE: " + inst.getOpcode());
        }
      }

      code.unindent();
      code.pr("};");
    }

    // Generate an array to store the schedule pointers.
    code.pr("const inst_t* static_schedules[] = {");
    code.indent();
    for (int i = 0; i < instructions.size(); i++) {
      code.pr("schedule_" + i + ",");
    }
    code.unindent();
    code.pr("};");

    // A function for initializing the non-compile-time constants.
    // We do not iterate over all entries of placeholderMaps because some
    // instructions might have been optimized away at this point.
    code.pr("// Fill in placeholders in the schedule.");
    code.pr("void initialize_static_schedule() {");
    code.indent();
    for (int w = 0; w < this.workers; w++) {
      var placeholderMap = placeholderMaps.get(w);
      Set<Instruction> workerInstructionsWithLabels = 
        instructions.get(w).stream()
                            .filter(it -> it.hasLabel())
                            .collect(Collectors.toSet());
      for (Instruction inst : workerInstructionsWithLabels) {
        PretVmLabel label = inst.getLabel();
        String labelFull = getWorkerLabelString(label, w);
        List<String> values = placeholderMap.get(label);
        if (values == null) continue;
        for (int i = 0; i < values.size(); i++) {
          code.pr("schedule_" + w + "[" + labelFull + "]" + ".op" + (i+1) + ".reg = (reg_t*)" + values.get(i) + ";");
        }
      }
    }
    code.unindent();
    code.pr("}");

    // Generate and print the pqueue functions here.
    // FIXME: Factor it out.
    for (ReactorInstance reactor : this.reactors) {
      for (PortInstance output : reactor.outputs) {
        
        // For each output port, iterate over each destination port.
        for (SendRange srcRange : output.getDependentPorts()) {
          for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
            
            // Can be used to identify a connection.
            PortInstance input = dstRange.instance;
            // Pqueue index (> 0 if multicast)
            int pqueueLocalIndex = 0;  // Assuming no multicast yet.
            // Logical delay of the connection
            Long delay = ASTUtils.getDelay(srcRange.connection.getDelay());
            if (delay == null) delay = 0L;
            // pqueue_heads index
            int pqueueIndex = getPqueueIndex(input);

            // Only generate pre-connection helpers for delayed connections.
            if (outputToDelayedConnection(output)) {
              // FIXME: Factor this out.
              /* Connection Source Helper */

              code.pr("void " + preConnectionHelperFunctionNameMap.get(input) + "() {");
              code.indent();
              
              // Set up the self struct, output port, pqueue,
              // and the current time.
              code.pr(CUtil.selfType(reactor) + "*" + " self = " + "(" + CUtil.selfType(reactor) + "*" + ")" + getReactorFromEnv(main, reactor) + ";");
              code.pr(CGenerator.variableStructType(output) + " port = " + "self->_lf_" + output.getName() + ";");
              code.pr("circular_buffer *pq = (circular_buffer*)port.pqueues[" + pqueueLocalIndex + "];");
              code.pr("instant_t current_time = self->base.tag.time;");

              // If the output port has a value, push it into the priority queue.
              // FIXME: Create a token and wrap it inside an event.
              code.pr(String.join("\n",
                "// If the output port has a value, push it into the connection buffer.",
                "if (port.is_present) {",
                " event_t event;",
                " if (port.token != NULL) event.token = port.token;",
                " else event.token = (void*)port.value; // FIXME: Only works with int, bool, and any type that can directly be assigned to a void* variable.",
                " // if (port.token != NULL) lf_print(\"Port value = %d\", *((int*)port.token->value));",
                " // lf_print(\"current_time = %lld\", current_time);",
                " event.time = current_time + " + "NSEC(" + delay + "ULL);",
                " // lf_print(\"event.time = %lld\", event.time);",
                " cb_push_back(pq, &event);",
                " // lf_print(\"Inserted an event @ %lld.\", event.time);",
                "}"
              ));

              code.pr(updateTimeFieldsToCurrentQueueHead(input));

              code.unindent();
              code.pr("}");
            }

            // FIXME: Factor this out.
            /* Connection Sink Helper */

            code.pr("void " + postConnectionHelperFunctionNameMap.get(input) + "() {");
            code.indent();

            // Clear the is_present field of the output port.
            code.pr(CUtil.selfType(reactor) + "*" + " self = " + "(" + CUtil.selfType(reactor) + "*" + ")" + getReactorFromEnv(main, reactor) + ";");
            code.pr("self->_lf_" + output.getName() + ".is_present = false;");

            // Only perform the buffer management for delayed connections.
            if (inputFromDelayedConnection(input)) {
              // Set up the self struct, output port, pqueue,
              // and the current time.
              ReactorInstance inputParent = input.getParent();
              code.pr(CUtil.selfType(inputParent) + "*" + " input_parent = " + "(" + CUtil.selfType(inputParent) + "*" + ")" + getReactorFromEnv(main, inputParent) + ";");
              code.pr(CUtil.selfType(reactor) + "*" + " output_parent = " + "(" + CUtil.selfType(reactor) + "*" + ")" + getReactorFromEnv(main, reactor) + ";");
              code.pr(CGenerator.variableStructType(output) + " port = " + "output_parent->_lf_" + output.getName() + ";");
              code.pr("circular_buffer *pq = (circular_buffer*)port.pqueues[" + pqueueLocalIndex + "];");
              code.pr("instant_t current_time = input_parent->base.tag.time;");

              // If the current head matches the current reactor's time,
              // pop the head.
              code.pr(String.join("\n",
                "// If the current head matches the current reactor's time, pop the head.",
                "event_t* head = (event_t*) cb_peek(pq);",
                "if (head != NULL && head->time <= current_time) {",
                "    cb_remove_front(pq);",
                "    // _lf_done_using(head->token); // Done using the token and let it be recycled.",
                updateTimeFieldsToCurrentQueueHead(input),
                "}"
              ));
            }

            code.unindent();
            code.pr("}");
          }
        }
      }
    }

    // Print to file.
    try {
      code.writeToFile(file.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Update op1 of trigger-testing instructions (i.e., BEQ) to the time field of
   * the current head of the queue.
   */
  private String updateTimeFieldsToCurrentQueueHead(PortInstance input) {
    CodeBuilder code = new CodeBuilder();

    // By this point, line macros have been generated. Get them from
    // a map that maps an input port to a list of TEST_TRIGGER macros.
    List<Instruction> triggerTimeTests = triggerPresenceTestMap.get(input);

    // Peek and update the head.
    code.pr(String.join("\n",
      "event_t* peeked = cb_peek(pq);",
      getPqueueHeadFromEnv(main, input) + " = " + "peeked" + ";"
    ));

    // FIXME: Find a way to rewrite the following using the address of
    // pqueue_heads, which does not need to change.
    // Update: We still need to update the pointers because we are
    // storing the pointer to the time field in one of the pqueue_heads,
    // which still needs to be updated.
    code.pr("if (peeked != NULL) {");
    code.indent();
    code.pr("// lf_print(\"Updated pqueue_head.\");");
    for (var test : triggerTimeTests) {
      code.pr("schedule_" + test.getWorker() + "[" + getWorkerLabelString(test.getLabel(), test.getWorker()) + "]" + ".op1.reg" + " = " + "(reg_t*)" + "&" + getPqueueHeadFromEnv(main, input) + "->time;");
    }
    code.unindent();
    code.pr("}");
    // If the head of the pqueue is NULL, then set the op1s to a NULL pointer,
    // in order to prevent the effect of "dangling pointers", since head is
    // freed earlier. 
    code.pr("else {");
    code.indent();
    for (var test : triggerTimeTests) {
      code.pr("schedule_" + test.getWorker() + "[" + getWorkerLabelString(test.getLabel(), test.getWorker()) + "]" + ".op1.reg" + " = " + "(reg_t*)" + "NULL;");
    }
    code.unindent();
    code.pr("}");

    return code.toString();
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(GlobalVarType type) {
    switch (type) {
      case GLOBAL_TIMEOUT:
        return "timeout";
      case GLOBAL_OFFSET:
        return "time_offset";
      case GLOBAL_OFFSET_INC:
        return "offset_inc";
      case GLOBAL_ZERO:
        return "zero";
      case GLOBAL_ONE:
        return "one";
      case WORKER_COUNTER:
        return "counters";
      case WORKER_RETURN_ADDR:
        return "return_addr";
      case WORKER_BINARY_SEMA:
        return "binary_sema";
      case EXTERN_START_TIME:
        return "start_time";
      case PLACEHOLDER:
        return "PLACEHOLDER";
      default:
        throw new RuntimeException("UNREACHABLE!");
    }
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(Register register, boolean isPointer) {
    GlobalVarType type = register.type;
    Integer worker = register.owner;
    // Ignore & for PLACEHOLDER because it's not possible to take address of NULL.
    String prefix = (isPointer && type != GlobalVarType.PLACEHOLDER) ? "&" : "";
    if (type.isShared())
      return prefix + getVarName(type);
    else
      return prefix + getVarName(type) + "[" + worker + "]";
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(String variable, Integer worker, boolean isPointer) {
    // If this variable comes from the environment, use a placeholder.
    if (placeholderMaps.get(worker).values().contains(variable))
      return getPlaceHolderMacroString();
    // Otherwise, return the string.
    return variable;
  }

  /** Return a string of a label for a worker */
  private String getWorkerLabelString(Object label, int worker) {
    if ((label instanceof PretVmLabel) || (label instanceof Phase) || (label instanceof String))
      return "WORKER" + "_" + worker + "_" + label.toString();
    throw new RuntimeException("Unsupported label type. Received: " + label.getClass().getName() + " = " + label);
  }

  /** Pretty printing instructions */
  public void display(PretVmObjectFile objectFile) {
    List<List<Instruction>> instructions = objectFile.getContent();
    for (int i = 0; i < instructions.size(); i++) {
      List<Instruction> schedule = instructions.get(i);
      System.out.println("Worker " + i + ":");
      for (int j = 0; j < schedule.size(); j++) {
        System.out.println(schedule.get(j));
      }
    }
  }

  /**
   * Link multiple object files into a single executable (represented also in an object file class).
   * Instructions are also inserted based on transition guards between fragments. In addition,
   * PREAMBLE and EPILOGUE instructions are inserted here.
   */
  public PretVmExecutable link(List<PretVmObjectFile> pretvmObjectFiles, Path graphDir) {

    // Create empty schedules.
    List<List<Instruction>> schedules = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      schedules.add(new ArrayList<Instruction>());
    }

    // Start with the first object file, which must not have upstream fragments.
    PretVmObjectFile current = pretvmObjectFiles.get(0);
    DagNode firstDagHead = current.getDag().head;

    // Generate and append the PREAMBLE code.
    List<List<Instruction>> preamble = generatePreamble(firstDagHead);
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(preamble.get(i));
    }

    // Create a queue for storing unlinked object files.
    Queue<PretVmObjectFile> queue = new LinkedList<>();

    // Create a set for tracking state space fragments seen,
    // so that we don't process the same object file twice.
    Set<PretVmObjectFile> seen = new HashSet<>();

    // Add the current fragment to the queue.
    queue.add(current);

    // Iterate while there are still object files in the queue.
    while (queue.size() > 0) {

      // Dequeue an object file.
      current = queue.poll();

      // Get the downstream fragments.
      Set<StateSpaceFragment> downstreamFragments = current.getFragment().getDownstreams().keySet();

      // Obtain partial schedules from the current object file.
      List<List<Instruction>> partialSchedules = current.getContent();

      // Append guards for downstream transitions to the partial schedules.
      List<Instruction> defaultTransition = null;
      for (var dsFragment : downstreamFragments) {
        List<Instruction> transition = current.getFragment().getDownstreams().get(dsFragment);
        // Check if a transition is a default transition.
        if (StateSpaceUtils.isDefaultTransition(transition)) {
          defaultTransition = transition;
          continue;
        }
        // Add COPIES of guarded transitions to the partial schedules.
        // They have to be copies since otherwise labels created for different
        // workers will be added to the same instruction object, creating conflicts.
        for (int i = 0; i < workers; i++) {
          partialSchedules.get(i).addAll(
            replaceAbstractRegistersToConcreteRegisters(transition, i));
        }
      }
      // Make sure to have the default transition copies to be appended LAST,
      // since default transitions are taken when no other transitions are taken.
      if (defaultTransition != null) {
        for (int i = 0; i < workers; i++) {
          partialSchedules.get(i).addAll(
            replaceAbstractRegistersToConcreteRegisters(defaultTransition, i));
        }
      }

      // Add a label to the first instruction using the exploration phase
      // (INIT, PERIODIC, SHUTDOWN_TIMEOUT, etc.).
      for (int i = 0; i < workers; i++) {
        partialSchedules.get(i).get(0).setLabel(current.getFragment().getPhase().toString());
      }

      // Add the partial schedules to the main schedule.
      for (int i = 0; i < workers; i++) {
        schedules.get(i).addAll(partialSchedules.get(i));
      }

      // Add current to the seen set.
      seen.add(current);

      // Get the object files associated with the downstream fragments.
      Set<PretVmObjectFile> downstreamObjectFiles =
          downstreamFragments.stream()
              .map(StateSpaceFragment::getObjectFile)
              // Filter out null object file since EPILOGUE has a null object file.
              .filter(it -> it != null)
              .collect(Collectors.toSet());

      // Remove object files that have been seen.
      downstreamObjectFiles.removeAll(seen);

      // Add object files related to the downstream fragments to the queue.
      queue.addAll(downstreamObjectFiles);
    }

    // Get a list of tail nodes. We can then attribute EPILOGUE and SyncBlock
    // instructions to these tail nodes. Note that this is an overapproximation
    // because some of these instructions will not actually get executed. For
    // example, the epilogue is only executed at the very end, so the periodic
    // fragment should not have to worry about it. But here we add it to these
    // tail nodes anyway because with the above link logic, it is unclear which
    // fragment is the actual last fragment in the execution.
    List<DagNode> dagTails = pretvmObjectFiles.stream().map(it -> it.getDag().tail).toList();

    // Generate the EPILOGUE code.
    List<List<Instruction>> epilogue = generateEpilogue(dagTails);
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(epilogue.get(i));
    }

    // Generate and append the synchronization block.
    List<List<Instruction>> syncBlock = generateSyncBlock(dagTails);
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(syncBlock.get(i));
    }

    // Generate DAGs with instructions.
    var dagList = pretvmObjectFiles.stream().map(it -> it.getDag()).toList();
    for (int i = 0; i < dagList.size(); i++) {
      // Generate another dot file with instructions displayed.
      Path file = graphDir.resolve("dag_partitioned_with_inst_" + i + ".dot");
      dagList.get(i).generateDotFile(file);
    }

    return new PretVmExecutable(schedules);
  }

  private List<Instruction> replaceAbstractRegistersToConcreteRegisters(List<Instruction> transitions, int worker) {
    List<Instruction> transitionCopy = transitions.stream().map(Instruction::clone).toList();
    for (Instruction inst : transitionCopy) {
      if (inst instanceof InstructionJAL jal
        && jal.retAddr == Register.ABSTRACT_WORKER_RETURN_ADDR) {
        jal.retAddr = registers.registerReturnAddrs.get(worker);
      }
    }
    return transitionCopy;
  }

  /** 
   * Generate the PREAMBLE code.
   *
   * @param node The node for which preamble code is generated
   */
  private List<List<Instruction>> generatePreamble(DagNode node) {

    List<List<Instruction>> schedules = new ArrayList<>();
    for (int worker = 0; worker < workers; worker++) {
      schedules.add(new ArrayList<Instruction>());
    }

    for (int worker = 0; worker < workers; worker++) {
      // [ONLY WORKER 0] Configure timeout register to be start_time + timeout.
      if (worker == 0) {
        // Configure offset register to be start_time.
        addInstructionForWorker(schedules, worker, node, null,
          new InstructionADDI(registers.registerOffset, registers.registerStartTime, 0L));
        // Configure timeout if needed.
        if (targetConfig.get(TimeOutProperty.INSTANCE) != null) {
          addInstructionForWorker(schedules, worker, node, null,
            new InstructionADDI(
              registers.registerTimeout,
              registers.registerStartTime,
              targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()));
        }
        // Update the time increment register.
        addInstructionForWorker(schedules, worker, node, null,
          new InstructionADDI(registers.registerOffsetInc, registers.registerZero, 0L));
      }
      // Let all workers go to SYNC_BLOCK after finishing PREAMBLE.
      addInstructionForWorker(schedules, worker, node, null, new InstructionJAL(registers.registerReturnAddrs.get(worker), Phase.SYNC_BLOCK));
      // Give the first PREAMBLE instruction to a PREAMBLE label.
      schedules.get(worker).get(0).setLabel(Phase.PREAMBLE.toString());
    }

    return schedules;
  }

  /** Generate the EPILOGUE code. */
  private List<List<Instruction>> generateEpilogue(List<DagNode> nodes) {

    List<List<Instruction>> schedules = new ArrayList<>();
    for (int worker = 0; worker < workers; worker++) {
      schedules.add(new ArrayList<Instruction>());
    }

    for (int worker = 0; worker < workers; worker++) {
      Instruction stp = new InstructionSTP();
      stp.setLabel(Phase.EPILOGUE.toString());
      addInstructionForWorker(schedules, worker, nodes, null, stp);
    }

    return schedules;
  }

  /** Generate the synchronization code block. */
  private List<List<Instruction>> generateSyncBlock(List<DagNode> nodes) {

    List<List<Instruction>> schedules = new ArrayList<>();

    for (int w = 0; w < workers; w++) {

      schedules.add(new ArrayList<Instruction>());

      // Worker 0 will be responsible for changing the global variables while
      // the other workers wait.
      if (w == 0) {

        // Wait for non-zero workers' binary semaphores to be set to 1.
        for (int worker = 1; worker < workers; worker++) {
          addInstructionForWorker(schedules, 0, nodes, null, new InstructionWU(registers.registerBinarySemas.get(worker), 1L));
        }

        // Update the global time offset by an increment (typically the hyperperiod).
        addInstructionForWorker(schedules, 0, nodes, null,
          new InstructionADD(
            registers.registerOffset,
            registers.registerOffset,
            registers.registerOffsetInc));

        // Reset all workers' counters.
        for (int worker = 0; worker < workers; worker++) {
          addInstructionForWorker(schedules, 0, nodes, null,
            new InstructionADDI(registers.registerCounters.get(worker), registers.registerZero, 0L));
        }

        // Advance all reactors' tags to offset + increment.
        for (int j = 0; j < this.reactors.size(); j++) {
          var reactor = this.reactors.get(j);
          var advi = new InstructionADVI(reactor, registers.registerOffset, 0L);
          advi.setLabel("ADVANCE_TAG_FOR_" + reactor.getFullNameWithJoiner("_") + "_" + generateShortUUID());
          placeholderMaps.get(0).put(
            advi.getLabel(),
            List.of(getReactorFromEnv(main, reactor)));
          addInstructionForWorker(schedules, 0, nodes, null, advi);
        }

        // Set non-zero workers' binary semaphores to be set to 0.
        for (int worker = 1; worker < workers; worker++) {
          addInstructionForWorker(schedules, 0, nodes, null,
            new InstructionADDI(
              registers.registerBinarySemas.get(worker),
              registers.registerZero,
              0L));
        }

        // Jump back to the return address.
        addInstructionForWorker(schedules, 0, nodes, null,
          new InstructionJALR(registers.registerZero, registers.registerReturnAddrs.get(0), 0L));

      } 
      // w >= 1
      else {

        // Set its own semaphore to be 1.
        addInstructionForWorker(schedules, w, nodes, null,
          new InstructionADDI(registers.registerBinarySemas.get(w), registers.registerZero, 1L));
        
        // Wait for the worker's own semaphore to be less than 1.
        addInstructionForWorker(schedules, w, nodes, null, new InstructionWLT(registers.registerBinarySemas.get(w), 1L));

        // Jump back to the return address.
        addInstructionForWorker(schedules, w, nodes, null,
          new InstructionJALR(registers.registerZero, registers.registerReturnAddrs.get(w), 0L));
      }

      // Give the first instruction to a SYNC_BLOCK label.
      schedules.get(w).get(0).setLabel(Phase.SYNC_BLOCK.toString());
    }

    return schedules;
  }

  /**
   * For a specific output port, generate an EXE instruction that puts tokens
   * into a priority queue buffer for that connection.
   * 
   * @param output The output port for which this connection helper is generated
   * @param workerSchedule To worker schedule to be updated
   * @param index The index where we insert the connection helper EXE
   */
  private void generatePreConnectionHelper(PortInstance output, List<List<Instruction>> instructions, int worker, int index, DagNode node) {
    // For each output port, iterate over each destination port.
    for (SendRange srcRange : output.getDependentPorts()) {
      for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
        // This input should uniquely identify a connection.
        // Check its position in the trigger array to get the pqueue index.
        PortInstance input = dstRange.instance;
        // Get the pqueue index from the index map.
        int pqueueIndex = getPqueueIndex(input);
        String sourceFunctionName = "process_connection_" + pqueueIndex + "_from_" + output.getFullNameWithJoiner("_") + "_to_" + input.getFullNameWithJoiner("_");
        // Update the connection helper function name map
        preConnectionHelperFunctionNameMap.put(input, sourceFunctionName);
        // Add the EXE instruction.
        var exe = new InstructionEXE(sourceFunctionName, "NULL", null);
        exe.setLabel("PROCESS_CONNECTION_" + pqueueIndex + "_FROM_" + output.getFullNameWithJoiner("_") + "_TO_" + input.getFullNameWithJoiner("_") + "_" + generateShortUUID());
        addInstructionForWorker(instructions, worker, node, index, exe);
      }
    }
  }

  private void generatePostConnectionHelpers(ReactionInstance reaction, List<List<Instruction>> instructions, int worker, int index, DagNode node) {
    for (TriggerInstance source : reaction.sources) {
      if (source instanceof PortInstance input) {
        // Get the pqueue index from the index map.
        int pqueueIndex = getPqueueIndex(input);
        String sinkFunctionName = "process_connection_" + pqueueIndex + "_after_" + input.getFullNameWithJoiner("_") + "_reads";
        // Update the connection helper function name map
        postConnectionHelperFunctionNameMap.put(input, sinkFunctionName);
        // Add the EXE instruction.
        var exe = new InstructionEXE(sinkFunctionName, "NULL", null);
        exe.setLabel("PROCESS_CONNECTION_" + pqueueIndex + "_AFTER_" + input.getFullNameWithJoiner("_") + "_" + "READS" + "_" + generateShortUUID());
        addInstructionForWorker(instructions, worker, node, index, exe);
      }
    }
  }

  /** Returns the placeholder macro string. */
  private String getPlaceHolderMacroString() {
    return "PLACEHOLDER";
  }

  /** Works similar as getPlaceHolderMacroString(), except this is used for
   * class constructors that must accept registers. */
  private Register getPlaceHolderMacroRegister() {
    return Register.PLACEHOLDER;
  }

  /** Generate short UUID to guarantee uniqueness in strings */
  private String generateShortUUID() {
    return UUID.randomUUID().toString().substring(0, 8); // take first 8 characters
  }

  private String getReactorFromEnv(ReactorInstance main, ReactorInstance reactor) {
    return CUtil.getEnvironmentStruct(main) + ".reactor_self_array" + "[" + this.reactors.indexOf(reactor) + "]";
  }

  private String getReactionFromEnv(ReactorInstance main, ReactionInstance reaction) {
    return CUtil.getEnvironmentStruct(main) + ".reaction_array" + "[" + this.reactions.indexOf(reaction) + "]";
  }

  private String getPqueueHeadFromEnv(ReactorInstance main, TriggerInstance trigger) {
    return CUtil.getEnvironmentStruct(main) + ".pqueue_heads" + "[" + getPqueueIndex(trigger) + "]";
  }

  private int getPqueueIndex(TriggerInstance trigger) {
    return this.triggers.indexOf(trigger);
  }

  private String getTriggerIsPresentFromEnv(ReactorInstance main, TriggerInstance trigger) {
    return "(" + "(" + nonUserFacingSelfType(trigger.getParent()) + "*)" + CUtil.getEnvironmentStruct(main) + ".reactor_self_array" + "[" + this.reactors.indexOf(trigger.getParent()) + "]" + ")" + "->" + "_lf_" + trigger.getName() + "->is_present";
  }

  private boolean outputToDelayedConnection(PortInstance output) {
    List<SendRange> dependentPorts = output.getDependentPorts(); // FIXME: Assume no broadcasts.
    if (dependentPorts.size() == 0) return false;
    Connection connection = dependentPorts.get(0).connection;
    Expression delayExpr = connection.getDelay();
    return delayExpr != null && ASTUtils.getDelay(delayExpr) > 0;
  }

  private boolean inputFromDelayedConnection(PortInstance input) {
    if (input.getDependsOnPorts().size() > 0) {
      PortInstance output = input.getDependsOnPorts().get(0).instance; // FIXME: Assume there is only one upstream port. This changes for multiports.
      return outputToDelayedConnection(output);
    } else {
      return false;
    }
  }

  /**
   * This mirrors userFacingSelfType(TypeParameterizedReactor tpr) in CReactorHeaderFileGenerator.java.
   */
  private String nonUserFacingSelfType(ReactorInstance reactor) {
    return "_" + reactor.getDefinition().getReactorClass().getName().toLowerCase() + "_self_t";
  }
}
