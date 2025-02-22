package org.lflang.pretvm;

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
import org.lflang.FileConfig;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Output;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagNode;
import org.lflang.pretvm.dag.JobNode;
import org.lflang.pretvm.dag.TimeNode;
import org.lflang.pretvm.instruction.*;
import org.lflang.pretvm.register.Register;
import org.lflang.pretvm.register.ReturnAddr;
import org.lflang.pretvm.register.RuntimeVar;
import org.lflang.pretvm.register.WorkerRegister;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.TimeOutProperty;

/**
 * A generator that generates PRET VM programs from DAGs. It also acts as a linker that piece
 * together multiple PRET VM object files.
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

  /** A list of port instances in the program */
  List<PortInstance> ports;

  /** Number of workers */
  int workers;

  /**
   * A nested map that maps a source port to a C function name, which updates a priority queue
   * holding tokens in a delayed connection. Each input can identify a unique connection because no
   * more than one connection can feed into an input port.
   */
  private Map<PortInstance, String> preConnectionHelperFunctionNameMap = new HashMap<>();

  private Map<PortInstance, String> postConnectionHelperFunctionNameMap = new HashMap<>();

  /**
   * A map that maps a trigger to a list of (BEQ) instructions where this trigger's presence is
   * tested.
   */
  private Map<TriggerInstance, List<Instruction>> triggerPresenceTestMap = new HashMap<>();

  /** PretVM registers */
  private Registers registers;

  /** Constructor */
  public InstructionGenerator(
      FileConfig fileConfig,
      TargetConfig targetConfig,
      int workers,
      ReactorInstance main,
      List<ReactorInstance> reactors,
      List<ReactionInstance> reactions,
      List<PortInstance> ports,
      Registers registers) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.workers = workers;
    this.main = main;
    this.reactors = reactors;
    this.reactions = reactions;
    this.ports = ports;
    this.registers = registers;
  }

  /** Topologically sort the dag nodes and assign release values to DAG nodes for counting locks. */
  public void assignReleaseIndices(Dag dagParitioned) {
    // Initialize a reaction index array to keep track of the latest counting
    // lock value for each worker.
    Long[] releaseValues = new Long[workers];
    Arrays.fill(releaseValues, 0L); // Initialize all elements to 0

    // Iterate over a topologically sorted list of dag nodes.
    for (DagNode current : dagParitioned.getTopologicalSort()) {
      if (current instanceof JobNode currentJob) {
        releaseValues[currentJob.getWorker()] += 1;
        currentJob.setReleaseIndex(releaseValues[currentJob.getWorker()]);
      }
    }
  }

  /** Traverse the DAG from head to tail using Khan's algorithm (topological sort). */
  public List<List<Instruction>> generateInstructions(
      Dag dagParitioned, PartialSchedule partialSchedule) {
    // Map from a reactor to its latest associated SYNC node.
    // Use case 1: This is used to determine when ADVIs and DUs should be generated without
    // duplicating them for each reaction node in the same reactor.
    // Use case 2: Determine a relative time increment for ADVIs.
    Map<ReactorInstance, TimeNode> reactorToLastSeenSyncNodeMap = new HashMap<>();

    // Map an output port to its last seen EXE instruction at the current
    // tag. When we know for sure that no other reactions can modify a port, we then
    // go back to the last seen reaction-invoking EXE that can modify this port and
    // _insert_ a connection helper right after the last seen EXE in the schedule.
    // All the key value pairs in this map are waiting to be handled,
    // since all the output port values must be written to the buffers at the
    // end of the tag.
    Map<PortInstance, EXE> portToUnhandledReactionExeMap = new HashMap<>();

    // Map a reaction to its last seen invocation, which is a DagNode.
    // If two invocations are mapped to different workers, a WU needs to
    // be generated to prevent race condition.
    // This map is used to check whether the WU needs to be generated.
    Map<ReactionInstance, JobNode> reactionToLastSeenInvocationMap = new HashMap<>();

    // Assign release values for the reaction nodes.
    assignReleaseIndices(dagParitioned);

    // Instructions for all workers
    List<List<Instruction>> instructions = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }

    // Iterate over a topologically sorted list of dag nodes.
    for (DagNode current : dagParitioned.getTopologicalSort()) {
      // Get the upstream reaction nodes.
      List<JobNode> upstreamReactionNodes =
          dagParitioned.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n instanceof JobNode)
              .map(n -> (JobNode) n)
              .toList();

      if (current instanceof JobNode currentJob) {
        // Find the worker assigned to the REACTION node,
        // the reactor, and the reaction.
        int worker = currentJob.getWorker();
        ReactionInstance reaction = currentJob.getReaction();
        ReactorInstance reactor = reaction.getParent();

        // Current worker schedule
        List<Instruction> currentSchedule = instructions.get(worker);

        // Get the nearest upstream sync node.
        TimeNode associatedSyncNode = currentJob.getAssociatedSyncNode();

        // WU Case 1:
        // If the reaction depends on upstream reactions owned by other
        // workers, generate WU instructions to resolve the dependencies.
        // FIXME: The current implementation generates multiple unnecessary WUs
        // for simplicity. How to only generate WU when necessary?
        for (JobNode n : upstreamReactionNodes) {
          int upstreamOwner = n.getWorker();
          if (upstreamOwner != worker) {
            addInstructionForWorker(
                instructions,
                currentJob.getWorker(),
                currentJob,
                null,
                new WU(registers.progressIndices.get(upstreamOwner), n.getReleaseIndex()));
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
        JobNode lastSeen = reactionToLastSeenInvocationMap.get(reaction);
        if (lastSeen != null && lastSeen.getWorker() != currentJob.getWorker()) {
          addInstructionForWorker(
              instructions,
              currentJob.getWorker(),
              currentJob,
              null,
              new WU(
                  registers.progressIndices.get(lastSeen.getWorker()), lastSeen.getReleaseIndex()));
          if (currentJob
              .getAssociatedSyncNode()
              .getTime()
              .isEarlierThan(lastSeen.getAssociatedSyncNode().getTime())) {
            System.out.println(
                "FATAL ERROR: The current node is earlier than the lastSeen node. This case should"
                    + " not be possible and this strategy needs to be revised.");
            System.exit(1);
          }
        }
        reactionToLastSeenInvocationMap.put(reaction, currentJob);

        // WU Case 3:
        // If the node has an upstream dependency based on connection, but the
        // upstream is mapped to a different worker. Generate a WU.
        List<JobNode> upstreamsFromConnection = dagParitioned.waitUntilDependencies.get(current);
        if (upstreamsFromConnection != null && upstreamsFromConnection.size() > 0) {
          for (JobNode us : upstreamsFromConnection) {
            if (us.getWorker() != currentJob.getWorker()) {
              addInstructionForWorker(
                  instructions,
                  currentJob.getWorker(),
                  currentJob,
                  null,
                  new WU(registers.progressIndices.get(us.getWorker()), us.getReleaseIndex()));
            }
          }
        }

        // When the new associated sync node _differs_ from the last associated sync
        // node of the reactor, this means that the current node's reactor needs
        // to advance to a new tag (i.e., reaches a new timestamp).
        // The code should update the associated sync node
        // in the reactorToLastSeenSyncNodeMap map. And if
        // associatedSyncNode is not the head, generate time-advancement
        // instructions (abbreviated as ADVI) and DU.
        if (associatedSyncNode != reactorToLastSeenSyncNodeMap.get(reactor)) {
          // Before updating reactorToLastSeenSyncNodeMap,
          // compute a relative time increment to be used when generating an ADVI.
          long relativeTimeIncrement;
          if (reactorToLastSeenSyncNodeMap.get(reactor) != null) {
            relativeTimeIncrement =
                associatedSyncNode.getTime().toNanoSeconds()
                    - reactorToLastSeenSyncNodeMap.get(reactor).getTime().toNanoSeconds();
          } else {
            relativeTimeIncrement = associatedSyncNode.getTime().toNanoSeconds();
          }

          // Update the mapping.
          reactorToLastSeenSyncNodeMap.put(reactor, associatedSyncNode);

          // If the reaction depends on a single SYNC node,
          // advance to the LOGICAL time of the SYNC node first,
          // as well as delay until the PHYSICAL time indicated by the SYNC node.
          // Skip if it is the start node since this is done in the sync block.
          // FIXME: Here we have an implicit assumption "logical time is
          // physical time." We need to find a way to relax this assumption.
          // FIXME: One way to relax this is that "logical time is physical time
          // only when executing real-time reactions, otherwise fast mode for
          // non-real-time reactions."
          if (associatedSyncNode != dagParitioned.start) {

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
                EXE lastPortModifyingReactionExe = portToUnhandledReactionExeMap.get(output);
                if (lastPortModifyingReactionExe != null) {
                  int exeWorker = lastPortModifyingReactionExe.getWorker();
                  int indexToInsert =
                      indexOfByReference(instructions.get(exeWorker), lastPortModifyingReactionExe)
                          + 1;
                  generatePreConnectionHelper(
                      output,
                      instructions,
                      exeWorker,
                      indexToInsert,
                      lastPortModifyingReactionExe.getDagNode());
                  // Remove the entry since this port is handled.
                  portToUnhandledReactionExeMap.remove(output);
                }
              }
            }

            // Generate an ADVI instruction using a relative time increment.
            // (instead of absolute). Relative style of coding promotes code reuse.
            // FIXME: Factor out in a separate function.
            String reactorTime = getFromEnvReactorTimePointer(main, reactor);
            Register reactorTimeReg = registers.getRuntimeVar(reactorTime);
            var timeAdvInsts =
                generateTimeAdvancementInstructions(reactor, reactorTimeReg, relativeTimeIncrement);
            addInstructionSequenceForWorker(instructions, worker, current, null, timeAdvInsts);

            // Generate a DU using a relative time increment.
            // There are two cases for NOT generating a DU within a
            // hyperperiod: 1. if fast is on, 2. if dash is on and the parent
            // reactor is not realtime.
            // Generate a DU instruction if neither case holds.
            if (!(targetConfig.get(FastProperty.INSTANCE))) {
              // reactorTimeReg is already updated by time advancement instructions.
              // Just delay until its recently updated value.
              addInstructionForWorker(
                  instructions, worker, current, null, new DU(reactorTimeReg, 0L));
            }
          }
        }

        // Create an EXE instruction that invokes the reaction.
        String reactorPointer = getFromEnvReactorPointer(main, reactor);
        String reactorTimePointer = getFromEnvReactorTimePointer(main, reactor);
        String reactionPointer = getFromEnvReactionFunctionPointer(main, reaction);
        EXE exeReaction =
            new EXE(
                registers.getRuntimeVar(reactionPointer),
                registers.getRuntimeVar(reactorPointer),
                reaction.index);
        exeReaction.addLabel(
            new Label(
                "EXECUTE_" + reaction.getFullNameWithJoiner("_") + "_" + generateShortUUID()));

        ////////////////////////////////////////////////////////////////
        // Generate instructions for deadline handling.
        // The general scheme for deadline handling is:
        //
        // Line x-3: ADDI temp0_reg, tag.time, reaction_deadline
        // Line x-2: EXE  update_temp1_to_current_time() // temp1_reg := lf_time_physical()
        // Line x-1: BLT  temp0_reg, temp1_reg, x+1
        // Line x  : EXE  reaction_body_function
        // Line x+1: JAL  x+3 // Jump pass the deadline handler if reaction body is executed.
        // Line x+2: EXE  deadline_handler_function
        //
        // Here we need to create the ADDI, EXE, and BLT instructions involved.
        ////////////////////////////////////////////////////////////////
        // Declare a sequence of instructions related to invoking the
        // reaction body and handling deadline violations.
        List<Instruction> reactionInvokingSequence = new ArrayList<>();
        if (reaction.declaredDeadline != null) {
          // Create ADDI for storing the physical time after which the
          // deadline is considered violated,
          // basically, current tag + deadline value.
          Instruction addiDeadlineTime =
              new ADDI(
                  registers.temp0.get(worker),
                  registers.getRuntimeVar(reactorTimePointer),
                  reaction.declaredDeadline.maxDelay.toNanoSeconds());
          addiDeadlineTime.addLabel(
              new Label(
                  "CALCULATE_DEADLINE_VIOLATION_TIME_FOR_"
                      + reaction.getFullNameWithJoiner("_")
                      + "_"
                      + generateShortUUID()));

          // Create EXE for updating the time register.
          var exeUpdateTimeRegister =
              new EXE(
                  registers.getRuntimeVar("update_temp1_to_current_time"),
                  registers.temp1.get(worker),
                  null);

          // Create deadline handling EXE
          String deadlineHandlerPointer =
              getFromEnvReactionDeadlineHandlerFunctionPointer(main, reaction);
          Instruction exeDeadlineHandler =
              new EXE(
                  registers.getRuntimeVar(deadlineHandlerPointer),
                  registers.getRuntimeVar(reactorPointer),
                  reaction.index);
          exeDeadlineHandler.addLabel(
              new Label(
                  "HANDLE_DEADLINE_VIOLATION_OF_"
                      + reaction.getFullNameWithJoiner("_")
                      + "_"
                      + generateShortUUID()));

          // Create BLT for checking deadline violation.
          var bltDeadlineViolation =
              new BLT(
                  registers.temp0.get(worker),
                  registers.temp1.get(worker),
                  exeDeadlineHandler.getLabel());

          // Create JAL for jumping pass the deadline handler if the
          // deadline is not violated.
          var jalPassHandler = new JAL(registers.zero, exeDeadlineHandler.getLabel(), 1L);

          // Add the reaction-invoking EXE and deadline handling
          // instructions to the schedule in the right order.
          reactionInvokingSequence.add(addiDeadlineTime);
          reactionInvokingSequence.add(exeUpdateTimeRegister);
          reactionInvokingSequence.add(bltDeadlineViolation);
          reactionInvokingSequence.add(exeReaction);
          reactionInvokingSequence.add(jalPassHandler);
          reactionInvokingSequence.add(exeDeadlineHandler);
        } else {
          // If the reaction does not have a deadline, just add the EXE
          // running the reaction body.
          reactionInvokingSequence.add(exeReaction);
        }

        // It is important that the beginning and the end of the
        // sequence has labels, so that the trigger checking BEQ
        // instructions can jump to the right place.
        if (reactionInvokingSequence.get(0).getLabel() == null
            || reactionInvokingSequence.get(reactionInvokingSequence.size() - 1) == null) {
          throw new RuntimeException(
              "The reaction invoking instruction sequence either misses a label at the first"
                  + " instruction or at the last instruction, or both.");
        }

        // Create BEQ instructions for checking ports.
        // Check if the reaction has input port ports or not. If so,
        // we need guards implemented using BEQ.
        boolean hasGuards = false;
        for (var trigger : reaction.triggers) {
          if (trigger instanceof PortInstance port && port.isInput()) {
            hasGuards = true;
            Register reg1;
            Register reg2;
            // If connection has delay, check the connection buffer to see if
            // the earliest event matches the reactor's current logical time.
            if (inputFromDelayedConnection(port)) {
              String pqueueHeadTime = getFromEnvPqueueHeadTimePointer(main, port);
              reg1 = registers.getRuntimeVar(pqueueHeadTime); // RUNTIME_STRUCT
              reg2 = registers.getRuntimeVar(reactorTimePointer); // RUNTIME_STRUCT
            }
            // Otherwise, if the connection has zero delay, check for the presence of the
            // downstream port.
            else {
              String isPresentField =
                  "&" + getTriggerIsPresentFromEnv(main, trigger); // The is_present field
              reg1 = registers.getRuntimeVar(isPresentField); // RUNTIME_STRUCT
              reg2 = registers.one; // Checking if is_present == 1
            }
            Instruction reactionSequenceFront = reactionInvokingSequence.get(0);
            Instruction beq = new BEQ(reg1, reg2, reactionSequenceFront.getLabel());
            beq.addLabel(
                new Label(
                    "TEST_TRIGGER_" + port.getFullNameWithJoiner("_") + "_" + generateShortUUID()));
            addInstructionForWorker(instructions, currentJob.getWorker(), current, null, beq);
            // Update triggerPresenceTestMap.
            if (triggerPresenceTestMap.get(port) == null)
              triggerPresenceTestMap.put(port, new LinkedList<>());
            triggerPresenceTestMap.get(port).add(beq);
          }
        }

        // If none of the guards are activated, jump to one line after the
        // reaction-invoking instruction sequence.
        if (hasGuards)
          addInstructionForWorker(
              instructions,
              worker,
              current,
              null,
              new JAL(
                  registers.zero,
                  reactionInvokingSequence.get(reactionInvokingSequence.size() - 1).getLabel(),
                  1L));

        // Add the reaction-invoking sequence to the instructions.
        addInstructionSequenceForWorker(
            instructions, currentJob.getWorker(), current, null, reactionInvokingSequence);

        // Add the post-connection helper to the schedule, in case this reaction
        // is triggered by an input port, which is connected to a connection
        // buffer.
        // Reaction invocations can be skipped,
        // and we don't want the connection management to be skipped.
        // FIXME: This does not seem to support the case when an input port
        // ports multiple reactions. We only want to add a post connection
        // helper after the last reaction triggered by this port.
        int indexToInsert = indexOfByReference(currentSchedule, exeReaction) + 1;
        generatePostConnectionHelpers(
            reaction, instructions, worker, indexToInsert, exeReaction.getDagNode());

        // Add this reaction invoking EXE to the output-port-to-EXE map,
        // so that we know when to insert pre-connection helpers.
        for (TriggerInstance effect : reaction.effects) {
          if (effect instanceof PortInstance output) {
            portToUnhandledReactionExeMap.put(output, exeReaction);
          }
        }

        // Increment the progress index of the worker.
        // IMPORTANT: This ADDI has to be last because executing it releases
        // downstream workers. If this ADDI is executed before
        // connection management, then there is a race condition between
        // upstream pushing events into connection buffers and downstream
        // reading connection buffers.
        // Instantiate an ADDI to be executed after EXE, releasing the counting locks.
        var addi =
            new ADDI(
                registers.progressIndices.get(currentJob.getWorker()),
                registers.progressIndices.get(currentJob.getWorker()),
                1L);
        addInstructionForWorker(instructions, worker, current, null, addi);

      } else if (current instanceof TimeNode currentTime) {
        if (current == dagParitioned.end) {
          // At this point, we know for sure that all reactors are done with
          // its current tag and are ready to advance time. We now insert a
          // connection helper after each port's last reaction's ADDI
          // (indicating the reaction is handled).
          // FIXME: This _after_ is sus. Should be before!
          for (var entry : portToUnhandledReactionExeMap.entrySet()) {
            PortInstance output = entry.getKey();
            // Only generate for delayed connections.
            if (outputToDelayedConnection(output)) {
              Instruction lastReactionExe = entry.getValue();
              int exeWorker = lastReactionExe.getWorker();
              int indexToInsert =
                  indexOfByReference(instructions.get(exeWorker), lastReactionExe) + 1;
              generatePreConnectionHelper(
                  output, instructions, exeWorker, indexToInsert, lastReactionExe.getDagNode());
            }
          }
          portToUnhandledReactionExeMap.clear();

          // When the timeStep = TimeValue.MAX_VALUE in a SYNC node,
          // this means that the DAG is acyclic and can end without
          // real-time constraints, hence we do not genereate DU and ADDI.
          if (currentTime.getTime() != TimeValue.MAX_VALUE) {
            for (int worker = 0; worker < workers; worker++) {
              // [Only Worker 0] Update the time offset increment register.
              if (worker == 0) {
                addInstructionForWorker(
                    instructions,
                    worker,
                    current,
                    null,
                    new ADDI(
                        registers.offsetInc,
                        registers.zero,
                        currentTime.getTime().toNanoSeconds()));
              }
              // Let all workers go to SYNC_BLOCK after finishing PREAMBLE.
              addInstructionForWorker(
                  instructions,
                  worker,
                  current,
                  null,
                  new JAL(
                      registers.returnAddrs.get(worker),
                      Label.getExecutionPhaseLabel(ExecutionPhase.SYNC_BLOCK)));
              // Add a DU instruction if the fast mode is off.
              // Turning on the dash mode does not affect this DU. The
              // hyperperiod is still real-time.
              // ALTERNATIVE DESIGN: remove the DU here and let the start node,
              // instead of the end node, handle DU. This potentially allows
              // breaking the hyperperiod boundary.
              //
              // At this point, the global offset register has been
              // updated in SYNC_BLOCK.
              //
              // We want to place this DU after the SYNC_BLOCK so that
              // workers enters a new hyperperiod with almost zero lag.
              // If this DU is placed before, then the SYNC_BLOCK will
              // contribute the lag at the beginning of the hyperperiod.
              if (!targetConfig.get(FastProperty.INSTANCE))
                addInstructionForWorker(
                    instructions, worker, current, null, new DU(registers.offset, 0L));
            }
          }
        }
      }
    }
    // Add a label to the first instruction using the exploration phase
    // (INIT, PERIODIC, SHUTDOWN_TIMEOUT, etc.).
    for (int w = 0; w < workers; w++) {
      // First, check if there is any instruction generated.
      // A worker without any work assignment has an empty schedule.
      // In this case, generate a dummy instruction: adding zero to a
      // temp register.
      // Without this dummy instruction, currently there will be
      // compilation errors due to not having a place to put phase labels.
      if (instructions.get(w).size() == 0) {
        addInstructionForWorker(
            instructions,
            w,
            dagParitioned.end,
            null,
            new ADD(registers.temp0.get(w), registers.temp0.get(w), registers.zero));
      }
      // Then assign a label to the first instruction.
      instructions.get(w).get(0).addLabel(Label.getExecutionPhaseLabel(partialSchedule.getPhase()));
    }

    return instructions;
  }

  /**
   * Generate a sequence of instructions for advancing a reactor's logical time. First, the
   * reactor's local time register needs to be incremented. Then, the `is_present` fields of the
   * reactor's output ports need to be set to false. This function replaces two previously
   * specialized instructions: ADV & ADVI.
   *
   * <p>This function is designed to have the same signature as ADV and ADVI.
   *
   * @param reactor The reactor instance to advance time and clear output ports for
   * @param baseTimeReg The base time this reactor should advance to (either the reactor's current
   *     time register, or the time offset for the next hyperperiod)
   * @param relativeTimeIncrement The time increment added on top of baseTimeReg
   * @return A list of instructions for advancing reactor's local time
   */
  private List<Instruction> generateTimeAdvancementInstructions(
      ReactorInstance reactor, Register baseTimeReg, long relativeTimeIncrement) {
    List<Instruction> timeAdvInsts = new ArrayList<>();

    // Increment the reactor local time.
    String reactorTimePointer = getFromEnvReactorTimePointer(main, reactor);
    Register reactorTimeReg = registers.getRuntimeVar(reactorTimePointer);
    var addiIncrementTime = new ADDI(reactorTimeReg, baseTimeReg, relativeTimeIncrement);
    var uuid = generateShortUUID();
    addiIncrementTime.addLabel(
        new Label("ADVANCE_TAG_FOR_" + reactor.getFullNameWithJoiner("_") + "_" + uuid));
    timeAdvInsts.add(addiIncrementTime);

    // Reset the is_present fields of all output ports of this reactor.
    var outputs = ASTUtils.allOutputs(reactor.tpr.reactor());
    for (int i = 0; i < outputs.size(); i++) {
      Output output = outputs.get(i);
      String selfType = CUtil.selfType(reactor.tpr);
      String portName = output.getName();
      String isPresentPointer = getPortIsPresentFieldPointer(main, reactor, selfType, portName);
      Register portIsPresentReg = registers.getRuntimeVar(isPresentPointer);
      var addiResetIsPresent = new ADD(portIsPresentReg, registers.zero, registers.zero);
      timeAdvInsts.add(addiResetIsPresent);
    }
    return timeAdvInsts;
  }

  /**
   * Helper function for adding an instruction to a worker schedule. This function is not meant to
   * be called in the code generation logic above, because a node needs to be associated with each
   * instruction added.
   *
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param inst The instruction to be added
   * @param index The index at which to insert the instruction. If the index is null, append the
   *     instruction at the end. Otherwise, append it at the specific index.
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
   * @param index The index at which to insert the instruction. If the index is null, append the
   *     instruction at the end. Otherwise, append it at the specific index.
   * @param inst The instruction to be added
   */
  private void addInstructionForWorker(
      List<List<Instruction>> instructions,
      int worker,
      DagNode node,
      Integer index,
      Instruction inst) {
    // Add an instruction to the instruction list.
    _addInstructionForWorker(instructions, worker, index, inst);
    // Store the reference to the DAG node in the instruction.
    inst.addDagNode(node);
  }

  /**
   * Helper function for adding an instruction to a worker schedule
   *
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param nodes A list of DAG nodes for which this instruction is added
   * @param index The index at which to insert the instruction. If the index is null, append the
   *     instruction at the end. Otherwise, append it at the specific index.
   * @param inst The instruction to be added
   */
  private void addInstructionForWorker(
      List<List<Instruction>> instructions,
      int worker,
      List<DagNode> nodes,
      Integer index,
      Instruction inst) {
    // Add an instruction to the instruction list.
    _addInstructionForWorker(instructions, worker, index, inst);
    for (DagNode node : nodes) {
      // Store the reference to the DAG node in the instruction.
      inst.addDagNode(node);
    }
  }

  /**
   * Helper function for adding a sequence of instructions to a worker schedule
   *
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param node The DAG node for which this instruction is added
   * @param index The index at which to insert the instruction. If the index is null, append the
   *     instruction at the end. Otherwise, append it at the specific index.
   * @param instList The list of instructions to be added
   */
  private void addInstructionSequenceForWorker(
      List<List<Instruction>> instructions,
      int worker,
      DagNode node,
      Integer index,
      List<Instruction> instList) {
    // Add instructions to the instruction list.
    for (int i = 0; i < instList.size(); i++) {
      Instruction inst = instList.get(i);
      _addInstructionForWorker(instructions, worker, index, inst);
      // Store the reference to the DAG node in the instruction.
      inst.addDagNode(node);
    }
  }

  /**
   * Helper function for adding a sequence of instructions to a worker schedule
   *
   * @param instructions The instructions under generation for a particular phase
   * @param worker The worker who owns the instruction
   * @param nodes A list of DAG nodes for which this instruction is added
   * @param index The index at which to insert the instruction. If the index is null, append the
   *     instruction at the end. Otherwise, append it at the specific index.
   * @param instList The list of instructions to be added
   */
  private void addInstructionSequenceForWorker(
      List<List<Instruction>> instructions,
      int worker,
      List<DagNode> nodes,
      Integer index,
      List<Instruction> instList) {
    // Add instructions to the instruction list.
    for (int i = 0; i < instList.size(); i++) {
      Instruction inst = instList.get(i);
      _addInstructionForWorker(instructions, worker, index, inst);
      // Store the reference to the DAG node in the instruction.
      for (DagNode node : nodes) {
        // Store the reference to the DAG node in the instruction.
        inst.addDagNode(node);
      }
    }
  }

  /** Generate C code from the instructions list. */
  public void generateCode(List<List<Instruction>> instructions) {

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
    Set<String> headerNames = new HashSet<>();
    for (var tpr : tprs) headerNames.add(CUtil.getName(tpr));
    for (var name : headerNames) {
      code.pr("#include " + "\"" + name + ".h" + "\"");
    }

    // Generate label macros.
    for (int workerId = 0; workerId < instructions.size(); workerId++) {
      List<Instruction> schedule = instructions.get(workerId);
      for (int lineNumber = 0; lineNumber < schedule.size(); lineNumber++) {
        Instruction inst = schedule.get(lineNumber);
        // If the instruction already has a label, print it.
        if (inst.hasLabel()) {
          List<Label> labelList = inst.getLabels();
          for (Label label : labelList) {
            code.pr("#define " + getWorkerLabelString(label, workerId) + " " + lineNumber);
          }
        }
        // Otherwise, if any of the instruction's operands needs a label for
        // delayed instantiation, create a label.
        else {
          List<Object> operands = inst.getOperands();
          for (int k = 0; k < operands.size(); k++) {
            Object operand = operands.get(k);
            if (operandRequiresDelayedInstantiation(operand)) {
              Label label =
                  new Label("DELAY_INSTANTIATE_" + inst.getOpcode() + "_" + generateShortUUID());
              inst.addLabel(label);
              code.pr("#define " + getWorkerLabelString(label, workerId) + " " + lineNumber);
              break;
            }
          }
        }
      }
    }
    code.pr("#define " + getPlaceHolderMacroString() + " " + "NULL");

    // Extern variables
    code.pr("// Extern variables");
    code.pr("extern environment_t envs[_num_enclaves];");
    code.pr("extern instant_t " + getVarName(registers.startTime, false) + ";");

    // Runtime variables
    code.pr("// Runtime variables");
    if (targetConfig.isSet(TimeOutProperty.INSTANCE))
      // FIXME: Why is timeout volatile?
      code.pr(
          "volatile uint64_t "
              + getVarName(registers.timeout, false)
              + " = "
              + targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()
              + "LL"
              + ";");
    code.pr("const size_t num_progress_indices = " + workers + ";"); // FIXME: Seems unnecessary.
    code.pr("volatile reg_t " + getVarName(registers.offset, false) + " = 0ULL;");
    code.pr("volatile reg_t " + getVarName(registers.offsetInc, false) + " = 0ULL;");
    code.pr("const uint64_t " + getVarName(registers.zero, false) + " = 0ULL;");
    code.pr("const uint64_t " + getVarName(registers.one, false) + " = 1ULL;");
    code.pr(
        "volatile uint64_t "
            + getVarName(registers.progressIndices)
            + "["
            + workers
            + "]"
            + " = {0ULL};"); // Must be uint64_t, otherwise writing a long long to it could cause
    // buffer overflow.
    code.pr(
        "volatile reg_t " + getVarName(registers.returnAddrs) + "[" + workers + "]" + " = {0ULL};");
    code.pr(
        "volatile reg_t " + getVarName(registers.binarySemas) + "[" + workers + "]" + " = {0ULL};");
    code.pr("volatile reg_t " + getVarName(registers.temp0) + "[" + workers + "]" + " = {0ULL};");
    code.pr("volatile reg_t " + getVarName(registers.temp1) + "[" + workers + "]" + " = {0ULL};");

    // Generate function prototypes.
    generateFunctionPrototypesForConnections(code);
    generateFunctionPrototypeForTimeUpdate(code);

    // Generate static schedules. Iterate over the workers (i.e., the size
    // of the instruction list).
    for (int worker = 0; worker < instructions.size(); worker++) {
      var schedule = instructions.get(worker);
      code.pr("inst_t schedule_" + worker + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);

        // If there is a label attached to the instruction, generate a comment.
        if (inst.hasLabel()) {
          List<Label> labelList = inst.getLabels();
          for (Label label : labelList) {
            code.pr("// " + getWorkerLabelString(label, worker) + ":");
          }
        }

        // Generate code based on opcode
        switch (inst.getClass().getSimpleName()) {
          case "ADD":
            {
              ADD add = (ADD) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + add.getOpcode()
                      + ", "
                      + ".opcode="
                      + add.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(add.getOperand1(), true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(add.getOperand2(), true)
                      + ", "
                      + ".op3.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(add.getOperand3(), true)
                      + "}"
                      + ",");
              break;
            }
          case "ADDI":
            {
              ADDI addi = (ADDI) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + addi.getOpcode()
                      + ", "
                      + ".opcode="
                      + addi.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(addi.getOperand1(), true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(addi.getOperand2(), true)
                      + ", "
                      + ".op3.imm="
                      + addi.getOperand3()
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case "BEQ":
            {
              BEQ instBEQ = (BEQ) inst;
              String rs1Str = getVarNameOrPlaceholder(instBEQ.getOperand1(), true);
              String rs2Str = getVarNameOrPlaceholder(instBEQ.getOperand2(), true);
              Label label = instBEQ.getOperand3();
              String labelString = getWorkerLabelString(label, worker);
              code.pr("// Line " + j + ": " + instBEQ);
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
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
          case "BGE":
            {
              BGE instBGE = (BGE) inst;
              String rs1Str = getVarNameOrPlaceholder(instBGE.getOperand1(), true);
              String rs2Str = getVarNameOrPlaceholder(instBGE.getOperand2(), true);
              Label label = instBGE.getOperand3();
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
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
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
          case "BLT":
            {
              BLT instBLT = (BLT) inst;
              String rs1Str = getVarNameOrPlaceholder(instBLT.getOperand1(), true);
              String rs2Str = getVarNameOrPlaceholder(instBLT.getOperand2(), true);
              Label label = instBLT.getOperand3();
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
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
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
          case "BNE":
            {
              BNE instBNE = (BNE) inst;
              String rs1Str = getVarNameOrPlaceholder(instBNE.getOperand1(), true);
              String rs2Str = getVarNameOrPlaceholder(instBNE.getOperand2(), true);
              Label label = instBNE.getOperand3();
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
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
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
          case "DU":
            {
              Register offsetRegister = ((DU) inst).getOperand1();
              Long releaseTime = ((DU) inst).getOperand2();
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Delay Until the variable offset plus "
                      + releaseTime
                      + " is reached.");
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(offsetRegister, true)
                      + ", "
                      + ".op2.imm="
                      + releaseTime
                      + "LL" // FIXME: LL vs ULL. Since we are giving time in signed ints. Why not
                      // use signed int as our basic data type not, unsigned?
                      + "}"
                      + ",");
              break;
            }
          case "EXE":
            {
              // functionPointer and functionArgumentPointer are not directly
              // printed in the code because they are not compile-time constants.
              // Use a PLACEHOLDER instead for delayed instantiation.
              Register functionPointer = ((EXE) inst).getOperand1();
              Register functionArgumentPointer = ((EXE) inst).getOperand2();
              Integer reactionNumber = ((EXE) inst).getOperand3();
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Execute function "
                      + functionPointer
                      + " with argument "
                      + functionArgumentPointer);
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(functionPointer, true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(functionArgumentPointer, true)
                      + ", "
                      + ".op3.imm="
                      + (reactionNumber == null ? "ULLONG_MAX" : reactionNumber)
                      + "}"
                      + ",");
              break;
            }
          case "JAL":
            {
              Register retAddr = ((JAL) inst).getOperand1();
              var targetLabel = ((JAL) inst).getOperand2();
              Long offset = ((JAL) inst).getOperand3();
              String targetFullLabel = getWorkerLabelString(targetLabel, worker);
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(retAddr, true)
                      + ", "
                      + ".op2.imm="
                      + targetFullLabel
                      + ", "
                      + ".op3.imm="
                      + (offset == null ? "0" : offset)
                      + "}"
                      + ",");
              break;
            }
          case "JALR":
            {
              Register destination = ((JALR) inst).getOperand1();
              Register baseAddr = ((JALR) inst).getOperand2();
              Long offset = ((JALR) inst).getOperand3();
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(destination, true)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(baseAddr, true)
                      + ", "
                      + ".op3.imm="
                      + offset
                      + "}"
                      + ",");
              break;
            }
          case "STP":
            {
              code.pr("// Line " + j + ": " + "Stop the execution");
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + "}"
                      + ",");
              break;
            }
          case "WLT":
            {
              Register register = ((WLT) inst).getOperand1();
              Long releaseValue = ((WLT) inst).getOperand2();
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(register, true)
                      + ", "
                      + ".op2.imm="
                      + releaseValue
                      + "}"
                      + ",");
              break;
            }
          case "WU":
            {
              Register register = ((WU) inst).getOperand1();
              Long releaseValue = ((WU) inst).getOperand2();
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{"
                      + ".func="
                      + "execute_inst_"
                      + inst.getOpcode()
                      + ", "
                      + ".opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + getVarNameOrPlaceholder(register, true)
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

    //// Delayed instantiation of operands
    // A function for initializing the non-compile-time constants.
    code.pr("// Fill in placeholders in the schedule.");
    code.pr("void initialize_static_schedule() {");
    code.indent();
    for (int w = 0; w < this.workers; w++) {
      var workerInstructions = instructions.get(w);
      // Iterate over each instruction operand and generate a delay
      // instantiation for each operand that needs one.
      for (Instruction inst : workerInstructions) {
        List<Object> operands = inst.getOperands();
        for (int i = 0; i < operands.size(); i++) {
          Object operand = operands.get(i);

          // If an operand does not need delayed instantiation, skip it.
          if (!operandRequiresDelayedInstantiation(operand)) continue;

          // For each case, turn the operand into a string.
          String operandStr = null;
          if (operand instanceof Register reg && reg instanceof RuntimeVar) {
            operandStr = getVarName(reg, false);
          } else if (operand instanceof ReactorInstance reactor) {
            operandStr = getFromEnvReactorPointer(main, reactor);
          } else throw new RuntimeException("Unhandled operand type!");

          // Get instruction label.
          // Since we create additional DELAY_INSTANTIATE labels when we start printing
          // static_schedule.c, at this point, an instruction must have a label.
          // So we can skip checking for the existence of labels here.
          Label label = inst.getLabel();
          String labelFull = getWorkerLabelString(label, w);

          // Since we are dealing with runtime structs and reactor pointers in
          // delayed instantiation,
          // casting unconditionally to (reg_t*) should be okay because these
          // structs are pointers. We also don't need to prepend & because
          // this is taken care of when generating the operand string above.
          code.pr(
              "schedule_"
                  + w
                  + "["
                  + labelFull
                  + "]"
                  + ".op"
                  + (i + 1)
                  + ".reg = (reg_t*)"
                  + operandStr
                  + ";");
        }
      }
    }
    code.unindent();
    code.pr("}");

    // Generate connection helper function definitions.
    generateHelperFunctionForConnections(code);

    // Generate helper functions for updating time.
    generateHelperFunctionForTimeUpdate(code);

    // Print to file.
    try {
      code.writeToFile(file.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Generate function prototypes for connection helper functions.
   *
   * @param code The code builder to add code to
   */
  private void generateFunctionPrototypesForConnections(CodeBuilder code) {
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
  }

  /**
   * Generate connection helper function definitions.
   *
   * @param code The code builder to add code to
   */
  private void generateHelperFunctionForConnections(CodeBuilder code) {
    for (ReactorInstance reactor : this.reactors) {
      for (PortInstance output : reactor.outputs) {

        // For each output port, iterate over each destination port.
        for (SendRange srcRange : output.getDependentPorts()) {
          for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {

            // Can be used to identify a connection.
            PortInstance input = dstRange.instance;
            // Pqueue index (> 0 if multicast)
            int pqueueLocalIndex = 0; // Assuming no multicast yet.
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
              code.pr(
                  CUtil.selfType(reactor)
                      + "*"
                      + " self = "
                      + "("
                      + CUtil.selfType(reactor)
                      + "*"
                      + ")"
                      + getFromEnvReactorPointer(main, reactor)
                      + ";");
              code.pr(
                  CUtil.variableStructType(output)
                      + " port = "
                      + "self->_lf_"
                      + output.getName()
                      + ";");
              code.pr(
                  "circular_buffer *pq = (circular_buffer*)port.pqueues["
                      + pqueueLocalIndex
                      + "];");
              code.pr("instant_t current_time = self->base.tag.time;");

              // If the output port has a value, push it into the priority queue.
              // FIXME: Create a token and wrap it inside an event.
              code.pr(
                  String.join(
                      "\n",
                      "// If the output port has a value, push it into the connection buffer.",
                      "if (port.is_present) {",
                      " event_t event;",
                      " if (port.token != NULL) event.token = port.token;",
                      " else event.token = (lf_token_t *)(uintptr_t)port.value; // FIXME: Only"
                          + " works with int, bool, and any type that can directly be assigned to a"
                          + " void* variable.",
                      " // if (port.token != NULL) lf_print(\"Port value = %d\","
                          + " *((int*)port.token->value));",
                      " // lf_print(\"current_time = %lld\", current_time);",
                      " event.base.tag.time = current_time + " + "NSEC(" + delay + "ULL);",
                      " // lf_print(\"event.time = %lld\", event.time);",
                      " cb_push_back(pq, &event);",
                      " // lf_print(\"Inserted an event @ %lld.\", event.time);",
                      "}"));

              code.pr(updateTimeFieldsToCurrentQueueHead(input));

              code.unindent();
              code.pr("}");
            }

            // FIXME: Factor this out.
            /* Connection Sink Helper */

            code.pr("void " + postConnectionHelperFunctionNameMap.get(input) + "() {");
            code.indent();

            // Clear the is_present field of the output port.
            code.pr(
                CUtil.selfType(reactor)
                    + "*"
                    + " self = "
                    + "("
                    + CUtil.selfType(reactor)
                    + "*"
                    + ")"
                    + getFromEnvReactorPointer(main, reactor)
                    + ";");
            code.pr("self->_lf_" + output.getName() + ".is_present = false;");

            // Only perform the buffer management for delayed connections.
            if (inputFromDelayedConnection(input)) {
              // Set up the self struct, output port, pqueue,
              // and the current time.
              ReactorInstance inputParent = input.getParent();
              code.pr(
                  CUtil.selfType(inputParent)
                      + "*"
                      + " input_parent = "
                      + "("
                      + CUtil.selfType(inputParent)
                      + "*"
                      + ")"
                      + getFromEnvReactorPointer(main, inputParent)
                      + ";");
              code.pr(
                  CUtil.selfType(reactor)
                      + "*"
                      + " output_parent = "
                      + "("
                      + CUtil.selfType(reactor)
                      + "*"
                      + ")"
                      + getFromEnvReactorPointer(main, reactor)
                      + ";");
              code.pr(
                  CUtil.variableStructType(output)
                      + " port = "
                      + "output_parent->_lf_"
                      + output.getName()
                      + ";");
              code.pr(
                  "circular_buffer *pq = (circular_buffer*)port.pqueues["
                      + pqueueLocalIndex
                      + "];");
              code.pr("instant_t current_time = input_parent->base.tag.time;");

              // If the current head matches the current reactor's time,
              // pop the head.
              code.pr(
                  String.join(
                      "\n",
                      "// If the current head matches the current reactor's time, pop the head.",
                      "event_t* head = (event_t*) cb_peek(pq);",
                      "if (head != NULL && head->base.tag.time <= current_time) {",
                      "    cb_remove_front(pq);",
                      "    // _lf_done_using(head->token); // Done using the token and let it be"
                          + " recycled.",
                      updateTimeFieldsToCurrentQueueHead(input),
                      "}"));
            }

            code.unindent();
            code.pr("}");
          }
        }
      }
    }
  }

  /**
   * Generate a function prototype for the helper function that updates the temp1 register to the
   * current physical time.
   *
   * @param code The code builder to add code to
   */
  private void generateFunctionPrototypeForTimeUpdate(CodeBuilder code) {
    code.pr("void update_temp1_to_current_time(void* worker);");
  }

  /**
   * Generate a definition for the helper function that updates the temp1 register to the current
   * physical time.
   *
   * @param code The code builder to add code to
   */
  private void generateHelperFunctionForTimeUpdate(CodeBuilder code) {
    code.pr(
        String.join(
            "\n",
            "void update_temp1_to_current_time(void* worker_temp1) {",
            "    *((reg_t*)worker_temp1) = lf_time_physical();",
            "}"));
  }

  /**
   * An operand requires delayed instantiation if: 1. it is a RUNTIME_STRUCT register (i.e., fields
   * in the generated LF self structs), or 2. it is a reactor instance. These pointers are not
   * considered "compile-time constants", so the C compiler will complain.
   */
  private boolean operandRequiresDelayedInstantiation(Object operand) {
    if ((operand instanceof Register reg && reg instanceof RuntimeVar)
        || (operand instanceof ReactorInstance)) {
      return true;
    }
    return false;
  }

  /**
   * Update op1 of trigger-testing instructions (i.e., BEQ) to the time field of the current head of
   * the queue.
   */
  private String updateTimeFieldsToCurrentQueueHead(PortInstance input) {
    CodeBuilder code = new CodeBuilder();

    // By this point, line macros have been generated. Get them from
    // a map that maps an input port to a list of TEST_TRIGGER macros.
    List<Instruction> triggerTimeTests = triggerPresenceTestMap.get(input);

    // Peek and update the head.
    code.pr(
        String.join(
            "\n",
            "event_t* peeked = cb_peek(pq);",
            getFromEnvPqueueHead(main, input) + " = " + "peeked" + ";"));

    // FIXME: Find a way to rewrite the following using the address of
    // pqueue_heads, which does not need to change.
    // Update: We still need to update the pointers because we are
    // storing the pointer to the time field in one of the pqueue_heads,
    // which still needs to be updated.
    code.pr("if (peeked != NULL) {");
    code.indent();
    code.pr("// lf_print(\"Updated pqueue_head.\");");
    for (var test : triggerTimeTests) {
      code.pr(
          "schedule_"
              + test.getWorker()
              + "["
              + getWorkerLabelString(test.getLabel(), test.getWorker())
              + "]"
              + ".op1.reg"
              + " = "
              + "(reg_t*)"
              + getFromEnvPqueueHeadTimePointer(main, input)
              + ";");
    }
    code.unindent();
    code.pr("}");
    // If the head of the pqueue is NULL, then set the op1s to a NULL pointer,
    // in order to prevent the effect of "dangling pointers", since head is
    // freed earlier.
    code.pr("else {");
    code.indent();
    for (var test : triggerTimeTests) {
      code.pr(
          "schedule_"
              + test.getWorker()
              + "["
              + getWorkerLabelString(test.getLabel(), test.getWorker())
              + "]"
              + ".op1.reg"
              + " = "
              + "(reg_t*)"
              + "NULL;");
    }
    code.unindent();
    code.pr("}");

    return code.toString();
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(Register register) {
    switch (register.getClass().getSimpleName()) {
      case "BinarySema":
        return "binary_sema";
      case "ProgressIndex":
        return "progress_index";
      case "Offset":
        return "time_offset";
      case "OffsetInc":
        return "offset_inc";
      case "One":
        return "one";
      case "Placeholder":
        return "PLACEHOLDER";
      case "ReturnAddr":
        return "return_addr";
      case "StartTime":
        return "start_time";
      case "Temp0":
        return "temp0";
      case "Temp1":
        return "temp1";
      case "Timeout":
        return "timeout";
      case "Zero":
        return "zero";
      default:
        throw new RuntimeException("Unhandled register type: " + register);
    }
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(List<Register> registers) {
    return getVarName(registers.get(0)); // Use the first register as an identifier.
  }

  /**
   * Return a C variable name based on the variable type. IMPORTANT: ALWAYS use this function when
   * generating the static schedule in C, so that we let the function decide automatically whether
   * delayed instantiation is used based on the type of variable.
   */
  private String getVarNameOrPlaceholder(Register register, boolean isPointer) {
    if (operandRequiresDelayedInstantiation(register)) return getPlaceHolderMacroString();
    return getVarName(register, isPointer);
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(Register register, boolean isPointer) {
    // If it is a runtime variable, return pointer directly.
    if (register instanceof RuntimeVar rv) return rv.pointer;
    // Look up the type in getVarName(type).
    String prefix = (isPointer) ? "&" : "";
    if (register.isGlobal()) {
      return prefix + getVarName(register);
    } else {
      Integer worker = ((WorkerRegister) register).getOwner();
      return prefix + getVarName(register) + "[" + worker + "]";
    }
  }

  /** Return a string of a label for a worker */
  private String getWorkerLabelString(Label label, int worker) {
    return "WORKER" + "_" + worker + "_" + label.toString();
  }

  /**
   * Link multiple object files into a single executable (represented also in an object file class).
   * Instructions are also inserted based on transition guards between fragments. In addition,
   * PREAMBLE and EPILOGUE instructions are inserted here.
   *
   * <p>Very importantly, transition guards are added to the DAG start nodes of the downstream
   * fragments, because they are placed after the sync block and DU, so they should factor into the
   * startup overhead of the next hyperperiod. Locations marked by "STARTUP OVERHEAD REASONING" is
   * related to this.
   */
  public List<List<Instruction>> link(List<PartialSchedule> partialSchedules, Path graphDir) {

    // Create empty instruction lists.
    List<List<Instruction>> instructions = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }

    // Start with the first object file, which must not have upstream fragments.
    PartialSchedule current = partialSchedules.get(0);
    DagNode firstDagStart = current.getDag().start;

    // Generate and append the PREAMBLE code.
    List<List<Instruction>> preamble = generatePreamble(firstDagStart, current);
    for (int i = 0; i < instructions.size(); i++) {
      instructions.get(i).addAll(preamble.get(i));
    }

    // Create a queue for storing unlinked object files.
    Queue<PartialSchedule> queue = new LinkedList<>();

    // Create a set for tracking state space fragments seen,
    // so that we don't process the same object file twice.
    Set<PartialSchedule> seen = new HashSet<>();

    // Add the current fragment to the queue.
    queue.add(current);

    // Iterate while there are still object files in the queue.
    while (queue.size() > 0) {

      // Dequeue an object file.
      current = queue.poll();

      // Get the downstream fragments.
      List<PartialSchedule> downstreamPartialSchedules = current.getDownstreams();

      // Obtain partial schedules from the current object file.
      List<List<Instruction>> currentInstructions = current.getInstructions();

      // Declare placeholders for default transition and default
      // fragment. They need to be added last, after the other
      // transitions.
      List<Instruction> defaultTransition = null;
      PartialSchedule defaultDownstreamSchedule = null;
      // Append guards for downstream transitions to the partial schedules.
      for (PartialSchedule downstreamSchedule : downstreamPartialSchedules) {
        List<Instruction> abstractTransition = current.guardMap.get(downstreamSchedule);
        // Check if a transition is a default transition.
        // If so, save them for later.
        if (PartialSchedule.isDefaultTransition(abstractTransition)) {
          defaultTransition = abstractTransition;
          defaultDownstreamSchedule = downstreamSchedule;
          continue;
        }
        // Add COPIES of guarded transitions to the partial schedules.
        // They have to be copies since otherwise labels created for different
        // workers will be added to the same instruction object,
        // creating conflicts.
        for (int w = 0; w < workers; w++) {
          // Replace the abstract registers with concrete registers.
          List<Instruction> concreteTransition =
              replaceAbstractRegistersToConcreteRegisters(abstractTransition, w);
          //// STARTUP OVERHEAD REASONING
          // Since the transition logic is executed after the sync
          // block and the DU, we need to attribute them to the head
          // node of the next phase.
          DagNode dagStartNodeOfNextPhase = downstreamSchedule.getDag().start;
          // Add instructions for worker.
          addInstructionSequenceForWorker(
              currentInstructions, w, dagStartNodeOfNextPhase, null, concreteTransition);
        }
      }
      // Handling the default transition
      // Make sure to have the default transition copies to be appended LAST,
      // since default transitions are taken when no other transitions are taken.
      if (defaultTransition != null) {
        for (int w = 0; w < workers; w++) {
          List<Instruction> concreteTransition =
              replaceAbstractRegistersToConcreteRegisters(defaultTransition, w);
          //// STARTUP OVERHEAD REASONING
          // If the downstream fragment is EPILOGUE, which does not have
          // object files. Set the associated DAG node to null.
          DagNode dagStartNodeOfNextPhase;
          if (defaultDownstreamSchedule.getPhase() == ExecutionPhase.EPILOGUE) {
            dagStartNodeOfNextPhase = null;
          } else if (defaultDownstreamSchedule.getDag() != null) {
            dagStartNodeOfNextPhase = defaultDownstreamSchedule.getDag().start;
          } else {
            throw new RuntimeException(
                "A non-epilogue phase without DAG is found: "
                    + defaultDownstreamSchedule.getPhase());
          }
          addInstructionSequenceForWorker(
              currentInstructions, w, dagStartNodeOfNextPhase, null, concreteTransition);
        }
      }

      // Add the partial schedules to the main schedule.
      for (int i = 0; i < workers; i++) {
        instructions.get(i).addAll(currentInstructions.get(i));
      }

      // Add current to the seen set.
      seen.add(current);

      // Remove object files that have been seen.
      downstreamPartialSchedules.removeAll(seen);

      // Add object files related to the downstream fragments to the queue.
      queue.addAll(downstreamPartialSchedules);
    }

    // Get a list of end nodes. We can then attribute EPILOGUE and SyncBlock
    // instructions to these end nodes. Note that this is an overapproximation
    // because some of these instructions will not actually get executed. For
    // example, the epilogue is only executed at the very end, so the periodic
    // fragment should not have to worry about it. But here we add it to these
    // end nodes anyway because with the above link logic, it is unclear which
    // fragment is the actual last fragment in the execution.
    List<DagNode> dagEndNodes = partialSchedules.stream().map(it -> it.getDag().end).toList();

    // Generate the EPILOGUE code.
    List<List<Instruction>> epilogue = generateEpilogue(dagEndNodes);
    for (int i = 0; i < instructions.size(); i++) {
      instructions.get(i).addAll(epilogue.get(i));
    }

    // Generate and append the synchronization block.
    List<List<Instruction>> syncBlock = generateSyncBlock(dagEndNodes);
    for (int i = 0; i < instructions.size(); i++) {
      instructions.get(i).addAll(syncBlock.get(i));
    }

    // Generate DAGs with instructions.
    var dagList = partialSchedules.stream().map(it -> it.getDag()).toList();
    var instructionsList =
        partialSchedules.stream().map(it -> it.getInstructions()).toList(); // One list per phase.
    for (int i = 0; i < dagList.size(); i++) {
      // Generate another dot file with instructions displayed.
      Path file = graphDir.resolve("dag_partitioned_with_inst_" + i + ".dot");
      dagList.get(i).generateDotFile(file, instructionsList.get(i));
    }

    return instructions;
  }

  private List<Instruction> replaceAbstractRegistersToConcreteRegisters(
      List<Instruction> transitions, int worker) {
    List<Instruction> transitionCopy = transitions.stream().map(Instruction::clone).toList();
    for (Instruction inst : transitionCopy) {
      if (inst instanceof JAL jal && jal.getOperand1() == ReturnAddr.ABSTRACT_REGISTER) {
        jal.setOperand1(registers.returnAddrs.get(worker));
      }
    }
    return transitionCopy;
  }

  /**
   * Generate the PREAMBLE code.
   *
   * @param node The node for which preamble code is generated
   * @param initialPhaseObjectFile The object file for the initial phase. This can be either INIT or
   *     PERIODIC.
   */
  private List<List<Instruction>> generatePreamble(
      DagNode node, PartialSchedule initialPhasePartialSchedule) {

    List<List<Instruction>> schedules = new ArrayList<>();
    for (int worker = 0; worker < workers; worker++) {
      schedules.add(new ArrayList<Instruction>());
    }

    for (int worker = 0; worker < workers; worker++) {
      // [ONLY WORKER 0] Configure timeout register to be start_time + timeout.
      if (worker == 0) {
        // Configure offset register to be start_time.
        addInstructionForWorker(
            schedules, worker, node, null, new ADDI(registers.offset, registers.startTime, 0L));
        // Configure timeout if needed.
        if (targetConfig.get(TimeOutProperty.INSTANCE) != null) {
          addInstructionForWorker(
              schedules,
              worker,
              node,
              null,
              new ADDI(
                  registers.timeout,
                  registers.startTime,
                  targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()));
        }
        // Update the time increment register.
        addInstructionForWorker(
            schedules, worker, node, null, new ADDI(registers.offsetInc, registers.zero, 0L));
      }
      // Let all workers jump to SYNC_BLOCK after finishing PREAMBLE.
      addInstructionForWorker(
          schedules,
          worker,
          node,
          null,
          new JAL(
              registers.returnAddrs.get(worker),
              Label.getExecutionPhaseLabel(ExecutionPhase.SYNC_BLOCK)));
      // Let all workers jump to the first phase (INIT or PERIODIC) after synchronization.
      addInstructionForWorker(
          schedules,
          worker,
          node,
          null,
          new JAL(
              registers.zero,
              Label.getExecutionPhaseLabel(initialPhasePartialSchedule.getPhase())));
      // Give the first PREAMBLE instruction to a PREAMBLE label.
      schedules.get(worker).get(0).addLabel(Label.getExecutionPhaseLabel(ExecutionPhase.PREAMBLE));
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
      Instruction stp = new STP();
      stp.addLabel(Label.getExecutionPhaseLabel(ExecutionPhase.EPILOGUE));
      addInstructionForWorker(schedules, worker, nodes, null, stp);
    }

    return schedules;
  }

  /** Generate the synchronization code block. */
  private List<List<Instruction>> generateSyncBlock(List<DagNode> nodes) {
    List<List<Instruction>> syncBlock = new ArrayList<>();

    for (int w = 0; w < workers; w++) {

      syncBlock.add(new ArrayList<Instruction>());

      // Worker 0 will be responsible for changing the global variables while
      // the other workers wait.
      if (w == 0) {

        // Wait for non-zero workers' binary semaphores to be set to 1.
        for (int worker = 1; worker < workers; worker++) {
          addInstructionForWorker(
              syncBlock, 0, nodes, null, new WU(registers.binarySemas.get(worker), 1L));
        }

        // Update the global time offset by an increment (typically the hyperperiod).
        addInstructionForWorker(
            syncBlock,
            0,
            nodes,
            null,
            new ADD(registers.offset, registers.offset, registers.offsetInc));

        // Reset all workers' progressIndices.
        for (int worker = 0; worker < workers; worker++) {
          addInstructionForWorker(
              syncBlock,
              0,
              nodes,
              null,
              new ADDI(registers.progressIndices.get(worker), registers.zero, 0L));
        }

        // Advance all reactors' tags to offset + increment.
        for (int j = 0; j < this.reactors.size(); j++) {
          var reactor = this.reactors.get(j);
          var timeAdvInsts = generateTimeAdvancementInstructions(reactor, registers.offset, 0L);
          addInstructionSequenceForWorker(syncBlock, 0, nodes, null, timeAdvInsts);
        }

        // Set non-zero workers' binary semaphores to be set to 0.
        for (int worker = 1; worker < workers; worker++) {
          addInstructionForWorker(
              syncBlock,
              0,
              nodes,
              null,
              new ADDI(registers.binarySemas.get(worker), registers.zero, 0L));
        }

        // Jump back to the return address.
        addInstructionForWorker(
            syncBlock, 0, nodes, null, new JALR(registers.zero, registers.returnAddrs.get(0), 0L));

      }
      // w >= 1
      else {

        // Set its own semaphore to be 1.
        addInstructionForWorker(
            syncBlock, w, nodes, null, new ADDI(registers.binarySemas.get(w), registers.zero, 1L));

        // Wait for the worker's own semaphore to be less than 1.
        addInstructionForWorker(
            syncBlock, w, nodes, null, new WLT(registers.binarySemas.get(w), 1L));

        // Jump back to the return address.
        addInstructionForWorker(
            syncBlock, w, nodes, null, new JALR(registers.zero, registers.returnAddrs.get(w), 0L));
      }

      // Give the first instruction to a SYNC_BLOCK label.
      syncBlock.get(w).get(0).addLabel(Label.getExecutionPhaseLabel(ExecutionPhase.SYNC_BLOCK));
    }

    return syncBlock;
  }

  /**
   * For a specific output port, generate an EXE instruction that puts tokens into a priority queue
   * buffer for that connection.
   *
   * @param output The output port for which this connection helper is generated
   * @param workerSchedule To worker schedule to be updated
   * @param index The index where we insert the connection helper EXE
   */
  private void generatePreConnectionHelper(
      PortInstance output,
      List<List<Instruction>> instructions,
      int worker,
      int index,
      DagNode node) {
    // For each output port, iterate over each destination port.
    for (SendRange srcRange : output.getDependentPorts()) {
      for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
        // This input should uniquely identify a connection.
        // Check its position in the trigger array to get the pqueue index.
        PortInstance input = dstRange.instance;
        // Get the pqueue index from the index map.
        int pqueueIndex = getPqueueIndex(input);
        String sourceFunctionName =
            "process_connection_"
                + pqueueIndex
                + "_from_"
                + output.getFullNameWithJoiner("_")
                + "_to_"
                + input.getFullNameWithJoiner("_");
        // Update the connection helper function name map
        preConnectionHelperFunctionNameMap.put(input, sourceFunctionName);
        // Add the EXE instruction.
        var exe =
            new EXE(
                registers.getRuntimeVar(sourceFunctionName), registers.getRuntimeVar("NULL"), null);
        exe.addLabel(
            new Label(
                "PROCESS_CONNECTION_"
                    + pqueueIndex
                    + "_FROM_"
                    + output.getFullNameWithJoiner("_")
                    + "_TO_"
                    + input.getFullNameWithJoiner("_")
                    + "_"
                    + generateShortUUID()));
        addInstructionForWorker(instructions, worker, node, index, exe);
      }
    }
  }

  private void generatePostConnectionHelpers(
      ReactionInstance reaction,
      List<List<Instruction>> instructions,
      int worker,
      int index,
      DagNode node) {
    for (TriggerInstance source : reaction.sources) {
      if (source instanceof PortInstance input) {
        // Get the pqueue index from the index map.
        int pqueueIndex = getPqueueIndex(input);
        String sinkFunctionName =
            "process_connection_"
                + pqueueIndex
                + "_after_"
                + input.getFullNameWithJoiner("_")
                + "_reads";
        // Update the connection helper function name map
        postConnectionHelperFunctionNameMap.put(input, sinkFunctionName);
        // Add the EXE instruction.
        var exe =
            new EXE(
                registers.getRuntimeVar(sinkFunctionName), registers.getRuntimeVar("NULL"), null);
        exe.addLabel(
            new Label(
                "PROCESS_CONNECTION_"
                    + pqueueIndex
                    + "_AFTER_"
                    + input.getFullNameWithJoiner("_")
                    + "_"
                    + "READS"
                    + "_"
                    + generateShortUUID()));
        addInstructionForWorker(instructions, worker, node, index, exe);
      }
    }
  }

  /** Returns the placeholder macro string. */
  private String getPlaceHolderMacroString() {
    return "PLACEHOLDER";
  }

  /** Generate short UUID to guarantee uniqueness in strings */
  private String generateShortUUID() {
    return UUID.randomUUID().toString().substring(0, 8); // take first 8 characters
  }

  private String getFromEnvReactorPointer(ReactorInstance main, ReactorInstance reactor) {
    return CUtil.getEnvironmentStruct(main)
        + ".reactor_self_array"
        + "["
        + this.reactors.indexOf(reactor)
        + "]";
  }

  private String getFromEnvReactorTimePointer(ReactorInstance main, ReactorInstance reactor) {
    return "&"
        + getFromEnvReactorPointer(main, reactor)
        + "->tag.time"; // pointer to time at reactor
  }

  private String getFromEnvReactorOutputPortPointer(
      ReactorInstance main, ReactorInstance reactor, String reactorBaseType, String portName) {
    return "("
        + "("
        + reactorBaseType
        + "*)"
        + getFromEnvReactorPointer(main, reactor)
        + ")"
        + "->"
        + "_lf_"
        + portName;
  }

  private String getPortIsPresentFieldPointer(
      ReactorInstance main, ReactorInstance reactor, String reactorBaseType, String portName) {
    return "&"
        + "("
        + getFromEnvReactorOutputPortPointer(main, reactor, reactorBaseType, portName)
        + ".is_present"
        + ")";
  }

  private String getFromEnvReactionStruct(ReactorInstance main, ReactionInstance reaction) {
    return CUtil.getEnvironmentStruct(main)
        + ".reaction_array"
        + "["
        + this.reactions.indexOf(reaction)
        + "]";
  }

  private String getFromEnvReactionFunctionPointer(
      ReactorInstance main, ReactionInstance reaction) {
    return getFromEnvReactionStruct(main, reaction) + "->function";
  }

  private String getFromEnvReactionDeadlineHandlerFunctionPointer(
      ReactorInstance main, ReactionInstance reaction) {
    return getFromEnvReactionStruct(main, reaction) + "->deadline_violation_handler";
  }

  private String getFromEnvPqueueHead(ReactorInstance main, TriggerInstance trigger) {
    return CUtil.getEnvironmentStruct(main) + ".pqueue_heads" + "[" + getPqueueIndex(trigger) + "]";
  }

  private String getFromEnvPqueueHeadTimePointer(ReactorInstance main, TriggerInstance trigger) {
    return "&" + getFromEnvPqueueHead(main, trigger) + "->base.tag.time";
  }

  private int getPqueueIndex(TriggerInstance trigger) {
    return this.ports.indexOf(trigger);
  }

  private String getTriggerIsPresentFromEnv(ReactorInstance main, TriggerInstance trigger) {
    return "("
        + "("
        + nonUserFacingSelfType(trigger.getParent())
        + "*)"
        + CUtil.getEnvironmentStruct(main)
        + ".reactor_self_array"
        + "["
        + this.reactors.indexOf(trigger.getParent())
        + "]"
        + ")"
        + "->"
        + "_lf_"
        + trigger.getName()
        + "->is_present";
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
      PortInstance output =
          input
              .getDependsOnPorts()
              .get(0)
              .instance; // FIXME: Assume there is only one upstream port. This changes for
      // multiports.
      return outputToDelayedConnection(output);
    } else {
      return false;
    }
  }

  /**
   * This mirrors userFacingSelfType(TypeParameterizedReactor tpr) in
   * CReactorHeaderFileGenerator.java.
   */
  private String nonUserFacingSelfType(ReactorInstance reactor) {
    return "_" + reactor.getDefinition().getReactorClass().getName().toLowerCase() + "_self_t";
  }

  public static int indexOfByReference(List<?> list, Object o) {
    for (int i = 0; i < list.size(); i++) {
      if (list.get(i) == o) { // Compare references using '=='
        return i;
      }
    }
    return -1; // Return -1 if not found
  }
}
