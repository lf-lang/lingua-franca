/*************
 * Copyright (c) 2019-2023, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.analyses.pretvm.GlobalVarType;
import org.lflang.analyses.pretvm.Instruction;
import org.lflang.analyses.pretvm.InstructionBGE;
import org.lflang.analyses.pretvm.InstructionGenerator;
import org.lflang.analyses.pretvm.PretVmExecutable;
import org.lflang.analyses.pretvm.PretVmObjectFile;
import org.lflang.analyses.scheduler.EgsScheduler;
import org.lflang.analyses.scheduler.LoadBalancedScheduler;
import org.lflang.analyses.scheduler.MocasinScheduler;
import org.lflang.analyses.scheduler.StaticScheduler;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.analyses.statespace.StateSpaceFragment;
import org.lflang.analyses.statespace.StateSpaceUtils;
import org.lflang.analyses.statespace.Tag;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.StaticSchedulerType;

public class CStaticScheduleGenerator {

  /** File config */
  protected final CFileConfig fileConfig;

  /** Target configuration */
  protected TargetConfig targetConfig;

  /** Message reporter */
  protected MessageReporter messageReporter;

  /** Main reactor instance */
  protected ReactorInstance main;

  /** The number of workers to schedule for */
  protected int workers;

  /** A list of reactor instances */
  protected List<ReactorInstance> reactors;

  /** A list of reaction instances */
  protected List<ReactionInstance> reactions;

  /** A list of reaction triggers */
  protected List<TriggerInstance> triggers;

  /** A path for storing graph */
  protected Path graphDir;

  // Constructor
  public CStaticScheduleGenerator(
      CFileConfig fileConfig,
      TargetConfig targetConfig,
      MessageReporter messageReporter,
      ReactorInstance main,
      List<ReactorInstance> reactorInstances,
      List<ReactionInstance> reactionInstances,
      List<TriggerInstance> reactionTriggers) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.messageReporter = messageReporter;
    this.main = main;
    this.workers = targetConfig.get(WorkersProperty.INSTANCE);
    this.reactors = reactorInstances;
    this.reactions = reactionInstances;
    this.triggers = reactionTriggers;

    // Create a directory for storing graph.
    this.graphDir = fileConfig.getSrcGenPath().resolve("graphs");
    try {
      Files.createDirectories(this.graphDir);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Main function for generating a static schedule file in C.
  public void generate() {

    // Generate a list of state space fragments that captures
    // all the behavior of the LF program.
    List<StateSpaceFragment> fragments = generateStateSpaceFragments();

    // Create a DAG generator
    DagGenerator dagGenerator = new DagGenerator(this.fileConfig);

    // Create a scheduler.
    StaticScheduler scheduler = createStaticScheduler();

    // Determine the number of workers, if unspecified.
    if (this.workers == 0) {
      // Update the previous value of 0.
      this.workers = scheduler.setNumberOfWorkers();
      WorkersProperty.INSTANCE.update(targetConfig, this.workers);

      // Update CMAKE compile definitions.
      final var defs = new HashMap<String, String>();
      defs.put("NUMBER_OF_WORKERS", String.valueOf(targetConfig.get(WorkersProperty.INSTANCE)));
      CompileDefinitionsProperty.INSTANCE.update(targetConfig, defs);
    }

    // Create InstructionGenerator, which acts as a compiler and a linker.
    InstructionGenerator instGen =
        new InstructionGenerator(
            this.fileConfig, this.targetConfig, this.workers, this.main, this.reactors, this.reactions, this.triggers);

    // For each fragment, generate a DAG, perform DAG scheduling (mapping tasks
    // to workers), and generate instructions for each worker.
    List<PretVmObjectFile> pretvmObjectFiles = new ArrayList<>();
    for (var i = 0; i < fragments.size(); i++) {
      // Get the fragment.
      StateSpaceFragment fragment = fragments.get(i);

      // Generate a raw DAG from a state space fragment.
      Dag dag = dagGenerator.generateDag(fragment.getDiagram());

      // Generate a dot file.
      Path file = graphDir.resolve("dag_raw" + "_frag_" + i + ".dot");
      dag.generateDotFile(file);

      // Generate a partitioned DAG based on the number of workers.
      // FIXME: Bring the DOT generation calls to this level instead of hiding
      // them inside partitionDag().
      Dag dagPartitioned = scheduler.partitionDag(dag, i, this.workers, "_frag_" + i);

      // Do not execute the following step for the MOCASIN scheduler yet.
      // FIXME: A pass-based architecture would be better at managing this.
      if (!(targetConfig.get(SchedulerProperty.INSTANCE).staticScheduler()
              == StaticSchedulerType.StaticScheduler.MOCASIN
          && (targetConfig.get(SchedulerProperty.INSTANCE).mocasinMapping() == null
              || targetConfig.get(SchedulerProperty.INSTANCE).mocasinMapping().size() == 0))) {
        // Ensure the DAG is valid before proceeding to generating instructions.
        if (!dagPartitioned.isValidDAG())
          throw new RuntimeException("The generated DAG is invalid:" + " fragment " + i);
        // Generate instructions (wrapped in an object file) from DAG partitions.
        PretVmObjectFile objectFile = instGen.generateInstructions(dagPartitioned, fragment);
        // Point the fragment to the new object file.
        fragment.setObjectFile(objectFile);
        // Add the object file to list.
        pretvmObjectFiles.add(objectFile);
      }
    }

    // Do not execute the following step if the MOCASIN scheduler in used and
    // mappings are not provided.
    // FIXME: A pass-based architecture would be better at managing this.
    if (targetConfig.get(SchedulerProperty.INSTANCE).staticScheduler()
            == StaticSchedulerType.StaticScheduler.MOCASIN
        && (targetConfig.get(SchedulerProperty.INSTANCE).mocasinMapping() == null
            || targetConfig.get(SchedulerProperty.INSTANCE).mocasinMapping().size() == 0)) {
      messageReporter
          .nowhere()
          .info(
              "SDF3 files generated. Please invoke `mocasin` to generate mappings and provide paths"
                  + " to them using the `mocasin-mapping` target property under `scheduler`. A"
                  + " sample mocasin command is `mocasin pareto_front graph=sdf3_reader"
                  + " trace=sdf3_reader platform=odroid sdf3.file=<abs_path_to_xml>`");
      System.exit(0);
    }

    // Link multiple object files into a single executable (represented also in an object file
    // class).
    // Instructions are also inserted based on transition guards between fragments.
    // In addition, PREAMBLE and EPILOGUE instructions are inserted here.
    PretVmExecutable executable = instGen.link(pretvmObjectFiles, graphDir);

    // Generate C code.
    instGen.generateCode(executable);
  }

  /**
   * Generate a list of state space fragments for an LF program. This function calls
   * generateStateSpaceDiagram(<phase>) multiple times to capture the full behavior of the LF
   * program.
   */
  private List<StateSpaceFragment> generateStateSpaceFragments() {

    // Initialize variables
    StateSpaceExplorer explorer = new StateSpaceExplorer(targetConfig);
    List<StateSpaceFragment> fragments = new ArrayList<>();

    /***************/
    /* Async phase */
    /***************/
    // Generate a state space diagram for the initialization and periodic phase
    // of an LF program.
    List<StateSpaceDiagram> asyncDiagrams =
        StateSpaceUtils.generateAsyncStateSpaceDiagrams(explorer, Phase.ASYNC, main, new Tag(0,0,true), targetConfig, graphDir, "state_space_" + Phase.ASYNC);
    for (var diagram : asyncDiagrams)
      diagram.display();
    
    /**************************************/
    /* Initialization and Periodic phases */
    /**************************************/

    // Generate a state space diagram for the initialization and periodic phase
    // of an LF program.
    StateSpaceDiagram stateSpaceInitAndPeriodic =
        StateSpaceUtils.generateStateSpaceDiagram(explorer, Phase.INIT_AND_PERIODIC, main, new Tag(0,0,true), targetConfig, graphDir, "state_space_" + Phase.INIT_AND_PERIODIC);

    // Split the graph into a list of diagrams.
    List<StateSpaceDiagram> splittedDiagrams
      = StateSpaceUtils.splitInitAndPeriodicDiagrams(stateSpaceInitAndPeriodic);

    // Merge async diagrams into the init and periodic diagrams.
    for (int i = 0; i < splittedDiagrams.size(); i++) {
      var diagram = splittedDiagrams.get(i);
      splittedDiagrams.set(i, StateSpaceUtils.mergeAsyncDiagramsIntoDiagram(asyncDiagrams, diagram));
      // Generate a dot file.
      if (!diagram.isEmpty()) {
        Path file = graphDir.resolve("merged_" + i + ".dot");
        diagram.generateDotFile(file);
      } else {
        System.out.println("*** Merged diagram is empty!");
      }
    }
    
    // Convert the diagrams into fragments (i.e., having a notion of upstream &
    // downstream and carrying object file) and add them to the fragments list.
    for (var diagram : splittedDiagrams) {
      fragments.add(new StateSpaceFragment(diagram));
    }

    // If there are exactly two fragments (init and periodic),
    // connect the first fragment to the async fragment and connect
    // the async fragment to the second fragment.
    if (splittedDiagrams.size() == 2) {
      StateSpaceUtils.connectFragmentsDefault(fragments.get(0), fragments.get(1));
    }

    // If the last fragment is periodic, make it transition back to itself.
    StateSpaceFragment lastFragment = fragments.get(fragments.size() - 1);
    if (lastFragment.getPhase() == Phase.PERIODIC)
      StateSpaceUtils.connectFragmentsDefault(lastFragment, lastFragment);

    if (fragments.size() > 2) {
      throw new RuntimeException("More than two fragments detected!");
    }

    // Get the init or periodic fragment, whichever is currently the last in the list.
    StateSpaceFragment initOrPeriodicFragment = fragments.get(fragments.size() - 1);

    /******************/
    /* Shutdown phase */
    /******************/

    // Scenario 1: TIMEOUT
    // Generate a state space diagram for the timeout scenario of the
    // shutdown phase.
    if (targetConfig.get(TimeOutProperty.INSTANCE) != null) {
      StateSpaceFragment shutdownTimeoutFrag =
          new StateSpaceFragment(StateSpaceUtils.generateStateSpaceDiagram(explorer, Phase.SHUTDOWN_TIMEOUT, main, new Tag(0,0,true), targetConfig, graphDir, "state_space_" + Phase.SHUTDOWN_TIMEOUT));

      if (!shutdownTimeoutFrag.getDiagram().isEmpty()) {

        // Generate a guarded transition.
        // Only transition to this fragment when offset >= timeout.
        List<Instruction> guardedTransition = new ArrayList<>();
        guardedTransition.add(
            new InstructionBGE(
                GlobalVarType.GLOBAL_OFFSET, GlobalVarType.GLOBAL_TIMEOUT, Phase.SHUTDOWN_TIMEOUT));

        // Connect init or periodic fragment to the shutdown-timeout fragment.
        StateSpaceUtils.connectFragmentsGuarded(
            initOrPeriodicFragment, shutdownTimeoutFrag, guardedTransition);

        // Connect the shutdown-timeout fragment to epilogue (which is not a
        // real fragment, so we use a new StateSpaceFragment() instead.
        // The guarded transition is the important component here.)
        // FIXME: It could make more sense to store the STP in the EPILOGUE's object
        // file, instead of directly injecting code during link time. This might not have any
        // performance benefit, but it might make the pipeline more intuitive.
        StateSpaceUtils.connectFragmentsDefault(shutdownTimeoutFrag, StateSpaceFragment.EPILOGUE);

        // Add the shutdown-timeout fragment to the list of fragments.
        fragments.add(shutdownTimeoutFrag); // Add new fragments to the list.
      }
    }

    // Scenario 2: STARVATION
    // Generate a state space diagram for the starvation scenario of the
    // shutdown phase.
    // FIXME: We do not need this fragment if the system has timers.
    // FIXME: We need a way to check for starvation. One approach is to encode
    // triggers explicitly as global variables, and use conditional branch to
    // jump to this fragment if all trigger variables are indicating absent.
    /*
    StateSpaceFragment shutdownStarvationFrag =
        new StateSpaceFragment(generateStateSpaceDiagram(explorer, Phase.SHUTDOWN_STARVATION));
    if (!shutdownStarvationFrag.getDiagram().isEmpty()) {
      StateSpaceUtils.connectFragmentsDefault(initOrPeriodicFragment, shutdownStarvationFrag);
      fragments.add(shutdownStarvationFrag); // Add new fragments to the list.
    }
    */

    // Generate fragment dot files for debugging
    for (int i = 0; i < fragments.size(); i++) {
      Path file = graphDir.resolve("state_space_fragment_" + i + ".dot");
      fragments.get(i).getDiagram().generateDotFile(file);
    }

    // TODO: Compose all fragments into a single dot file.

    return fragments;
  }

  /** Create a static scheduler based on target property. */
  private StaticScheduler createStaticScheduler() {
    return switch (this.targetConfig.get(SchedulerProperty.INSTANCE).staticScheduler()) {
      case LOAD_BALANCED -> new LoadBalancedScheduler(this.graphDir);
      case EGS -> new EgsScheduler(this.fileConfig);
      case MOCASIN -> new MocasinScheduler(this.fileConfig, this.targetConfig);
      default -> new LoadBalancedScheduler(this.graphDir);
    };
  }
}
