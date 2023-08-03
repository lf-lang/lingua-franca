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
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.analyses.pretvm.InstructionGenerator;
import org.lflang.analyses.pretvm.PretVmExecutable;
import org.lflang.analyses.pretvm.PretVmObjectFile;
import org.lflang.analyses.scheduler.BaselineScheduler;
import org.lflang.analyses.scheduler.EgsScheduler;
import org.lflang.analyses.scheduler.MocasinScheduler;
import org.lflang.analyses.scheduler.StaticScheduler;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.StateSpaceExplorer.Mode;
import org.lflang.analyses.statespace.StateSpaceFragment;
import org.lflang.analyses.statespace.StateSpaceUtils;
import org.lflang.analyses.statespace.Tag;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;

public class CStaticScheduleGenerator {

  /** File config */
  protected final CFileConfig fileConfig;

  /** Target configuration */
  protected TargetConfig targetConfig;

  /** Main reactor instance */
  protected ReactorInstance main;

  /** The number of workers to schedule for */
  protected int workers;

  /** A list of reactor instances */
  protected List<ReactorInstance> reactors;

  /** A list of reaction instances */
  protected List<ReactionInstance> reactions;

  /** A path for storing graph */
  protected Path graphDir;

  // Constructor
  public CStaticScheduleGenerator(
      CFileConfig fileConfig,
      TargetConfig targetConfig,
      ReactorInstance main,
      List<ReactorInstance> reactorInstances,
      List<ReactionInstance> reactionInstances) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.main = main;
    this.workers = targetConfig.workers;
    this.reactors = reactorInstances;
    this.reactions = reactionInstances;

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
      targetConfig.workers = this.workers;
      targetConfig.compileDefinitions.put(
          "NUMBER_OF_WORKERS", String.valueOf(targetConfig.workers));
    }

    // Create InstructionGenerator, which acts as a compiler and a linker.
    InstructionGenerator instGen =
        new InstructionGenerator(
            this.fileConfig, this.targetConfig, this.workers, this.reactors, this.reactions);

    // For each fragment, generate a DAG, perform DAG scheduling (mapping tasks
    // to workers), and generate instructions for each worker.
    List<PretVmObjectFile> pretvmObjectFiles = new ArrayList<>();
    for (var i = 0; i < fragments.size(); i++) {
      StateSpaceFragment fragment = fragments.get(i);

      // Generate a raw DAG from a state space fragment.
      Dag dag = dagGenerator.generateDag(fragment.getDiagram());

      // Generate a dot file.
      Path file = graphDir.resolve("dag_raw" + "_frag_" + i + ".dot");
      dag.generateDotFile(file);

      // Generate a partitioned DAG based on the number of workers.
      Dag dagPartitioned = scheduler.partitionDag(dag, this.workers, "_frag_" + i);

      // Generate instructions (wrapped in an object file) from DAG partitions.
      PretVmObjectFile objectFile = instGen.generateInstructions(dagPartitioned, fragment);
      pretvmObjectFiles.add(objectFile);
    }

    // Link the fragments and produce a single Object File.
    PretVmExecutable executable = instGen.link(pretvmObjectFiles);

    // Generate C code.
    instGen.generateCode(executable);
  }

  /**
   * A helper function that generates a state space diagram for an LF program based on an
   * exploration mode.
   */
  private StateSpaceDiagram generateStateSpaceDiagram(
      StateSpaceExplorer explorer, StateSpaceExplorer.Mode exploreMode) {
    // Explore the state space with the mode specified.
    StateSpaceDiagram stateSpaceDiagram = explorer.explore(main, new Tag(0, 0, true), exploreMode);

    // Generate a dot file.
    if (!stateSpaceDiagram.isEmpty()) {
      Path file = graphDir.resolve("state_space_" + exploreMode + ".dot");
      stateSpaceDiagram.generateDotFile(file);
    }

    return stateSpaceDiagram;
  }

  /**
   * Generate a list of state space fragments for an LF program. This function calls
   * generateStateSpaceDiagram(<mode>) multiple times to capture the full behavior of the LF
   * program.
   */
  private List<StateSpaceFragment> generateStateSpaceFragments() {

    // Initialize variables
    StateSpaceExplorer explorer = new StateSpaceExplorer(targetConfig);
    List<StateSpaceFragment> fragments = new ArrayList<>();

    /* Initialization and Periodic phases */

    // Generate a state space diagram for the initialization and periodic phase
    // of an LF program.
    StateSpaceDiagram stateSpaceInitAndPeriodic =
        generateStateSpaceDiagram(explorer, Mode.INIT_AND_PERIODIC);

    // Split the graph into a list of diagram fragments.
    fragments.addAll(StateSpaceUtils.fragmentizeInitAndPeriodic(stateSpaceInitAndPeriodic));

    /* Shutdown phase */

    // Generate a state space diagram for the timeout scenario of the
    // shutdown phase.
    if (targetConfig.timeout != null) {
      StateSpaceFragment shutdownTimeoutFrag =
          new StateSpaceFragment(generateStateSpaceDiagram(explorer, Mode.SHUTDOWN_TIMEOUT));
      if (!shutdownTimeoutFrag.getDiagram().isEmpty()) {
        StateSpaceUtils.connectFragments(fragments.get(fragments.size() - 1), shutdownTimeoutFrag);
        fragments.add(shutdownTimeoutFrag); // Add new fragments to the list.
      }
    }

    // Generate a state space diagram for the starvation scenario of the
    // shutdown phase.
    // FIXME: We do not need this if the system has timers.
    StateSpaceFragment shutdownStarvationFrag =
        new StateSpaceFragment(generateStateSpaceDiagram(explorer, Mode.SHUTDOWN_STARVATION));
    if (!shutdownStarvationFrag.getDiagram().isEmpty()) {
      StateSpaceUtils.connectFragments(fragments.get(fragments.size() - 1), shutdownStarvationFrag);
      fragments.add(shutdownStarvationFrag); // Add new fragments to the list.
    }

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
    return switch (this.targetConfig.staticScheduler) {
      case BASELINE -> new BaselineScheduler(this.graphDir);
      case EGS -> new EgsScheduler(this.fileConfig);
      case MOCASIN -> new MocasinScheduler(this.fileConfig);
    };
  }
}
