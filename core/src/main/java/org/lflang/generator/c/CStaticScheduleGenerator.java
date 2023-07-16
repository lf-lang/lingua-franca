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

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.analyses.evm.EvmObjectFile;
import org.lflang.analyses.evm.InstructionGenerator;
import org.lflang.analyses.scheduler.BaselineScheduler;
import org.lflang.analyses.scheduler.ExternalSchedulerBase;
import org.lflang.analyses.scheduler.StaticScheduler;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
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
  }

  // Main function for generating a static schedule file in C.
  public void generate() {

    // Generate a state space diagram for the LF program.
    StateSpaceDiagram stateSpace = generateStateSpaceDiagram();

    // Split the diagrams into a list of diagram fragments.
    Path srcgen = fileConfig.getSrcGenPath();
    ArrayList<StateSpaceFragment> fragments = StateSpaceUtils.fragmentizeForDagGen(stateSpace, srcgen);

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
        new InstructionGenerator(this.fileConfig, this.workers, this.reactors, this.reactions);

    // For each fragment, generate a DAG, perform DAG scheduling (mapping tasks
    // to workers), and generate instructions for each worker.
    List<EvmObjectFile> evmObjectFiles = new ArrayList<>();
    for (var i = 0; i < fragments.size(); i++) {
      StateSpaceFragment fragment = fragments.get(i);

      // Generate a raw DAG from a state space fragment.
      Dag dag = dagGenerator.generateDag(fragment);

      // Generate a dot file.
      Path file = srcgen.resolve("dag_raw" + "_frag_" + i + ".dot");
      dag.generateDotFile(file);

      // Generate a partitioned DAG based on the number of workers.
      Dag dagPartitioned = scheduler.partitionDag(dag, this.workers, "_frag_" + i);

      // Generate instructions (wrapped in an object file) from DAG partitions.
      EvmObjectFile objectFile = instGen.generateInstructions(dagPartitioned, fragment.hyperperiod);
      evmObjectFiles.add(objectFile);
    }

    // Link the fragments and produce a single Object File.
    EvmObjectFile executable = instGen.link(evmObjectFiles);

    // Generate C code.
    instGen.generateCode(executable);
  }

  /** Generate a state space diagram for the LF program. */
  private StateSpaceDiagram generateStateSpaceDiagram() {
    StateSpaceExplorer explorer = new StateSpaceExplorer(this.main);
    // FIXME: An infinite horizon may lead to non-termination.
    explorer.explore(new Tag(0, 0, true), true);
    StateSpaceDiagram stateSpaceDiagram = explorer.getStateSpaceDiagram();

    // Generate a dot file.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("state_space.dot");
    stateSpaceDiagram.generateDotFile(file);

    return stateSpaceDiagram;
  }

  /** Create a static scheduler based on target property. */
  private StaticScheduler createStaticScheduler() {
    return switch (this.targetConfig.staticScheduler) {
      case BASELINE -> new BaselineScheduler(this.fileConfig);
      case RL -> new ExternalSchedulerBase(this.fileConfig); // FIXME
    };
  }
}
