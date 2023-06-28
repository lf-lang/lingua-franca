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
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.analyses.evm.InstructionGenerator;
import org.lflang.analyses.scheduler.BaselineScheduler;
import org.lflang.analyses.scheduler.ExternalSchedulerBase;
import org.lflang.analyses.scheduler.StaticScheduler;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
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
  // FIXME: "generate" is mostly used for code generation.
  public void generate() {

    StateSpaceDiagram stateSpace = generateStateSpaceDiagram();

    Dag dag = generateDagFromStateSpaceDiagram(stateSpace);

    generatePartitionsFromDag(dag);

    generateInstructionsFromPartitions(dag);
  }

  /** Generate a state space diagram for the LF program. */
  public StateSpaceDiagram generateStateSpaceDiagram() {
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

  /** Generate a pre-processed DAG from the state space diagram. */
  public Dag generateDagFromStateSpaceDiagram(StateSpaceDiagram stateSpace) {
    // Generate a pre-processed DAG from the state space diagram.
    DagGenerator dagGenerator = new DagGenerator(this.fileConfig, this.main, stateSpace);
    dagGenerator.generateDag();

    // Generate a dot file.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("dag_raw.dot");
    dagGenerator.getDag().generateDotFile(file);

    return dagGenerator.getDag();
  }

  /** Generate a partitioned DAG based on the number of workers. */
  public void generatePartitionsFromDag(Dag dag) {

    // Create a scheduler.
    StaticScheduler scheduler = createStaticScheduler(dag);

    // Determine the number of workers, if unspecified.
    if (this.workers == 0) {
      this.workers = scheduler.setNumberOfWorkers();
      // Update the previous value of 0.
      targetConfig.workers = this.workers;
      targetConfig.compileDefinitions.put(
          "NUMBER_OF_WORKERS", String.valueOf(targetConfig.workers));
    }

    // Perform scheduling.
    scheduler.partitionDag(this.workers);
  }

  /** Create a static scheduler based on target property. */
  public StaticScheduler createStaticScheduler(Dag dag) {
    return switch (this.targetConfig.staticScheduler) {
      case BASELINE -> new BaselineScheduler(dag, this.fileConfig);
      case RL -> new ExternalSchedulerBase(dag, this.fileConfig); // FIXME
    };
  }

  /** Generate VM instructions for each DAG partition. */
  public void generateInstructionsFromPartitions(Dag dagParitioned) {
    InstructionGenerator instGen =
        new InstructionGenerator(
            dagParitioned,
            this.fileConfig,
            this.targetConfig,
            this.workers,
            this.reactors,
            this.reactions);
    instGen.generateInstructions();
    instGen.display();
    instGen.generateCode();

    // Generate a dot file.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("dag_debug.dot");
    instGen.getDag().generateDotFile(file);
  }
}
