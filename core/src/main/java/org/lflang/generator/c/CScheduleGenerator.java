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
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.pretvm.InstructionGenerator;
import org.lflang.pretvm.PartialSchedule;
import org.lflang.pretvm.Registers;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagGenerator;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.pretvm.scheduler.LoadBalancedScheduler;
import org.lflang.pretvm.scheduler.StaticScheduler;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.WorkersProperty;

public class CScheduleGenerator {

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

  /** A list of ports */
  protected List<PortInstance> ports;

  /** A path for storing graph */
  protected Path graphDir;

  /** PretVM registers */
  protected Registers registers;

  /** Flag indicating whether optimizers are used */
  protected boolean optimize = false;

  // Constructor
  public CScheduleGenerator(
      CFileConfig fileConfig,
      TargetConfig targetConfig,
      ReactorInstance main,
      List<ReactorInstance> reactors,
      List<ReactionInstance> reactions,
      List<PortInstance> ports) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.main = main;
    this.workers =
        targetConfig.get(WorkersProperty.INSTANCE) == 0
            ? 1
            : targetConfig.get(WorkersProperty.INSTANCE);
    this.reactors = reactors;
    this.reactions = reactions;
    this.ports = ports;
    this.registers = new Registers(workers);
    this.optimize = false;

    // Create a directory for storing graph.
    this.graphDir = fileConfig.getSrcGenPath().resolve("graphs");
    try {
      Files.createDirectories(this.graphDir);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Main function for generating static_schedule.c
  public void doGenerate() {
    // Generate a list of state space fragments that captures
    // all the behavior of the LF program.
    List<StateSpaceDiagram> SSDs =
        StateSpaceExplorer.generateStateSpaceDiagrams(main, targetConfig, this.graphDir);

    // Instantiate object files with SSDs and connect them.
    List<PartialSchedule> schedules = new ArrayList<>();
    for (var ssd : SSDs) {
      PartialSchedule ps = new PartialSchedule();
      ps.setDiagram(ssd);
      schedules.add(ps);
    }
    PartialSchedule.link(schedules, registers);

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
            this.fileConfig,
            this.targetConfig,
            this.workers,
            this.main,
            this.reactors,
            this.reactions,
            this.ports,
            this.registers);

    // For each partial schedule, generate a DAG, perform DAG scheduling (mapping tasks
    // to workers), and generate instructions for each worker.
    for (int i = 0; i < schedules.size(); i++) {
      // Get the partial schedule.
      PartialSchedule schedule = schedules.get(i);

      // Generate a raw DAG from the partial schedule's state space diagram.
      Dag dag = dagGenerator.generateDag(schedule.getDiagram());

      // Validate the generated raw DAG.
      if (!dag.isValid()) throw new RuntimeException("The generated DAG is invalid: " + dag);

      // Generate a dot file.
      Path dagRawDot = graphDir.resolve("dag_raw" + "_" + i + ".dot");
      dag.generateDotFile(dagRawDot);

      // Prune redundant edges and generate a dot file.
      // FIXME: To remove.
      dag.removeRedundantEdges();
      Path file = graphDir.resolve("dag_pruned" + "_" + i + ".dot");
      dag.generateDotFile(file);

      // Generate a partitioned DAG based on the number of workers,
      // and generate a dot graph.
      Dag dagPartitioned = scheduler.partitionDag(dag, i, this.workers);
      Path dagPartitionedDot = graphDir.resolve("dag_partitioned" + "_" + i + ".dot");
      dagPartitioned.generateDotFile(dagPartitionedDot);

      // Generate instructions (wrapped in an object file) from DAG partitions.
      List<List<Instruction>> instructions = instGen.generateInstructions(dagPartitioned, schedule);

      // TODO: Check if deadlines could be violated.

      // Point the partitioned DAG and the instructions to the partial schedule.
      schedule.setDag(dagPartitioned);
      schedule.setInstructions(instructions);
    }

    // Invoke the dag-based optimizer on each object file.
    // It is invoked before linking because after linking,
    // the DAG information is gone.

    // Link multiple object files into a single executable
    // (represented also in an object file class).
    // Instructions are also inserted based on transition guards between fragments.
    // In addition, PREAMBLE and EPILOGUE instructions are inserted here.
    List<List<Instruction>> linkedInstructions = instGen.link(schedules, graphDir);

    // Invoke the peephole optimizer.
    // FIXME: Should only apply to basic blocks!

    // Generate C code.
    instGen.generateCode(linkedInstructions);
  }

  /** Create a static scheduler. */
  private StaticScheduler createStaticScheduler() {
    return new LoadBalancedScheduler(this.graphDir);
  }
}
