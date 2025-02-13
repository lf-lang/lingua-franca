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
import org.lflang.MessageReporter;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.pretvm.PartialSchedule;
import org.lflang.pretvm.Registers;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.WorkersProperty;

public class CScheduleGenerator {

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

  /** PretVM registers */
  protected Registers registers;

  /** Flag indicating whether optimizers are used */
  protected boolean optimize = false;

  // Constructor
  public CScheduleGenerator(
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
    // DagGenerator dagGenerator = new DagGenerator(this.fileConfig);

    // Create a scheduler.
    // StaticScheduler scheduler = createStaticScheduler();
  }
}
