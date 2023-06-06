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
import java.nio.file.Path;
import java.lang.ProcessBuilder;

import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.Tag;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;

public class CStaticScheduleGenerator {

    public ReactorInstance     main;
    public StateSpaceExplorer  explorer;
    public StateSpaceDiagram   stateSpaceDiagram;
    public DagGenerator        dagGenerator;

    /** File config */
    protected final CFileConfig fileConfig;

    // Constructor
    public CStaticScheduleGenerator(
        CFileConfig fileConfig,
        ReactorInstance main
    ) {
        this.fileConfig = fileConfig;
        this.main       = main;
        this.explorer   = new StateSpaceExplorer(main);
    }

    // Main function for generating a static schedule file in C.
    public void generate() {
        // Generate a state space diagram for the LF program.
        // FIXME: An infinite horizon may lead to non-termination.
        this.explorer.explore(new Tag(0, 0, true), true);
        this.stateSpaceDiagram = this.explorer.getStateSpaceDiagram();
        this.stateSpaceDiagram.display();

        // Generate a pre-processed DAG from the state space diagram.
        this.dagGenerator = new DagGenerator(
            this.fileConfig,
            this.main,
            this.stateSpaceDiagram);
        this.dagGenerator.generateDag();

        // Generate a dot file.
        try {
            CodeBuilder dot = this.dagGenerator.generateDot();
            Path srcgen = fileConfig.getSrcGenPath();
            Path file = srcgen.resolve("dag.dot");
            String filename = file.toString();
            dot.writeToFile(filename);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Use a DAG scheduling algorithm to partition the DAG.
        // Construct a process to run the Python program of the RL agent
        ProcessBuilder dagScheduler = new ProcessBuilder(
            "python3",
            "script.py", // FIXME: to be updated with the script file name
            "dag.dot"
        );

        // If the partionned DAG file is generated, then read the contents
        // and update the edges array.
        try {
            Process dagSchedulerProcess = dagScheduler.start();
            
            // Wait until the process is done
            int exitValue = dagSchedulerProcess.waitFor();
            
            // FIXME: Put the correct file name
            this.dagGenerator.updateDag("partionedDagFileName.odt");
        } catch (InterruptedException | IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Note: this is for double checking...
        // Generate another dot file with the updated Dag.
        try {
            CodeBuilder dot = this.dagGenerator.generateDot();
            Path srcgen = fileConfig.getSrcGenPath();
            Path file = srcgen.resolve("dagUpdated.dot");
            String filename = file.toString();
            dot.writeToFile(filename);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Generate VM instructions for each DAG partition.
        // can be something like: generateVMInstructions(partionedDag); 
    }
    
}
