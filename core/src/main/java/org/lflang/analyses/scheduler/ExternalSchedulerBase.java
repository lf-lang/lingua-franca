package org.lflang.analyses.scheduler;

import java.io.IOException;
import java.nio.file.Path;

import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;
import org.lflang.generator.CodeBuilder;

/** 
 * A base class for all schedulers that are invoked as separate processes.
 */
public class ExternalSchedulerBase implements StaticScheduler {

    DagGenerator dagGenerator;

    public ExternalSchedulerBase(DagGenerator dagGenerator) {
        this.dagGenerator = dagGenerator;
    }
    
    public Dag generatePartitionedDag() {
        // Use a DAG scheduling algorithm to partition the DAG.
        // Construct a process to run the Python program of the RL agent
        ProcessBuilder dagScheduler = new ProcessBuilder(
            "python3",
            "script.py", // FIXME: to be updated with the script file name
            "dag.dot"
        );

        try {
            // If the partionned DAG file is generated, then read the contents
            // and update the edges array.
            Process dagSchedulerProcess = dagScheduler.start();
            
            // Wait until the process is done
            int exitValue = dagSchedulerProcess.waitFor();
            
            // FIXME: Put the correct file name
            this.dagGenerator.updateDag("partionedDagFileName.dot");
        } catch (InterruptedException | IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
        // Note: this is for double checking...
        // Generate another dot file with the updated Dag.
        try {
            CodeBuilder dot = this.dagGenerator.generateDot();
            Path srcgen = this.dagGenerator.fileConfig.getSrcGenPath();
            Path file = srcgen.resolve("dagUpdated.dot");
            String filename = file.toString();
            dot.writeToFile(filename);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // FIXME
        return null;
    }

}
