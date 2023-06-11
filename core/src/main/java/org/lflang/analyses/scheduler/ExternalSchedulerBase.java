package org.lflang.analyses.scheduler;

import java.io.IOException;
import java.nio.file.Path;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagGenerator;

/** A base class for all schedulers that are invoked as separate processes. */
public class ExternalSchedulerBase extends StaticSchedulerBase {

  DagGenerator dagGenerator;

  public ExternalSchedulerBase(Dag dag, DagGenerator dagGenerator) {
    super(dag);
    this.dagGenerator = dagGenerator;
  }

  @Override
  public void partitionDag(int workers) {
    // Use a DAG scheduling algorithm to partition the DAG.
    // Construct a process to run the Python program of the RL agent
    ProcessBuilder dagScheduler =
        new ProcessBuilder(
            "python3",
            "script.py", // FIXME: to be updated with the script file name
            "dag.dot");

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
    Path srcgen = this.dagGenerator.fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("dagUpdated.dot");
    dag.generateDotFile(file);
  }

  @Override
  public void removeRedundantEdges() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'removeRedundantEdges'");
  }
}
