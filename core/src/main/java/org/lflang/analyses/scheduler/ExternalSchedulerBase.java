package org.lflang.analyses.scheduler;

import java.io.IOException;
import java.nio.file.Path;
import org.lflang.analyses.dag.Dag;
import org.lflang.generator.c.CFileConfig;

/** A base class for all schedulers that are invoked as separate processes. */
public class ExternalSchedulerBase implements StaticScheduler {

  /** File config */
  protected final CFileConfig fileConfig;

  public ExternalSchedulerBase(CFileConfig fileConfig) {
    this.fileConfig = fileConfig;
  }

  public Dag partitionDag(Dag dag, int workers, String dotFilePostfix) {
    // Set all Paths and files
    Path src = this.fileConfig.srcPath;
    Path srcgen = this.fileConfig.getSrcGenPath();

    // Files
    Path dotFile = srcgen.resolve("dag.dot");
    Path updatedDotFile = srcgen.resolve("dagUpdated.dot");
    Path finalDotFile = srcgen.resolve("dagFinal.dot");
    // FIXME: Make the script file part of the target config
    Path scriptFile = src.resolve("randomStaticScheduler.py");

    // Start by generating the .dot file from the DAG
    dag.generateDotFile(dotFile);

    // Construct a process to run the Python program of the RL agent
    ProcessBuilder dagScheduler =
        new ProcessBuilder(
            "python3",
            scriptFile.toString(),
            "-dot",
            dotFile.toString(),
            "-out",
            updatedDotFile.toString());

    // Use a DAG scheduling algorithm to partition the DAG.
    try {
      // If the partionned DAG file is generated, then read the contents
      // and update the edges array.
      Process dagSchedulerProcess = dagScheduler.start();

      // Wait until the process is done
      int exitValue = dagSchedulerProcess.waitFor();

      assert exitValue != 0 : "Problem calling the external static scheduler... Abort!";

      // Update the Dag
      dag.updateDag(updatedDotFile.toString());

    } catch (InterruptedException | IOException e) {
      throw new RuntimeException(e);
    }

    // Note: this is for double checking...
    // Generate another dot file with the updated Dag.
    dag.generateDotFile(finalDotFile);

    // FIXME: This does not work yet.
    return dag;
  }

  public int setNumberOfWorkers() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNumberOfWorkers'");
  }
}
