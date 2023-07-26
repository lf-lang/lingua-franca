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
    Path finalDotFile = srcgen.resolve("dagFinal.dot");
    // FIXME: Make the script file part of the target config
    Path scriptFile = src.resolve("egs_script.sh");

    // Start by generating the .dot file from the DAG
    dag.generateDotFile(dotFile);

    // Construct a process to run the Python program of the RL agent
    ProcessBuilder dagScheduler =
        new ProcessBuilder(
            "bash", 
            scriptFile.toString(),
            dotFile.toString(),
            finalDotFile.toString(), 
            String.valueOf(workers)
        );

    // Use a DAG scheduling algorithm to partition the DAG.
    try {
      // If the partionned DAG file is generated, then read the contents
      // and update the edges array.
      Process dagSchedulerProcess = dagScheduler.start();

      // Wait until the process is done
      int exitValue = dagSchedulerProcess.waitFor();

      String dagSchedulerProcessOutput = new String(dagSchedulerProcess.getInputStream().readAllBytes());
      String dagSchedulerProcessError = new String(dagSchedulerProcess.getErrorStream().readAllBytes());
    
      if (!dagSchedulerProcessOutput.isEmpty()) {
        System.out.println(">>>>> EGS output: " + dagSchedulerProcessOutput);
      }
      if (!dagSchedulerProcessError.isEmpty()) {
        System.out.println(">>>>> EGS Error: " + dagSchedulerProcessError);
      }

      assert exitValue != 0 : "Problem calling the external static scheduler... Abort!";

    } catch (InterruptedException | IOException e) {
      throw new RuntimeException(e);
    }

    // Read the generated DAG
    try {
      dag.updateDag(finalDotFile.toString());
      System.out.println("=======================\nDag succesfully updated\n=======================");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // FIXME: Compute the partitions and perform graph coloring
    
    return dag;
  }

  public int setNumberOfWorkers() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNumberOfWorkers'");
  }
}
