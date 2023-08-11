package org.lflang.analyses.scheduler;

import java.io.IOException;
import java.nio.file.Path;
import org.lflang.analyses.dag.Dag;
import org.lflang.generator.c.CFileConfig;

/**
 * An external static scheduler based on edge generation
 *
 * @author Chadlia Jerad
 * @author Shaokai Lin
 */
public class EgsScheduler implements StaticScheduler {

  /** File config */
  protected final CFileConfig fileConfig;

  public EgsScheduler(CFileConfig fileConfig) {
    this.fileConfig = fileConfig;
  }

  public Dag partitionDag(Dag dag, int workers, String filePostfix) {
    // Set all Paths and files
    Path src = this.fileConfig.srcPath;
    Path graphDir = fileConfig.getSrcGenPath().resolve("graphs");

    // Files
    Path rawDagDotFile = graphDir.resolve("dag_raw" + filePostfix + ".dot");
    Path partionedDagDotFile = graphDir.resolve("dag_partitioned" + filePostfix + ".dot");
    // FIXME: Make the script file part of the target config?
    Path scriptFile = src.resolve("egs_script.sh");

    // Start by generating the .dot file from the DAG
    dag.generateDotFile(rawDagDotFile);

    // Construct a process to run the Python program of the RL agent
    ProcessBuilder dagScheduler =
        new ProcessBuilder(
            "bash",
            scriptFile.toString(),
            rawDagDotFile.toString(),
            partionedDagDotFile.toString(),
            String.valueOf(workers));

    // Use a DAG scheduling algorithm to partition the DAG.
    try {
      // If the partionned DAG file is generated, then read the contents
      // and update the edges array.
      Process dagSchedulerProcess = dagScheduler.start();

      // Wait until the process is done
      int exitValue = dagSchedulerProcess.waitFor();

      String dagSchedulerProcessOutput =
          new String(dagSchedulerProcess.getInputStream().readAllBytes());
      String dagSchedulerProcessError =
          new String(dagSchedulerProcess.getErrorStream().readAllBytes());

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
      dag.updateDag(partionedDagDotFile.toString());
      System.out.println(
          "=======================\nDag succesfully updated\n=======================");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // TODO: Check the number of workers
    // TODO: Perform graph coloring

    return dag;
  }

  public int setNumberOfWorkers() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNumberOfWorkers'");
  }
}
