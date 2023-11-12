package org.lflang.analyses.scheduler;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;
import org.lflang.generator.c.CFileConfig;

/**
 * An external static scheduler based on edge generation.
 * This scheduler assumes that all the python dependencies have been installed,
 * `egs.py` is added to the PATH variable, and there is a pretrained model
 * located at `models/pretrained` under the same directory as `egs.py`.
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

  public Dag partitionDag(Dag dag, int fragmentId, int workers, String filePostfix) {
    // Set all Paths and files
    Path src = this.fileConfig.srcPath;
    Path graphDir = fileConfig.getSrcGenPath().resolve("graphs");

    // Files
    Path rawDagDotFile = graphDir.resolve("dag_raw" + filePostfix + ".dot");
    Path partionedDagDotFile = graphDir.resolve("dag_partitioned" + filePostfix + ".dot");

    // Start by generating the .dot file from the DAG
    dag.generateDotFile(rawDagDotFile);

    // Find the directory where the EGS script is located.
    String egsDir = findEgsDirectory();

    // Construct a process to run the Python program of the RL agent
    ProcessBuilder dagScheduler =
        new ProcessBuilder(
            "egs.py",
            "--in_dot",
            rawDagDotFile.toString(),
            "--out_dot",
            partionedDagDotFile.toString(),
            "--workers",
            String.valueOf(workers + 1),
            "--model",
            new File(egsDir, "models/pretrained").getAbsolutePath());

    // Use a DAG scheduling algorithm to partition the DAG.
    try {

      // Redirect the output and error streams
      dagScheduler.redirectOutput(ProcessBuilder.Redirect.INHERIT);
      dagScheduler.redirectError(ProcessBuilder.Redirect.INHERIT);

      // If the partionned DAG file is generated, then read the contents
      // and update the edges array.
      Process dagSchedulerProcess = dagScheduler.start();

      // Wait until the process is done
      int exitValue = dagSchedulerProcess.waitFor();

      if (exitValue != 0)
        throw new RuntimeException("Problem calling the external static scheduler... Abort!");

    } catch (InterruptedException | IOException e) {
      throw new RuntimeException(e);
    }

    // Clone the initial dag
    Dag dagPartitioned = new Dag(dag);

    // Read the generated DAG
    try {
      dagPartitioned.updateDag(partionedDagDotFile.toString());
      System.out.println(
          "=======================\nDag succesfully updated\n=======================");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // FIXME: decrement all the workers by 1
    // FIXME (Shaokai): Why is this necessary?

    // Retreive the number of workers
    Set<Integer> setOfWorkers = new HashSet<>();
    for (int i = 0; i < dagPartitioned.dagNodes.size(); i++) {
      int workerId = dagPartitioned.dagNodes.get(i).getWorker();
      workerId--;
      dagPartitioned.dagNodes.get(i).setWorker(workerId);
      setOfWorkers.add(workerId);
    }

    int egsNumberOfWorkers = setOfWorkers.size() - 1;

    // Check that the returned number of workers is less than the one set by the user
    if (egsNumberOfWorkers > workers) {
      throw new RuntimeException(
          "The EGS scheduler returned a minimum number of workers of "
              + egsNumberOfWorkers
              + " while the user specified number is "
              + workers);
    }

    // Define a color for each worker
    String[] workersColors = new String[egsNumberOfWorkers];
    for (int i = 0; i < egsNumberOfWorkers; i++) {
      workersColors[i] = StaticSchedulerUtils.generateRandomColor();
    }

    // Set the color of each node
    for (int i = 0; i < dagPartitioned.dagNodes.size(); i++) {
      int wk = dagPartitioned.dagNodes.get(i).getWorker();
      if (wk != -1) dagPartitioned.dagNodes.get(i).setColor(workersColors[wk]);
    }

    // Set the partitions
    for (int i = 0; i < egsNumberOfWorkers ; i++) {
      List<DagNode> partition = new ArrayList<DagNode>();
      for (int j = 0; j < dagPartitioned.dagNodes.size(); j++) {
        int wk = dagPartitioned.dagNodes.get(j).getWorker();
        if (wk == i) {
          partition.add(dagPartitioned.dagNodes.get(j));
        }
      }
      dagPartitioned.partitions.add(partition);
    }

    Path dpu = graphDir.resolve("dag_partitioned" + filePostfix + ".dot");
    dagPartitioned.generateDotFile(dpu);

    return dagPartitioned;
  }

  public String findEgsDirectory() {
    try {
      // Find the full path of egs.py using 'which' command
      ProcessBuilder whichBuilder = new ProcessBuilder("which", "egs.py");
      Process whichProcess = whichBuilder.start();
      BufferedReader reader = new BufferedReader(new InputStreamReader(whichProcess.getInputStream()));
      String egsPath = reader.readLine();
      whichProcess.waitFor();

      // Assuming egsPath is not null and contains the full path to egs.py
      File egsFile = new File(egsPath);
      String egsDir = egsFile.getParent();
      return egsDir;
    } catch(InterruptedException | IOException e) {
      throw new RuntimeException(e);
    }
  }

  public int setNumberOfWorkers() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNumberOfWorkers'");
  }
}
