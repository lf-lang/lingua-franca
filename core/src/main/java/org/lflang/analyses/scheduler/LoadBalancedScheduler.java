package org.lflang.analyses.scheduler;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;

/**
 * A simple static scheduler that split work evenly among workers
 *
 * @author Shaokai Lin
 */
public class LoadBalancedScheduler implements StaticScheduler {

  /** Directory where graphs are stored */
  protected final Path graphDir;

  public LoadBalancedScheduler(Path graphDir) {
    this.graphDir = graphDir;
  }

  public class Worker {
    private long totalWCET = 0;
    private List<DagNode> tasks = new ArrayList<>();

    public void addTask(DagNode task) {
      tasks.add(task);
      totalWCET += task.getReaction().wcet.toNanoSeconds();
    }

    public long getTotalWCET() {
      return totalWCET;
    }
  }

  public Dag partitionDag(Dag dagRaw, int numWorkers, String filePostfix) {

    // Prune redundant edges.
    Dag dag = StaticSchedulerUtils.removeRedundantEdges(dagRaw);

    // Generate a dot file.
    Path file = graphDir.resolve("dag_pruned" + filePostfix + ".dot");
    dag.generateDotFile(file);

    // Initialize workers
    Worker[] workers = new Worker[numWorkers];
    for (int i = 0; i < numWorkers; i++) {
      workers[i] = new Worker();
    }

    // Sort tasks in descending order by WCET
    List<DagNode> reactionNodes =
        dag.dagNodes.stream()
            .filter(node -> node.nodeType == dagNodeType.REACTION)
            .collect(Collectors.toCollection(ArrayList::new));
    reactionNodes.sort(
        Comparator.comparing((DagNode node) -> node.getReaction().wcet.toNanoSeconds()).reversed());

    // Assign tasks to workers
    for (DagNode node : reactionNodes) {
      // Find worker with least work
      Worker minWorker =
          Arrays.stream(workers).min(Comparator.comparing(Worker::getTotalWCET)).orElseThrow();

      // Assign task to this worker
      minWorker.addTask(node);
    }

    // Update partitions
    for (int i = 0; i < numWorkers; i++) {
      dag.partitions.add(workers[i].tasks);
    }

    // Assign colors to each partition
    for (int j = 0; j < dag.partitions.size(); j++) {
      List<DagNode> partition = dag.partitions.get(j);
      String randomColor = StaticSchedulerUtils.generateRandomColor();
      for (int i = 0; i < partition.size(); i++) {
        partition.get(i).setColor(randomColor);
        partition.get(i).setWorker(j);
      }
    }

    // Generate another dot file.
    Path file2 = graphDir.resolve("dag_partitioned" + filePostfix + ".dot");
    dag.generateDotFile(file2);

    return dag;
  }

  /**
   * If the number of workers is unspecified, determine a value for the number of workers. This
   * scheduler base class simply returns 1. An advanced scheduler is free to run advanced algorithms
   * here.
   */
  public int setNumberOfWorkers() {
    return 1;
  }
}
