package org.lflang.pretvm.scheduler;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagNode;
import org.lflang.pretvm.dag.JobNode;

/**
 * A static scheduler that distributes work evenly among workers in a best-effort way
 *
 * @author Shaokai J. Lin
 */
public class LoadBalancedScheduler extends StaticScheduler {

  /** Directory where graphs are stored */
  protected final Path graphDir;

  public LoadBalancedScheduler(Path graphDir) {
    this.graphDir = graphDir;
  }

  public class Worker {
    private long totalWCET = 0;
    private List<JobNode> tasks = new ArrayList<>();

    public void addTask(JobNode task) {
      tasks.add(task);
      totalWCET += task.getReaction().wcet.toNanoSeconds();
    }

    public long getTotalWCET() {
      return totalWCET;
    }
  }

  public Dag partitionDag(Dag dag, MessageReporter reporter, int fragmentId, int numWorkers) {

    // Prune redundant edges.
    dag.removeRedundantEdges();

    // Initialize workers
    Worker[] workers = new Worker[numWorkers];
    for (int i = 0; i < numWorkers; i++) {
      workers[i] = new Worker();
    }

    // Sort tasks in descending order by WCET
    List<JobNode> reactionNodes =
        dag.dagNodes.stream()
            .filter(node -> node instanceof JobNode)
            .map(it -> (JobNode) it)
            .collect(Collectors.toCollection(ArrayList::new));

    reactionNodes.sort(
        Comparator.comparing((JobNode node) -> node.getReaction().wcet.toNanoSeconds()).reversed());

    // Assign tasks to workers
    for (JobNode node : reactionNodes) {
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
      List<JobNode> partition = dag.partitions.get(j);
      String randomColor = generateRandomColor();
      for (int i = 0; i < partition.size(); i++) {
        partition.get(i).setColor(randomColor);
        partition.get(i).setWorker(j);
      }
    }

    // Linearize partitions by adding edges.
    linearizePartitions(dag, numWorkers);

    // Prune redundant edges again.
    dag.removeRedundantEdges();

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

  /**
   * A valid DAG must linearize all nodes within a partition, such that there is a chain from the
   * first node to the last node executed by a worker owning the partition. In other words, the
   * width of the partition needs to be 1. Forming this chain enables WCET analysis at the system
   * level by tracing back edges from the end node. It also makes it clear what the order of
   * execution in a partition is.
   *
   * @param dag Dag whose partitions are to be linearized
   */
  private void linearizePartitions(Dag dag, int numWorkers) {
    // Initialize an array of previous nodes.
    DagNode[] prevNodes = new DagNode[numWorkers];
    for (int i = 0; i < prevNodes.length; i++) prevNodes[i] = null;

    for (DagNode current : dag.getTopologicalSort()) {
      if (current instanceof JobNode jn) {
        int worker = jn.getWorker();

        // Check if the previous node of the partition is null. If so, store the
        // node and go to the next iteration.
        if (prevNodes[worker] == null) {
          prevNodes[worker] = current;
          continue;
        }

        // Draw an edge between the previous node and the current node.
        dag.addEdge(prevNodes[worker], current);

        // Update previous nodes.
        prevNodes[worker] = current;
      }
    }
  }
}
