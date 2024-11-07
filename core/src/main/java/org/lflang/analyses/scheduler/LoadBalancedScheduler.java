package org.lflang.analyses.scheduler;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
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
      totalWCET += task.getReaction().wcets.get(0).toNanoSeconds();
    }

    public long getTotalWCET() {
      return totalWCET;
    }
  }

  public Dag partitionDag(
      Dag dag, MessageReporter reporter, int fragmentId, int numWorkers, String filePostfix) {

    // Prune redundant edges.
    dag.removeRedundantEdges();

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
        Comparator.comparing(
                (DagNode node) ->
                    node.getReaction()
                        .wcets
                        .get(0) // The default scheduler only assumes 1 WCET.
                        .toNanoSeconds())
            .reversed());

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

    // Linearize partitions by adding edges.
    linearizePartitions(dag, numWorkers);

    // Prune redundant edges again.
    dag.removeRedundantEdges();

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

  /**
   * A valid DAG must linearize all nodes within a partition, such that there is a chain from the
   * first node to the last node executed by a worker owning the partition. In other words, the
   * width of the partition needs to be 1. Forming this chain enables WCET analysis at the system
   * level by tracing back edges from the tail node. It also makes it clear what the order of
   * execution in a partition is.
   *
   * @param dag Dag whose partitions are to be linearized
   */
  private void linearizePartitions(Dag dag, int numWorkers) {
    // Initialize an array of previous nodes.
    DagNode[] prevNodes = new DagNode[numWorkers];
    for (int i = 0; i < prevNodes.length; i++) prevNodes[i] = null;

    for (DagNode current : dag.getTopologicalSort()) {
      if (current.nodeType == dagNodeType.REACTION) {
        int worker = current.getWorker();

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
