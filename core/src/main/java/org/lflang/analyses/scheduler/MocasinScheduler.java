package org.lflang.analyses.scheduler;

import java.nio.file.Path;
import org.lflang.analyses.dag.Dag;

/** An external static scheduler using the `mocasin` tool */
public class MocasinScheduler implements StaticScheduler {

  /** Directory where graphs are stored */
  protected final Path graphDir;

  public MocasinScheduler(Path graphDir) {
    this.graphDir = graphDir;
  }

  public Dag turnDagIntoSdfFormat(Dag dagRaw) {
    // Create a copy of the original dag.
    Dag dag = new Dag(dagRaw);

    return dag;
  }

  public Dag partitionDag(Dag dagRaw, int numWorkers, String dotFilePostfix) {

    // Prune redundant edges.
    Dag dagPruned = StaticSchedulerUtils.removeRedundantEdges(dagRaw);

    // Generate a dot file.
    Path file = graphDir.resolve("dag_pruned" + dotFilePostfix + ".dot");
    dagPruned.generateDotFile(file);

    // Prune redundant edges.
    Dag dagSdf = turnDagIntoSdfFormat(dagPruned);

    return dagSdf;
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
