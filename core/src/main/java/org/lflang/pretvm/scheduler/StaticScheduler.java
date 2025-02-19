package org.lflang.pretvm.scheduler;

import java.util.List;
import java.util.Random;
import org.lflang.MessageReporter;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.JobNode;

/**
 * Interface for static scheduler
 *
 * @author Shaokai J. Lin
 */
public abstract class StaticScheduler {

  ////////////////////////////////////////////////////////////////////
  /// Abstract methods to be implemented by child classes

  public abstract Dag partitionDag(Dag dag, MessageReporter reporter, int fragmentId, int workers);

  public abstract int setNumberOfWorkers();

  ////////////////////////////////////////////////////////////////////
  /// Static utility methods

  public static String generateRandomColor() {
    Random random = new Random();
    int r = random.nextInt(256);
    int g = random.nextInt(256);
    int b = random.nextInt(256);

    return String.format("#%02X%02X%02X", r, g, b);
  }

  public static void assignColorsToPartitions(Dag dag) {
    // Assign colors to each partition
    for (int j = 0; j < dag.partitions.size(); j++) {
      List<JobNode> partition = dag.partitions.get(j);
      String randomColor = generateRandomColor();
      for (int i = 0; i < partition.size(); i++) {
        partition.get(i).setColor(randomColor);
        partition.get(i).setWorker(j);
      }
    }
  }
}
