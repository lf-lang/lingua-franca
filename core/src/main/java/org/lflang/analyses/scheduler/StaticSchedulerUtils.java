package org.lflang.analyses.scheduler;

import java.util.List;
import java.util.Random;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;

/**
 * A utility class for static scheduler-related methods
 *
 * 
 */
public class StaticSchedulerUtils {

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
      List<DagNode> partition = dag.partitions.get(j);
      String randomColor = StaticSchedulerUtils.generateRandomColor();
      for (int i = 0; i < partition.size(); i++) {
        partition.get(i).setColor(randomColor);
        partition.get(i).setWorker(j);
      }
    }
  }
}
