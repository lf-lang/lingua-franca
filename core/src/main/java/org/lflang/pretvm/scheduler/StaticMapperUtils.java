package org.lflang.pretvm.scheduler;

import java.util.List;
import java.util.Random;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.JobNode;

/** Utility methods for static mappers. */
public class StaticMapperUtils {

  /** Generate a random hex color string (e.g. "#A3F0C2"). */
  public static String generateRandomColor() {
    Random random = new Random();
    int r = random.nextInt(256);
    int g = random.nextInt(256);
    int b = random.nextInt(256);
    return String.format("#%02X%02X%02X", r, g, b);
  }

  /** Assign a random color and worker index to each node in every partition. */
  public static void assignColorsToPartitions(Dag dag) {
    for (int j = 0; j < dag.partitions.size(); j++) {
      List<JobNode> partition = dag.partitions.get(j);
      String randomColor = generateRandomColor();
      for (JobNode node : partition) {
        node.setColor(randomColor);
        node.setWorker(j);
      }
    }
  }
}
