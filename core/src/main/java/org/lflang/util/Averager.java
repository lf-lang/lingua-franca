package org.lflang.util;

import java.util.Arrays;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure1;

/** Average asynchronously reported numbers and do something with them. */
public class Averager {
  private final int n;
  private final int[] reports;

  /** Create an averager of reports from {@code n} processes. */
  public Averager(int n) {
    this.n = n;
    reports = new int[n];
  }

  /**
   * Receive {@code x} from process {@code id} and invoke {@code callback} on the mean of the
   * numbers most recently reported by the processes.
   */
  public synchronized void report(int id, int x, Procedure1<Integer> callback) {
    assert 0 <= id && id < n;
    reports[id] = x;
    callback.apply(Arrays.stream(reports).sum() / n);
  }
}
