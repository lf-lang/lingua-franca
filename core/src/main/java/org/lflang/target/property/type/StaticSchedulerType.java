package org.lflang.target.property.type;

import org.lflang.target.property.type.StaticSchedulerType.StaticScheduler;

public class StaticSchedulerType extends OptionsType<StaticScheduler> {

  @Override
  protected Class<StaticScheduler> enumClass() {
    return StaticScheduler.class;
  }

  /**
   * Supported schedulers.
   *
   * 
   */
  public enum StaticScheduler {
    LB,
    EGS,
    MOCASIN;

    public static StaticScheduler getDefault() {
      return LB;
    }
  }
}
