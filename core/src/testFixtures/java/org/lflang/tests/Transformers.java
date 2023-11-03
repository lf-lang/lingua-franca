package org.lflang.tests;

import org.eclipse.emf.ecore.resource.Resource;

public class Transformers {

  /** Function to adapts a given resource. */
  @FunctionalInterface
  public interface Transformer {

    /**
     * Apply a side effect to the given test case to change its resource. Return true if
     * transformation succeeded, false otherwise.
     */
    boolean transform(Resource resource);
  }

  public static boolean noChanges(Resource resource) {
    return true;
  }
}
