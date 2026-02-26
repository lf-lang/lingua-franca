package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;

// import org.jetbrains.annotations.Nullable;

/**
 * An exception that occurred during code generation. May also wrap another exception.
 *
 * @ingroup Generator
 */
public class GenerationException
    extends RuntimeException { // note that this is an unchecked exception.

  /* @Nullable */
  private final EObject location;

  public GenerationException(String message) {
    this(null, message, null);
  }

  public GenerationException(/* @Nullable */ EObject location, String message) {
    this(location, message, null);
  }

  public GenerationException(String message, Throwable cause) {
    this(null, message, cause);
  }

  public GenerationException(/* @Nullable */ EObject location, String message, Throwable cause) {
    super(message, cause);
    this.location = location;
  }

  public GenerationException(Throwable cause) {
    this(null, null, cause);
  }

  /* @Nullable */
  public EObject getLocation() {
    return location;
  }
}
