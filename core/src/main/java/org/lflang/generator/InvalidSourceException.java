package org.lflang.generator;

/**
 * This exception is thrown when a program fails a validity check performed by a code generator (and
 * not the validator). This should be thrown only when local control flow cannot recover, otherwise
 * using {@link GeneratorBase#messageReporter} should be preferred, in order to collect more errors
 * before failing.
 *
 * @author Clément Fournier
 * @ingroup Validation
 */
public class InvalidSourceException extends RuntimeException {

  /** Create a new instance of the exception with the given message. */
  public InvalidSourceException(String message) {
    super(message);
  }
}
