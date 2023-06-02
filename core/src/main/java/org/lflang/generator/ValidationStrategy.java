package org.lflang.generator;

import java.nio.file.Path;
import org.lflang.util.LFCommand;

/**
 * A means of validating generated code.
 *
 * @author Peter Donovan
 */
public interface ValidationStrategy {

  /**
   * Return the command that produces validation output in association with {@code generatedFile},
   * or {@code null} if this strategy has no command that will successfully produce validation
   * output.
   */
  LFCommand getCommand(Path generatedFile);

  /**
   * Return a strategy for parsing the stderr of the validation command.
   *
   * @return A strategy for parsing the stderr of the validation command.
   */
  DiagnosticReporting.Strategy getErrorReportingStrategy();

  /**
   * Return a strategy for parsing the stdout of the validation command.
   *
   * @return A strategy for parsing the stdout of the validation command.
   */
  DiagnosticReporting.Strategy getOutputReportingStrategy();

  /**
   * Return whether this strategy validates all generated files, as opposed to just the given one.
   *
   * @return whether this strategy validates all generated files
   */
  boolean isFullBatch();

  /**
   * Return the priority of this. Strategies with higher priorities are more likely to be used.
   *
   * @return The priority of this.
   */
  int getPriority();
}
