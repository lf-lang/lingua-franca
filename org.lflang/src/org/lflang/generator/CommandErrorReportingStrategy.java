package org.lflang.generator;

import java.nio.file.Path;
import java.util.Map;

import org.lflang.ErrorReporter;

/**
 * Represents a strategy for parsing the output of a
 * validator.
 */
@FunctionalInterface public interface CommandErrorReportingStrategy {
    // Note: The expected use case is to parse the output of
    //  a target language validator that is run in a separate
    //  process.
    // Note: This replaces the method GeneratorBase::reportCommandErrors,
    //  which uses the template design pattern. The justification is that
    //  1. Not all validator output fits the template. In particular, ESLint
    //  uses JSON. It is also possible that discretion should be allowed
    //  on whether the entire validation message should be sent to the
    //  error reporter vs. just the first line of it, and that is why
    //  the implementation of GeneratorBase::reportCommandErrors is unsuitable
    //  for use of GCC with an IDE. There is also the inconvenience that
    //  different tools make inconsistent choices about what goes to stderr
    //  vs. stdout.
    //  2. Using composition for code reuse in this way may lead to smaller,
    //  less tightly coupled modules than if all of the shared functionality
    //  of the code generators is inherited from GeneratorBase.
    //  FIXME: In order to avoid redundancy, it is necessary to
    //   delete either this interface or the methods
    //   GeneratorBase::reportCommandErrors and
    //   GeneratorBase::parseCommandOutput.

    /**
     * Parses the validation output and reports any errors
     * that it contains.
     * @param validationOutput any validation output
     * @param errorReporter any error reporter
     * @param map the map from generated files to CodeMaps
     */
    void report(String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map);
}
