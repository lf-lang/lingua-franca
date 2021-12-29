package org.lflang.generator;

import java.nio.file.Path;
import java.util.Map;

import org.lflang.ErrorReporter;

/**
 * Represents a strategy for parsing the output of a
 * validator.
 */
@FunctionalInterface public interface CommandErrorReportingStrategy {
    /**
     * Parses the validation output and reports any errors
     * that it contains.
     * @param validationOutput any validation output
     * @param errorReporter any error reporter
     * @param map the map from generated files to CodeMaps
     */
    void report(String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map);
}
