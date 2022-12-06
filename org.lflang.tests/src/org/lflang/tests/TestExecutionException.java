package org.lflang.tests;

/// Indicates an error during test execution
public class TestExecutionException extends Exception {
    public TestExecutionException(String errorMessage) {
        super(errorMessage);
    }
}
