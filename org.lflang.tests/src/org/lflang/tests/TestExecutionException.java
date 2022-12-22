package org.lflang.tests;

import org.lflang.tests.LFTest.Result;

/// Indicates an error during test execution
public class TestExecutionException extends Exception {

    private Throwable exception;
    private Result result;

    public TestExecutionException(String errorMessage, Result result, Throwable exception) {
        super(errorMessage);
        this.exception = exception;
        this.result = result;
    }

    public TestExecutionException(String errorMessage, Result result) {
        this(errorMessage, result, null);
    }

    public TestExecutionException(Result result) {
        this(null, result, null);
    }

    public Result getResult() {return result;}

    public Throwable getException() {return exception;}
}
