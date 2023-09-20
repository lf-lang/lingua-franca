package org.lflang.tests;

import org.lflang.tests.LFTest.Result;

/// Indicates an error during test execution
public class TestError extends Exception {

  private final String stackTrace;
  private final Result result;

  public TestError(String errorMessage, Result result, Throwable exception) {
    super(errorMessage);
    assert result != null;

    this.stackTrace = exception == null ? null : TestBase.stackTraceToString(exception);
    this.result = result;
  }

  public TestError(String errorMessage, Result result) {
    this(errorMessage, result, null);
  }

  public TestError(Result result) {
    this(null, result, null);
  }

  public Result getResult() {
    return result;
  }

  /// Return true, if the TestError instance was created based on an exception.
  public boolean causeIsException() {
    return stackTrace != null;
  }

  /// Retrieve the stack trace of the exception that caused the test error.
  public String getOriginalStackTrace() {
    return stackTrace;
  }
}
