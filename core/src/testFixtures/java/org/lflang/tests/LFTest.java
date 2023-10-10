package org.lflang.tests;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.eclipse.xtext.util.RuntimeIOException;
import org.lflang.FileConfig;
import org.lflang.generator.LFGeneratorContext;

/**
 * Information about an indexed Lingua Franca test program.
 *
 * @author Marten Lohstroh
 */
public class LFTest implements Comparable<LFTest> {

  /** The path to the test. */
  private final Path srcPath;

  /** The name of the test. */
  private final String name;

  /** The result of the test. */
  private Result result = Result.UNKNOWN;

  /** Context provided to the code generators */
  private LFGeneratorContext context;

  /** Path of the test program relative to the package root. */
  private final Path relativePath;

  /** Records compilation stdout/stderr. */
  private final ByteArrayOutputStream compilationLog = new ByteArrayOutputStream();

  /** Specialized object for capturing output streams while executing the test. */
  private final ExecutionLogger execLog = new ExecutionLogger();

  /** String builder for collecting issues encountered during test execution. */
  private final StringBuilder issues = new StringBuilder();

  /** Reference to System.out for restoring the default output. */
  private static final PrintStream out = System.out;

  /** Reference to System.err for restoring the default output. */
  private static final PrintStream err = System.err;

  private long executionTimeNanoseconds;

  /**
   * Create a new test.
   *
   * @param srcFile The path to the file of the test program.
   */
  public LFTest(Path srcFile) {
    this.srcPath = srcFile;
    this.name = FileConfig.findPackageRoot(srcFile, s -> {}).relativize(srcFile).toString();
    this.relativePath = Paths.get(name);
  }

  public FileConfig getFileConfig() {
    return context.getFileConfig();
  }

  public LFGeneratorContext getContext() {
    return context;
  }

  public Path getSrcPath() {
    return srcPath;
  }

  /** Redirect outputs for recording. */
  public void redirectOutputs() {
    System.setOut(new PrintStream(compilationLog, false, StandardCharsets.UTF_8));
    System.setErr(new PrintStream(compilationLog, false, StandardCharsets.UTF_8));
  }

  /** End output redirection. */
  public static void restoreOutputs() {
    System.out.flush();
    System.err.flush();
    System.setOut(out);
    System.setErr(err);
  }

  /**
   * Comparison implementation to allow for tests to be sorted (e.g., when added to a tree set)
   * based on their path (relative to the root of the test directory).
   */
  public int compareTo(LFTest t) {
    return this.relativePath.compareTo(t.relativePath);
  }

  /**
   * Return true if the given object is an LFTest instance with a name identical to this test.
   *
   * @param o The object to test for equality with respect to this one.
   * @return True if the given object is equal to this one, false otherwise.
   */
  @Override
  public boolean equals(Object o) {
    return o instanceof LFTest && ((LFTest) o).name.equals(this.name);
  }

  /**
   * Return a string representing the name of this test.
   *
   * @return The name of this test.
   */
  @Override
  public String toString() {
    return this.name;
  }

  /**
   * Identify tests uniquely on the basis of their name.
   *
   * @return The hash code of the name of this test.
   */
  @Override
  public int hashCode() {
    return this.name.hashCode();
  }

  /**
   * Report whether this test has failed.
   *
   * @return True if the test has failed, false otherwise.
   */
  public boolean hasFailed() {
    return result != Result.TEST_PASS;
  }

  public boolean hasPassed() {
    return result == Result.TEST_PASS;
  }

  /** Print a report of all the collected errors. */
  public void reportErrors() {
    if (this.hasFailed()) {
      System.out.println(
          "+---------------------------------------------------------------------------+");
      System.out.println(
          "Failed: "
              + this
              + String.format(" in %.2f seconds%n", getExecutionTimeNanoseconds() / 1.0e9));
      System.out.println(
          "-----------------------------------------------------------------------------");
      System.out.println("Reason: " + this.result.message);
      printIfNotEmpty("Reported issues", this.issues.toString());
      printIfNotEmpty("Compilation output", this.compilationLog.toString(StandardCharsets.UTF_8));
      printIfNotEmpty("Execution output", this.execLog.toString());
      System.out.println(
          "+---------------------------------------------------------------------------+");
    }
  }

  public void handleTestError(TestError e) {
    result = e.getResult();
    if (e.getMessage() != null) {
      issues.append(e.getMessage());
    }
    if (e.causeIsException()) {
      issues.append(System.lineSeparator());
      issues.append(e.getOriginalStackTrace());
    }
  }

  public void markPassed() {
    result = Result.TEST_PASS;
    // clear the execution log to free memory
    execLog.clear();
  }

  void configure(LFGeneratorContext context) {
    this.context = context;
  }

  /**
   * Print the message to the system output, but only if the message is not empty.
   *
   * @param header Header for the message to be printed.
   * @param message The log message to print.
   */
  private static void printIfNotEmpty(String header, String message) {
    if (!message.isEmpty()) {
      System.out.println(header + ":");
      System.out.println(message);
    }
  }

  /** Enumeration of test outcomes. */
  public enum Result {
    UNKNOWN("No information available."),
    CONFIG_FAIL("Could not apply configuration."),
    PARSE_FAIL("Unable to parse test."),
    VALIDATE_FAIL("Unable to validate test."),
    CODE_GEN_FAIL("Error while generating code for test."),
    NO_EXEC_FAIL("Did not execute test."),
    TEST_FAIL("Test did not pass."),
    TEST_EXCEPTION("Test exited with an exception."),
    TEST_TIMEOUT("Test timed out."),
    TEST_PASS("Test passed.");

    /** Description of the outcome. */
    public final String message;

    /**
     * Private constructor.
     *
     * @param message Description of the test outcome.
     */
    Result(String message) {
      this.message = message;
    }
  }

  /**
   * Inner class for capturing streams during execution of a test, capable of recording output
   * streams up until the moment that a test is interrupted upon timing out.
   *
   * @author Marten Lohstroh
   */
  public static final class ExecutionLogger {

    /**
     * String buffer used to record the standard output and error streams from the input process.
     */
    StringBuffer buffer = new StringBuffer();

    /**
     * Return a thread responsible for recording the standard output stream of the given process. A
     * separate thread is used so that the activity can be preempted.
     */
    public Thread recordStdOut(Process process) {
      return recordStream(buffer, process.getInputStream());
    }

    /**
     * Return a thread responsible for recording the error stream of the given process. A separate
     * thread is used so that the activity can be preempted.
     */
    public Thread recordStdErr(Process process) {
      return recordStream(buffer, process.getErrorStream());
    }

    /**
     * Return a thread responsible for recording the given stream.
     *
     * @param builder The builder to append to.
     * @param inputStream The stream to read from.
     */
    private Thread recordStream(StringBuffer builder, InputStream inputStream) {
      return new Thread(
          () -> {
            try (Reader reader = new InputStreamReader(inputStream, StandardCharsets.UTF_8)) {
              int len;
              char[] buf = new char[1024];
              while ((len = reader.read(buf)) > 0) {
                builder.append(buf, 0, len);
                // If the buffer gets too large, then we delete the first half of the buffer
                // and trim it down in size. It is important to decide what "too large" means.
                // Here we take 1/4 of the total memory available to the Java runtime as a rule of
                // thumb.
                if (builder.length() > Runtime.getRuntime().totalMemory() / 4) {
                  builder.delete(0, builder.length() / 2);
                  builder.insert(0, "[earlier messages were removed to free up memory]\n");
                  builder.trimToSize();
                }
              }
            } catch (IOException e) {
              throw new RuntimeIOException(e);
            }
          });
    }

    @Override
    public String toString() {
      return buffer.toString();
    }

    public void clear() {
      buffer = null;
    }
  }

  /**
   * Return a thread responsible for recording the standard output stream of the given process. A
   * separate thread is used so that the activity can be preempted.
   */
  public Thread recordStdOut(Process process) {
    return execLog.recordStdOut(process);
  }

  /**
   * Return a thread responsible for recording the error stream of the given process. A separate
   * thread is used so that the activity can be preempted.
   */
  public Thread recordStdErr(Process process) {
    return execLog.recordStdErr(process);
  }

  /** Record the execution time of this test in nanoseconds. */
  public void setExecutionTimeNanoseconds(long time) {
    assert executionTimeNanoseconds == 0; // it should only be set once
    executionTimeNanoseconds = time;
  }

  /** Return the execution time of this test in nanoseconds. */
  public long getExecutionTimeNanoseconds() {
    return executionTimeNanoseconds;
  }
}
