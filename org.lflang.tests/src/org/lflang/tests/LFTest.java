package org.lflang.tests;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.Reader;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.generator.LFGeneratorContext;

/**
 * Information about an indexed Lingua Franca test program.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public class LFTest implements Comparable<LFTest> {

    /** The path to the test. */
    public final Path srcFile;

    /** The name of the test. */
    public final String name;

    /** The result of the test. */
    public Result result = Result.UNKNOWN;
    
    /** The exit code of the test. **/
    public String exitValue = "?";

    /** Object used to determine where the code generator puts files. */
    public FileConfig fileConfig;

    /** Context provided to the code generators */
    public LFGeneratorContext context;

    /** Path of the test program relative to the package root. */
    private final Path relativePath;

    /** Records compilation stdout/stderr. */
    private final ByteArrayOutputStream compilationLog = new ByteArrayOutputStream();

    /** Specialized object for capturing output streams while executing the test. */
    public final ExecutionLogger execLog = new ExecutionLogger();

    /** String builder for collecting issues encountered during test execution. */
    public final StringBuilder issues = new StringBuilder();

    /** The target of the test program. */
    public final Target target;

    /**
     * Create a new test.
     *
     * @param target The target of the test program.
     * @param srcFile The path to the file of the test program.
     */
    public LFTest(Target target, Path srcFile) {
        this.target = target;
        this.srcFile = srcFile;
        this.name = FileConfig.findPackageRoot(srcFile, s -> {}).relativize(srcFile).toString();
        this.relativePath = Paths.get(name);
    }

    /** Stream object for capturing standard and error output. */
    public OutputStream getOutputStream() {
        return compilationLog;
    }

    /**
     * Comparison implementation to allow for tests to be sorted (e.g., when added to a
     * tree set) based on their path (relative to the root of the test directory).
     */
    public int compareTo(LFTest t) {
        return this.relativePath.compareTo(t.relativePath);
    }

    /**
     * Return true if the given object is an LFTest instance with a name identical to this test.
     * @param o The object to test for equality with respect to this one.
     * @return True if the given object is equal to this one, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        return o instanceof LFTest && ((LFTest) o).name.equals(this.name);
    }

    /**
     * Return a string representing the name of this test.
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
     * @return True if the test has failed, false otherwise.
     */
    public boolean hasFailed() {
        return result != Result.TEST_PASS;
    }

    /**
     * Compile a string that contains all collected errors and return it.
     * @return A string that contains all collected errors.
     */
    public String reportErrors() {
        if (this.hasFailed()) {
            StringBuilder sb = new StringBuilder(System.lineSeparator());
            sb.append("+---------------------------------------------------------------------------+").append(System.lineSeparator());
            sb.append("Failed: ").append(this.name).append(System.lineSeparator());
            sb.append("-----------------------------------------------------------------------------").append(System.lineSeparator());
            sb.append("Reason: ").append(this.result.message).append(" Exit code: ").append(this.exitValue).append(System.lineSeparator());
            appendIfNotEmpty("Reported issues", this.issues.toString(), sb);
            appendIfNotEmpty("Compilation output", this.compilationLog.toString(), sb);
            appendIfNotEmpty("Execution output", this.execLog.toString(), sb);
            sb.append("+---------------------------------------------------------------------------+\n");
        return sb.toString();
        } else {
            return "";
        }
    }

    /**
     * Append the given header and message to the log, but only if the message is not empty.
     *
     * @param header Header for the message to append to the log.
     * @param message The log message to add.
     * @param log The log so far.
     */
    private static void appendIfNotEmpty(String header, String message, StringBuilder log) {
        if (!message.isEmpty()) {
            log.append(header).append(":").append(System.lineSeparator());
            log.append(message).append(System.lineSeparator());
        }
    }

    /**
     * Enumeration of test outcomes.
     */
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

        /**
         * Description of the outcome.
         */
        public final String message;

        /**
         * Private constructor.
         * @param message Description of the test outcome.
         */
        Result(String message) {
            this.message = message;
        }
    }


    /**
     * Inner class for capturing streams during execution of a test, capable of
     * recording output streams up until the moment that a test is interrupted
     * upon timing out.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public static final class ExecutionLogger {

        /**
         * String buffer used to record the standard output and error
         * streams from the input process.
         */
        final StringBuffer buffer = new StringBuffer();

        /**
         * Return a thread responsible for recording the standard output stream
         * of the given process.
         * A separate thread is used so that the activity can be preempted.
         */
        public Thread recordStdOut(Process process) {
            return recordStream(buffer, process.getInputStream());
        }

        /**
         * Return a thread responsible for recording the error stream of the
         * given process.
         * A separate thread is used so that the activity can be preempted.
         */
        public Thread recordStdErr(Process process) {
            return recordStream(buffer, process.getErrorStream());
        }

        /**
         * Return a thread responsible for recording the given stream.
         *
         * @param builder     The builder to append to.
         * @param inputStream The stream to read from.
         */
        private Thread recordStream(StringBuffer builder, InputStream inputStream) {
            Thread t = new Thread(() -> {
                try (Reader reader = new InputStreamReader(inputStream)) {
                    int len;
                    char[] buf = new char[1024];
                    while ((len = reader.read(buf)) > 0) {
                        builder.append(buf, 0, len);
                    }
                } catch (IOException e) {
                    throw new RuntimeException("While reading from a stream, an I/O exception occurred:\n" + e);
                }
            });
            t.start();
            return t;
        }

        @Override
        public String toString() {
            return buffer.toString();
        }
    }
}
