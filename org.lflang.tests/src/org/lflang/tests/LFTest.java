package org.lflang.tests;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.generator.StandaloneContext;

/**
 * Information about an indexed Lingua Franca test program.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public class LFTest implements Comparable<LFTest> {
    
    /**
     * Inner class for capturing streams during execution of a test, capable of
     * recording output streams up until the moment that a test is interrupted
     * upon timing out.
     * 
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public static class ExecutionLogger {

        /**
         * String buffer used to record the standard output stream.
         */
        final StringBuffer std = new StringBuffer();

        /**
         * String builder used to record the standard error stream.
         */
        final StringBuffer err = new StringBuffer();

        /**
         * Return a thread responsible for recording the standard output stream
         * of the given process.
         * A separate thread is used so that the activity can preempted.
         */
        public Thread recordStdOut(Process process) {
            return recordStream(std, process.getInputStream());
        }
        
        /**
         * Return a thread responsible for recording the error stream of the
         * given process.
         * A separate thread is used so that the activity can preempted.
         */
        public Thread recordStdErr(Process process) {
            return recordStream(err, process.getErrorStream());
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
                } catch (Exception e) {
                    builder.append("[truncated...]\n");
                }
            });
            t.start();
            return t;
        }
    }
    
    /**
     * The path to the test.
     */
    public final Path srcFile;

    /**
     * The name of the test.
     */
    public final String name;
    
    /**
     * The result of the test.
     * @see Result
     */
    public Result result = Result.UNKNOWN;
    
    /**
     * Object used to determine where the code generator puts files.
     */
    public FileConfig fileConfig;
    
    /**
     * Path of the test program relative to the the package root.
     */
    private final Path relativePath;

    /**
     * Stream object for capturing standard output.
     */
    public ByteArrayOutputStream out = new ByteArrayOutputStream();

    /**
     * Stream object for capturing standard error output.
     */
    public ByteArrayOutputStream err = new ByteArrayOutputStream();

    /**
     * Specialized object for capturing output streams while executing the test.
     */
    public ExecutionLogger execLog = new ExecutionLogger();

    /**
     * String builder for collecting issues encountered during test executeion.
     */
    public StringBuilder issues = new StringBuilder();

    /**
     * The target of the test program.
     */
    public final Target target;

    /**
     * The path of the package root of the test program.
     */
    public final Path packageRoot;

    /**
     * Create a new test.
     * @param target The target of the test program.
     * @param srcFile The path to the file of the test program.
     * @param packageRoot The path of the package root of the test program.
     */
    public LFTest(Target target, Path srcFile, Path packageRoot) {
        this.target = target;
        this.packageRoot = packageRoot;
        
        this.srcFile = srcFile;
        this.name = packageRoot.relativize(srcFile).toString();
        this.relativePath = Paths.get(name);
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
        if (o instanceof LFTest && ((LFTest) o).name.equals(this.name)) {
            return true;
        }
        return false;
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
     * Report whether or not this test has failed.
     * @return True if the test has failed, false otherwise.
     */
    public boolean hasFailed() {
        if (result == Result.TEST_PASS) {
            return false;
        }
        return true;
    }

    /**
     * Return the standalone context stored in this test's file configuration.
     *
     * @return The context for this test, to be passed to the code generator.
     */
    public StandaloneContext getContext() {
        return (StandaloneContext)this.fileConfig.context;
    }

    /**
     *
     * @return
     */
    public String reportErrors() {
        if (this.hasFailed()) {
            StringBuilder sb = new StringBuilder(System.lineSeparator());
            sb.append("+---------------------------------------------------------------------------+").append(System.lineSeparator());
            sb.append("Failed: ").append(this.name).append(System.lineSeparator());
            sb.append("-----------------------------------------------------------------------------").append(System.lineSeparator());
            sb.append("Reason: ").append(this.result.message).append(System.lineSeparator());
            appendIfNotEmpty("Reported issues", this.issues.toString(), sb);
            appendIfNotEmpty("Compilation error output", this.err.toString(), sb);
            appendIfNotEmpty("Compilation standard output", this.out.toString(), sb);
            appendIfNotEmpty("Execution error output", this.execLog.err.toString(), sb);
            appendIfNotEmpty("Execution standard output", this.execLog.std.toString(), sb);
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
        private Result(String message) {
            this.message = message;
        }
    }
}
