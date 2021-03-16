package org.icyphy.tests;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Properties;
import java.util.regex.Pattern;

import org.icyphy.FileConfig;
import org.icyphy.Target;
import org.icyphy.tests.runtime.TestBase;

/**
 * Information about an indexed Lingua Franca test program.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public class LFTest implements Comparable<LFTest> {
    
    /**
     * Inner class for capturing streams during a particular phase of testing,
     * such as compilation or execution.
     * 
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public class TestPhase {
       
        /**
         * String builder used to record the standard output stream.
         */
        StringBuilder std = new StringBuilder("");
        
        /**
         * String builder used to record the standard error stream.
         */
        StringBuilder err = new StringBuilder("");
        
        /**
         * Return a thread responsible for recording the given stream that
         * should be attached to standard out.
         * A separate thread is used so that the activity can preempted.
         * @param stream The stream to record.
         * @return A thread that will record the given stream.
         */
        public Thread recordStdOut(InputStream stream) { // FIXME: change argument to Process.
            return recordStream(std, stream);
        }
        
        /**
         * Return a thread responsible for recording the given stream that
         * should be attached to standard error.
         * A separate thread is used so that the activity can preempted.
         * @param stream The stream to record.
         * @return A thread that will record the given stream.
         */
        public Thread recordStdErr(InputStream stream) {
            return recordStream(err, stream);
        }
        
        /**
         * 
         * @param builder
         * @param inputStream
         * @return
         */
        private Thread recordStream(StringBuilder builder, InputStream inputStream) {
            Thread t = new Thread(() -> {
                try {
                    char[] buf = new char[1024];
                    int len;
                    BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
                    while ((len = reader.read(buf)) > 0) {
                        builder.append(buf, 0, len);
                    }
                    reader.close();
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
    public final Path path;
    
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
     * 
     */
    private final Path relativePath;
    
    public ByteArrayOutputStream out = new ByteArrayOutputStream();
    
    public ByteArrayOutputStream err = new ByteArrayOutputStream();
    
    public TestPhase exec = new TestPhase();
    
    public TestPhase compile = new TestPhase();
    
    public StringBuilder issues = new StringBuilder();
    
    public Properties properties = new Properties();
    
    public final Target target;
    
    public LFTest(Target target, Path path) {
        this.target = target; // FIXME: do we need the target?
        this.path = path;
        this.name = normalize(target, path);
        this.relativePath = Paths.get(name);
    }
    
    public int compareTo(LFTest t) {
        return this.relativePath.compareTo(t.relativePath);
    }
    
    private static String normalize(Target target, Path path) {
        return path.toString().replaceFirst(Pattern.quote(TestRegistry.LF_TEST_PATH + target + File.separator), "");
    }
    
    @Override
    public boolean equals(Object o) {
        if (o instanceof LFTest && ((LFTest) o).name.equals(this.name)) {
            return true;
        }
        return false;
    }
    
    @Override
    public String toString() {
        return this.name;
    }
    
    @Override
    public int hashCode() {
        return this.name.hashCode();
    }
    
    public boolean hasFailed() {
        if (result == Result.TEST_PASS) {
            return false;
        }
        return true;
    }
    
    public String reportErrors() {
        if (this.hasFailed()) {
            StringBuffer sb = new StringBuffer();
            sb.append("\n+---------------------------------------------------------------------------+\n");
            sb.append("Failed: " + this.name + "\n");
            sb.append("-----------------------------------------------------------------------------\n");
            sb.append("Reason: " + this.result.reason + TestBase.NEW_LINE);
            appendIfNotEmpty("Reported issues", this.issues.toString(), sb);
            appendIfNotEmpty("Compilation error output", this.err.toString(), sb);
            appendIfNotEmpty("Compilation standard output", this.out.toString(), sb);
            appendIfNotEmpty("Execution error output", this.exec.err.toString(), sb);
            appendIfNotEmpty("Execution standard output", this.exec.std.toString(), sb);
            sb.append("+---------------------------------------------------------------------------+\n");
        return sb.toString();
        } else {
            return "";
        }
    }

    public void appendIfNotEmpty(String description, String log, StringBuffer buffer) {
        if (!log.isEmpty()) {
            buffer.append(description + ":" + TestBase.NEW_LINE);
            buffer.append(log);
        }
    }
    
    public enum Result {
        UNKNOWN("No information available."),
        CONFIG_FAIL("Unable to succesfully apply configuration."),
        NO_MAIN_FAIL("No main reactor."),
        PARSE_FAIL("Unable to successfully parse test."),
        VALIDATE_FAIL("Unable to successfully validate test."),
        CODE_GEN_FAIL("Unable to successfully generate code for test."),
        BUILD_FAIL("Unable to successfully compile generated code for test."),
        NO_EXEC_FAIL("Unable to successfully execute test."),
        TEST_FAIL("Test did not pass."),
        TEST_TIMEOUT("Test timed out."),
        TEST_PASS("Test passed.");

        public final String reason;
        
        private Result(String message) {
            this.reason = message;
        }
    }
}
