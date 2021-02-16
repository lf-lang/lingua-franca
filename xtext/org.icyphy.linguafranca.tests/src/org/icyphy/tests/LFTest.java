package org.icyphy.tests;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Properties;

import org.eclipse.emf.ecore.resource.Resource;
import org.icyphy.Target;

public class LFTest implements Comparable<LFTest> {
    
    public final Path path;
    public final String name;
    
    public Result result = Result.UNKNOWN;
    
    public Resource resource;
    
    private final Path relativePath;
    
    public ByteArrayOutputStream out = new ByteArrayOutputStream();
    
    public boolean execute = true;
    
    public ByteArrayOutputStream err = new ByteArrayOutputStream();
    
    public String compilationIssues = "";
    
    public String executionErrors = "";
    
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
        return path.toString().replaceFirst(TestRegistry.LF_TEST_PATH + target + File.separator, "");
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
    
    public void clear() {
        this.compilationIssues = "";
        this.executionErrors = "";
        this.properties.clear();
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
            sb.append("Reason: " + this.result.reason + "\n"); // FIXME: Use System.getProperty("line.separator") or StringFormat to be platform-independent
            if (this.result.compareTo(Result.VALIDATE_FAIL) >= 0) { // FIXME: Just print stuff when the strings are not empty.
                sb.append("Std Err:\n");
                sb.append(this.err.toString());
                sb.append("Std Out:\n");
                sb.append(this.out.toString());
            }
            sb.append("+---------------------------------------------------------------------------+\n");
        return sb.toString();
        } else {
            return "";
        }
    }

//    public String getExplanation() {
//        StringBuilder sb = new StringBuilder()
//        if (this.hasFailed()) {
//            sb.append("\n" + TestBase.DIVIDER)
//            println("Failed: " + test.name)
//            print(THIN_LINE)
//            println("Reason: " + test.result.message)
//            print(DIVIDER)
//        }
//    }
//    
    
    public enum Result {
        UNKNOWN("No information available."),
        CONFIG_FAIL("Unable to succesfully apply configuration."),
        NO_MAIN_FAIL("No main reactor."),
        PARSE_FAIL("Unable to successfully parse test."),
        VALIDATE_FAIL("Unable to successfully validate test."),
        CODE_GEN_FAIL("Unable to successfully generate code for test."),
        BUILD_FAIL("Unable to successfully compile generated code for test."),
        TEST_FAIL("Test did not pass."),
        TEST_TIMEOUT("Test timed out."),
        TEST_PASS("Test passed.");

        public final String reason;
        
        private Result(String message) {
            this.reason = message;
        }

    }
}
