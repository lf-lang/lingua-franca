package org.icyphy.tests;

import static java.nio.file.FileVisitResult.CONTINUE;
import static java.nio.file.FileVisitResult.SKIP_SUBTREE;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.TreeSet;
import java.util.regex.Pattern;

import org.icyphy.Target;
import org.icyphy.tests.runtime.TestBase;

/**
 * A registry to retrieve tests from, organized by target and category.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 */
public class TestRegistry {
    
    /**
     * List of directories that should be skipped when indexing test files. Any
     * test file that has a directory in its path that matches an entry in this
     * array will not be discovered.
     */
    public static final String[] IGNORED_DIRECTORIES = new String[] {"bin", "build", "include", "lib", "share", "src-gen"};
    
    /**
     * The name of our test package.
     */
    public static final String TEST_PACKAGE_NAME = "org.icyphy.linguafranca.tests";
    
    /**
     * The location in which to find the tests.
     */
    public static final String LF_TEST_PATH = new File("").getAbsolutePath()
            .split("xtext" + Pattern.quote(File.separator) + TEST_PACKAGE_NAME)[0] + "test" +
            File.separator;

    /**
     * Registry that maps targets and test categories to a list of paths.
     */
    protected final static List<List<Set<LFTest>>> registry = new ArrayList<List<Set<LFTest>>>();
    
    /**
     * Maps each target to a set of target categories it supports.
     */
    protected final static Map<Target, Set<TestCategory>> support = new HashMap<Target, Set<TestCategory>>();
    
    /**
     * Maps each test category to a set of tests that is the union of tests in that category across all targets.
     */
    protected final static Map<TestCategory, Set<LFTest>> allTargets = new HashMap<TestCategory, Set<LFTest>>();
    
    public static StringBuffer messages = new StringBuffer();

    /**
     * Enumeration of test categories, used to map tests to categories. The
     * nearest containing directory that matches any of the categories will
     * determine the category that the test is mapped to. Matching is case
     * insensitive.
     * 
     * For example, the following files will all map to THREADED:
     * - C/threaded/Foo.lf
     * - C/THREADED/Foo.lf 
     * - C/Threaded/Foo.lf
     * - C/foo/threaded/Bar.lf 
     * - C/foo/bar/threaded/Threaded.lf 
     * - C/federated/threaded/bar.lf
     * but the following will not: 
     * - C/Foo.lf (maps to COMMON)
     * - C/Threaded.lf (maps to COMMON)
     * - C/threaded/federated/foo.lf (maps to FEDERATED)
     * 
     * @author Marten Lohstroh <marten@berkeley.edu>
     */
    public enum TestCategory {
        CONCURRENT(true), GENERIC(true), MULTIPORT(true), TARGET(false), FEDERATED(true);
        
        public final boolean isCommon;
        
        private TestCategory(boolean isCommon) {
            this.isCommon = isCommon;
        }
        
        public String getHeader() {
            StringBuffer sb = new StringBuffer();
            sb.append(TestBase.THICK_LINE);
            sb.append("Category: " + this.name());
            return sb.toString();
        }
    }
    
    static {
        // Prepare for the collection of tests per category.
        for (TestCategory t: TestCategory.values()) {
            allTargets.put(t, new TreeSet<LFTest>());
        }
        // Populate the registry.
        for (Target target : Target.values()) {
            // Initialize the lists.
            ArrayList<Set<LFTest>> categories = new ArrayList<Set<LFTest>>();
            for (int i = 0; i < TestCategory.values().length; i++) {
                categories.add(new LinkedHashSet<LFTest>());
            }
            registry.add(target.ordinal(), categories);
            Set<TestCategory> supported = new HashSet<TestCategory>();
            support.put(target, supported);
            // Walk the tree.
            try {
                Path dir = Paths.get(LF_TEST_PATH + target);
                if (Files.exists(dir)) {
                    // A test directory exist. Assume support for generic tests.
                    supported.add(TestCategory.GENERIC);
                    Files.walkFileTree(dir, new TestFileVisitor(target));
                } else {
                    messages.append("WARNING: No test directory target " + target + "\n");
                }
                
            } catch (IOException e) {
                System.err.println(
                        "Error while indexing tests for target " + target);
                e.printStackTrace();
            }
            // Record the tests for later use when reporting coverage.
            Arrays.asList(TestCategory.values()).forEach(
                    c -> allTargets.get(c).addAll(getTests(target, c)));
        }
    }
    
    public static Set<LFTest> getTests(Target target, TestCategory category) {
        Set<LFTest> sorted = new TreeSet<LFTest>();
        registry.get(target.ordinal()).get(category.ordinal()).forEach(
                test -> sorted.add(new LFTest(test.target, test.path)));
        return sorted;
    }
    
    /**
     * Return all tests that belong to the given test categories.
     * @param target The target for which to look up the tests.
     * @param included The test categories to include in the returned list.
     * @return
     */
    public static List<LFTest> getTestsIncluding(Target target,
            List<TestCategory> included) {
        List<LFTest> tests = new LinkedList<LFTest>();
        for (TestCategory category : included) {
            tests.addAll(getTests(target, category));
        }
        return tests;
    }
    

    public static String getCoverageReport(Target target, TestCategory category) {
        StringBuilder s = new StringBuilder();
        Set<LFTest> own = getTests(target, category);
        if (support.get(target).contains(category)) {
            Set<LFTest> all = allTargets.get(category);
            s.append(TestBase.THIN_LINE);
            s.append("Covered: " + own.size() + "/" + all.size() + "\n");
            s.append(TestBase.THIN_LINE);
            int missing = all.size() - own.size();
            if (missing > 0) {
                all.stream().filter(test -> !own.contains(test))
                        //.map(test -> test.toString())
                        .forEach(test -> s.append("Missing: " + test.toString() + "\n"));
                        //.collect(Collectors.joining(", ", "[", "]")) + "\n");
            }
        } else {
            s.append("\n" + TestBase.THIN_LINE);
            s.append("Covered: " + own.size() + "/" + own.size() + "\n");
            s.append(TestBase.THIN_LINE);
        }
        
        return s.toString();
    }
    
    
    
//    /**
//     * Return all test for the given target minus those that fall in the
//     * excluded categories.
//     * 
//     * @param target The target for which to look up the tests.
//     * @param excluded The test categories to exclude from the returned list.
//     * @return
//     */
//    public static List<LFTest> getTestsExcluding(Target target,
//            List<TestCategory> excluded) {
//        List<LFTest> tests = new LinkedList<LFTest>();
//        for (TestCategory category : TestCategory.values()) {
//            if (!excluded.contains(category)) {
//                tests.addAll(
//                        registry.get(target.ordinal()).get(category.ordinal()));
//            }
//        }
//        tests.forEach(test -> test.clear());
//        return tests;
//    }
    
    /**
     * FileVisitor implementation that maintains a stack to map found tests to
     * the appropriate category and excludes directories that are listed as 
     * "ignored" from walks.
     * 
     * Specifically, when a directory is encountered that matches a category,
     * this category is pushed onto the stack. Similarly, when the DFS leaves
     * such a directory, its corresponding category is popped from the stack.
     * Any test (*.lf) file that is encountered will be mapped to the category
     * that is on top of the stack. Initially, the stack has one element that 
     * is TestCategory.COMMON, meaning that test files in the top-level test
     * directory for a given target will be mapped to that category.
     * 
     * @author Marten Lohstroh <marten@berkeley.edu>
     */
    public static class TestFileVisitor extends SimpleFileVisitor<Path> {

        /**
         * The stack of which the top element indicates the "current" category.
         */
        protected Stack<TestCategory> stack = new Stack<TestCategory>();

        /**
         * The target that all encountered tests belong to.
         */
        protected Target target;
        
        
        /**
         * Create a new file visitor based on a given target.
         * @param target The target that all encountered tests belong to.
         */
        public TestFileVisitor(Target target) {
            stack.push(TestCategory.GENERIC);
            this.target = target;
        }
        
        /**
         * Push categories onto the stack as appropriate and skip directories
         * that should be ignored.
         */
        @Override
        public FileVisitResult preVisitDirectory(Path dir,
                BasicFileAttributes attrs) {
            for (String ignored : IGNORED_DIRECTORIES) {
                if (dir.getFileName().toString().equalsIgnoreCase(ignored)) {
                    return SKIP_SUBTREE;
                }
            }

            for (TestCategory category : TestCategory.values()) {
                if (dir.getFileName().toString()
                        .equalsIgnoreCase(category.name())) {
                    support.get(target).add(category);
                    stack.push(category);
                }
            }
            return CONTINUE;
        }
        
        /**
         * Pop categories from the stack as appropriate.
         */
        @Override
        public FileVisitResult postVisitDirectory(Path dir, IOException exc)
                throws IOException {
            for (TestCategory category : TestCategory.values()) {
                if (dir.getFileName().toString()
                        .equalsIgnoreCase(category.name())) {
                    this.stack.pop();
                }
            }
            return CONTINUE;
        }
        
        /**
         * Add test files to the registry if they end with ".lf".
         */
        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attr) {
            if (attr.isRegularFile() && file.toString().endsWith(".lf")) {
                registry.get(this.target.ordinal())
                        .get(this.stack.peek().ordinal()).add(new LFTest(target, file));
            }
            return CONTINUE;
        }
    }
}
