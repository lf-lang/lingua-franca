package org.icyphy.tests;

import static java.nio.file.FileVisitResult.CONTINUE;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.TreeSet;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.Resource.Diagnostic;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.icyphy.LinguaFrancaStandaloneSetup;
import org.icyphy.Target;
import org.icyphy.generator.Main;
import org.icyphy.linguaFranca.Reactor;
import org.icyphy.tests.LFTest.Result;
import org.icyphy.tests.runtime.TestBase;

import com.google.common.collect.Iterables;

/**
 * A registry to retrieve tests from, organized by target and category.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 */
public class TestRegistry {
    
    static class TestMap {
        /**
         * Registry that maps targets to maps from categories to sets of tests. 
         */
        protected final Map<Target,
                Map<TestCategory, Set<LFTest>>> map = new HashMap<Target,
                        Map<TestCategory, Set<LFTest>>>();

        public TestMap() {
            for (Target target : Target.values()) {
                Map<TestCategory, Set<LFTest>> categories = new HashMap<TestCategory, Set<LFTest>>();
                for (TestCategory cat : TestCategory.values()) {
                    categories.put(cat, new TreeSet<LFTest>());
                }
                map.put(target, categories);
            }
        }
        
        public Set<LFTest> getTests(Target t, TestCategory c) {
            return this.map.get(t).get(c);
        }
        
    }
    
    /**
     * The location in which to find the tests.
     */
    public static final Path LF_TEST_PATH = Paths.get(new File("").getAbsolutePath()).getParent().getParent().resolve("test");

    /**
     * Registry that maps targets to maps from categories to sets of tests. 
     */
    protected final static TestMap registered = new TestMap();

    protected final static TestMap ignored = new TestMap();

    /**
     * Maps each target to a set of target categories it supports.
     */
    protected final static Map<Target, Set<TestCategory>> support = new HashMap<Target, Set<TestCategory>>();
    
    /**
     * Maps each test category to a set of tests that is the union of tests in
     * that category across all targets.
     */
    protected final static Map<TestCategory, Set<LFTest>> allTargets = new HashMap<TestCategory, Set<LFTest>>();
    
    /**
     * Messages to print when done indexing.
     */
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
        
        /**
         * Whether or not we should compare coverage against other targets.
         */
        public final boolean isCommon;
        
        /**
         * Create a new test category.
         * @param isCommon
         */
        private TestCategory(boolean isCommon) {
            this.isCommon = isCommon;
        }
        
        /**
         * Return a header associated with the category.
         * 
         * @return A header to print in the test report.
         */
        public String getHeader() {
            StringBuffer sb = new StringBuffer();
            sb.append(TestBase.THICK_LINE);
            sb.append("Category: " + this.name());
            return sb.toString();
        }
    }
    
    static {
        ResourceSet rs = new LinguaFrancaStandaloneSetup()
                .createInjectorAndDoEMFRegistration()
                .<Main>getInstance(Main.class).getResourceSet();

        // Prepare for the collection of tests per category.
        for (TestCategory t: TestCategory.values()) {
            allTargets.put(t, new TreeSet<LFTest>());
        }
        // Populate the registry.
        for (Target target : Target.values()) {
            // Initialize the lists.
            Set<TestCategory> supported = new HashSet<TestCategory>();
            support.put(target, supported);
            // Walk the tree.
            try {
                Path dir = LF_TEST_PATH.resolve(target.toString());
                if (Files.exists(dir)) {
                    // A test directory exist. Assume support for generic tests.
                    supported.add(TestCategory.GENERIC);
                    Files.walkFileTree(dir, new TestFileVisitor(rs, target));
                } else {
                    messages.append("WARNING: No test directory for target " + target + "\n");
                }
                
            } catch (IOException e) {
                System.err.println(
                        "Error while indexing tests for target " + target);
                e.printStackTrace();
            }
            // Record the tests for later use when reporting coverage.
            Arrays.asList(TestCategory.values()).forEach(
                    c -> allTargets.get(c).addAll(getRegisteredTests(target, c)));
        }
    }
    
    /**
     * Return the tests that were indexed for a given target and category.
     * 
     * @param target
     * @param category
     * @return
     */
    public static Set<LFTest> getRegisteredTests(Target target, TestCategory category) {
        return registered.getTests(target, category);
    }
    
    /**
     * Return the test that were found but not indexed because they did not
     * have a main reactor.
     * @param target
     * @param category
     * @return
     */
    public static Set<LFTest> getIgnoredTests(Target target, TestCategory category) {
        return ignored.getTests(target, category);
    }
    
    /**
     * 
     * @param target
     * @param category
     * @return
     */
    public static String getCoverageReport(Target target, TestCategory category) {
        StringBuilder s = new StringBuilder();
        Set<LFTest> ignored = TestRegistry.getIgnoredTests(target, category);
        s.append(TestBase.THIN_LINE);
        s.append("Ignored: " + ignored.size() + "\n");
        s.append(TestBase.THIN_LINE);
        
        for (LFTest test : ignored) {
            s.append("No main reactor in: " + test.name + "\n");
        }
        
        Set<LFTest> own = getRegisteredTests(target, category);
        if (support.get(target).contains(category)) {
            Set<LFTest> all = allTargets.get(category);
            s.append("\n" + TestBase.THIN_LINE);
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
        
        protected ResourceSet rs;
        
        /**
         * Create a new file visitor based on a given target.
         * @param target The target that all encountered tests belong to.
         */
        public TestFileVisitor(ResourceSet rs, Target target) {
            stack.push(TestCategory.GENERIC);
            this.rs = rs;
            this.target = target;
        }
        
        /**
         * Push categories onto the stack as appropriate and skip directories
         * that should be ignored.
         */
        @Override
        public FileVisitResult preVisitDirectory(Path dir,
                BasicFileAttributes attrs) {

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
         * Add test files to the registry if they end with ".lf", but only if they have a main reactor.
         */
        @Override
        public FileVisitResult visitFile(Path path, BasicFileAttributes attr) {
            if (attr.isRegularFile() && path.toString().endsWith(".lf")) {
                // Parse the file. If this is unsuccessful, add the test and
                // report that it didn't compile.
                Resource r = rs.getResource(
                        URI.createFileURI(path.toFile().getAbsolutePath()),
                        true);
                LFTest test = new LFTest(r, target, path);
                EList<Diagnostic> errors = r.getErrors();
                if (!errors.isEmpty()) {
                    for (Diagnostic d : errors) {
                        test.issues.append(d.toString()); // FIXME: Not showing
                                                          // up for some reason.
                    }
                    test.result = Result.PARSE_FAIL;
                } else {
                    Iterable<Reactor> reactors = Iterables.<Reactor>filter(
                            IteratorExtensions.<EObject>toIterable(
                                    r.getAllContents()),
                            Reactor.class);
                    if (!IteratorExtensions.exists(reactors.iterator(),
                            it -> it.isMain() || it.isFederated())) {
                        // If the test compiles but doesn't have a main reactor,
                        // _do not add the file_. We assume it is a library
                        // file.
                        ignored.getTests(this.target, this.stack.peek()).add(test);
                        return CONTINUE;
                    }
                }
                registered.getTests(this.target, this.stack.peek()).add(test);
            }
            return CONTINUE;
        }
    }
}
