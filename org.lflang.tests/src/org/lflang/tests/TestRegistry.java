package org.lflang.tests;

import static java.nio.file.FileVisitResult.CONTINUE;
import static java.nio.file.FileVisitResult.SKIP_SUBTREE;
import static org.eclipse.xtext.xbase.lib.IteratorExtensions.exists;
import static org.eclipse.xtext.xbase.lib.IteratorExtensions.filter;

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
import java.util.Iterator;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.Stack;
import java.util.TreeSet;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.Resource.Diagnostic;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;

import org.lflang.LFResourceProvider;
import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.tests.LFTest.Result;

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
                Map<TestCategory, Set<LFTest>>> map = new HashMap<>();

        /**
         * Create a new test map.
         */
        public TestMap() {
            // Populate the internal datastructures.
            for (Target target : Target.values()) {
                Map<TestCategory, Set<LFTest>> categories = new HashMap<>();
                for (TestCategory cat : TestCategory.values()) {
                    categories.put(cat, new TreeSet<>());
                }
                map.put(target, categories);
            }
        }
        
        /**
         * Return a set of tests given a target and test category.
         * @param t The target.
         * @param c The test category.
         * @return A set of tests for the given target and test category.
         */
        public Set<LFTest> getTests(Target t, TestCategory c) {
            return this.map.get(t).get(c);
        }
        
    }
    
    /**
     * List of directories that should be skipped when indexing test files. Any
     * test file that has a directory in its path that matches an entry in this
     * array will not be discovered.
     */
    public static final String[] IGNORED_DIRECTORIES = {"failing", "knownfailed", "failed"};
    
    /**
     * Path to the root of the repository.
     */
    public static final Path LF_REPO_PATH = Paths.get("").toAbsolutePath();
    
    /**
     * Path to the example directory in the repository.
     */
    public static final Path LF_EXAMPLE_PATH = LF_REPO_PATH.resolve("example");
    
    /**
     * Path to the test directory in the repository.
     */
    public static final Path LF_TEST_PATH = LF_REPO_PATH.resolve("test");

    /**
     * Internal data structure that stores registered tests. 
     */
    protected static final TestMap registered = new TestMap();

    /**
     * Internal data structure that stores ignored tests. For instance,
     * source files with no main reactor are indexed here.
     */
    protected static final TestMap ignored = new TestMap();

    /**
     * The set of found examples that that did not compile or specify a target.
     */
    protected static final Set<Path> erroneousExamples = new TreeSet<>();
    
    /**
     * A map from each test category to a set of tests that is the union of
     * all registered tests in that category across all targets.
     */
    protected static final Map<TestCategory, Set<LFTest>> allTargets = new HashMap<>();
    
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
        /** Tests about concurrent execution. */
        CONCURRENT(true),
        /** Generic tests, ie, tests that all targets are supposed to implement. */
        GENERIC(true),
        /** Tests about generics, not to confuse with {@link #GENERIC}. */
        GENERICS(true),
        /** Tests about multiports and banks of reactors. */
        MULTIPORT(true),
        /** Tests about federated execution. */
        FEDERATED(true),
        /** Tests about specific target properties. */
        PROPERTIES(true),
        /** Tests concerning modal reactors */
        MODAL_MODELS(true),

        // non-shared tests
        DOCKER(true),
        DOCKER_FEDERATED(true, "docker" + File.separator + "federated"),
        SERIALIZATION(false),
        TARGET(false),
        EXAMPLE(false),
        EXAMPLE_TEST(false);
        
        /**
         * Whether or not we should compare coverage against other targets.
         */
        public final boolean isCommon;
        public final String path;
        
        /**
         * Create a new test category.
         */
        TestCategory(boolean isCommon) {
            this.isCommon = isCommon;
            this.path = this.name().toLowerCase();
        }

        /**
         * Create a new test category.
         */
        TestCategory(boolean isCommon, String path) {
            this.isCommon = isCommon;
            this.path = path;
        }

        public String getPath() {
            return path;
        }

        /**
         * Return a header associated with the category.
         * 
         * @return A header to print in the test report.
         */
        public String getHeader() {
            return TestBase.THICK_LINE + "Category: " + this.name();
        }
    }
    
    // Static code that performs the file system traversal and discovers
    // all .lf files to be included in the registry.
    static {
        System.out.println("Indexing...");
        ResourceSet rs = new LFStandaloneSetup()
            .createInjectorAndDoEMFRegistration()
            .getInstance(LFResourceProvider.class).getResourceSet();

        // Prepare for the collection of tests per category.
        for (TestCategory t: TestCategory.values()) {
            allTargets.put(t, new TreeSet<>());
        }
        // Populate the registry.
        for (Target target : Target.values()) {

            // Walk the tree.
            try {
                Path dir = LF_TEST_PATH.resolve(target.toString());
                if (Files.exists(dir)) {
                    Files.walkFileTree(dir, new TestDirVisitor(rs, target));
                } else {
                    System.out.println("WARNING: No test directory for target " + target + "\n");
                }
                
            } catch (IOException e) {
                System.err.println(
                        "ERROR: Caught exception while indexing tests for target " + target);
                e.printStackTrace();
            }
            // Record the tests for later use when reporting coverage.
            Arrays.asList(TestCategory.values()).forEach(
                    c -> allTargets.get(c).addAll(getRegisteredTests(target, c, false)));
        }
        
        // Also scan the examples directory.
        try {
            Files.walkFileTree(LF_EXAMPLE_PATH, new ExampleDirVisitor(rs));
        } catch (IOException e) {
            System.err.println(
                    "Error while indexing tests from example directory.");
            e.printStackTrace();
        }
        
        if (erroneousExamples.size() > 0) {
            System.err.println("The following examples do not compile or specify no target:");
            erroneousExamples.forEach(System.err::println);
        }
        
    }
    
    /**
     * Calling this function forces the lazy initialization of the static code
     * that indexes all files. It is advisable to do this prior to executing
     * other code that prints to standard out so that any error messages
     * printed while indexing are printed first.
     */
    public static void initialize() {}
    
    /**
     * Return the tests that were indexed for a given target and category.
     * 
     * @param target The target to get indexed tests for.
     * @param category The category of tests to include in the returned tests.
     * @param copy Whether to return copies of the indexed tests instead of the indexed tests themselves.
     * @return A set of tests for the given target/category.
     */
    public static Set<LFTest> getRegisteredTests(Target target,
            TestCategory category, boolean copy) {
        if (copy) {
            Set<LFTest> copies = new TreeSet<>();
            for (LFTest test : registered.getTests(target, category)) {
                copies.add(new LFTest(test.target, test.srcFile));
            }
            return copies;
        } else {
            return registered.getTests(target, category);
        }
    }
    
    /**
     * Return the test that were found but not indexed because they did not
     * have a main reactor.
     */
    public static Set<LFTest> getIgnoredTests(Target target, TestCategory category) {
        return ignored.getTests(target, category);
    }

    public static String getCoverageReport(Target target, TestCategory category) {
        StringBuilder s = new StringBuilder();
        Set<LFTest> ignored = TestRegistry.getIgnoredTests(target, category);
        s.append(TestBase.THIN_LINE);
        s.append("Ignored: ").append(ignored.size()).append("\n");
        s.append(TestBase.THIN_LINE);
        
        for (LFTest test : ignored) {
            s.append("No main reactor in: ").append(test.name).append("\n");
        }
        
        Set<LFTest> own = getRegisteredTests(target, category, false);
        if (category.isCommon) {
            Set<LFTest> all = allTargets.get(category);
            s.append("\n").append(TestBase.THIN_LINE);
            s.append("Covered: ").append(own.size()).append("/").append(all.size()).append("\n");
            s.append(TestBase.THIN_LINE);
            int missing = all.size() - own.size();
            if (missing > 0) {
                all.stream().filter(test -> !own.contains(test))
                        .forEach(test -> s.append("Missing: ").append(test.toString()).append("\n"));
            }
        } else {
            s.append("\n").append(TestBase.THIN_LINE);
            s.append("Covered: ").append(own.size()).append("/").append(own.size()).append("\n");
            s.append(TestBase.THIN_LINE);
        }
        
        return s.toString();
    }

    /**
     * File visitor for indexing examples and example tests.
     */
    public static class ExampleDirVisitor extends SimpleFileVisitor<Path> {

        protected ResourceSet rs;
        
        boolean inTestDir = false;
        
        public ExampleDirVisitor(ResourceSet rs) {
            this.rs = rs;
        }
        
        /**
         * Update the state of the visitor to reflect whether it currently 
         * is in a test directory or not.
         */
        @Override
        public FileVisitResult postVisitDirectory(Path dir, IOException exc) {
            if (dir.getFileName().toString()
                    .equalsIgnoreCase("test")) {
                this.inTestDir = false;
            }
            return CONTINUE;
        }
        
        /**
         * Update the state of the visitor to reflect whether it currently 
         * is in a test directory or not.
         */
        @Override
        public FileVisitResult preVisitDirectory(Path dir,
                BasicFileAttributes attrs) {
            for (String ignored : IGNORED_DIRECTORIES) {
                if (dir.getFileName().toString().equalsIgnoreCase(ignored)) {
                    return SKIP_SUBTREE;
                }
            }
            if (dir.getFileName().toString()
                    .equalsIgnoreCase("test")) {
                this.inTestDir = true;
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

                EList<Diagnostic> errors = r.getErrors();
                if (!errors.isEmpty()) {
                    erroneousExamples.add(path);
                } else {
                    // No errors. Find the target.
                    Iterator<TargetDecl> targetDecls = filter(r.getAllContents(), TargetDecl.class);
                    Optional<Target> opt;
                    if (targetDecls.hasNext() && (opt = Target.forName(targetDecls.next().getName())).isPresent() ) {
                        Target target = opt.get();
                        Iterator<Reactor> reactors = filter(r.getAllContents(), Reactor.class);
                        if (exists(reactors, it -> it.isMain() || it.isFederated())) {
                            LFTest test = new LFTest(target, path);
                            if (this.inTestDir
                                || path.getFileName().toString().toLowerCase().contains("test")) {
                                // File is labeled as test.
                                registered.getTests(target, TestCategory.EXAMPLE_TEST)
                                          .add(test);
                            } else {
                                // Ordinary example.
                                registered.getTests(target, TestCategory.EXAMPLE)
                                          .add(test);
                            }
                            return CONTINUE;
                        }
                    } else {
                        // No target, list as failure.
                        erroneousExamples.add(path);
                    }
                }
            }
            return CONTINUE;
        }
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
    public static class TestDirVisitor extends SimpleFileVisitor<Path> {

        /**
         * The stack of which the top element indicates the "current" category.
         */
        protected Stack<TestCategory> stack = new Stack<>();

        /**
         * The target that all encountered tests belong to.
         */
        protected Target target;
        
        protected ResourceSet rs;

        protected Path srcBasePath;
        
        /**
         * Create a new file visitor based on a given target.
         * @param target The target that all encountered tests belong to.
         */
        public TestDirVisitor(ResourceSet rs, Target target) {
            stack.push(TestCategory.GENERIC);
            this.rs = rs;
            this.target = target;
            this.srcBasePath = LF_TEST_PATH.resolve(target.toString()).resolve("src");
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
                var relativePathName = srcBasePath.relativize(dir).toString();
                if (relativePathName.equalsIgnoreCase(category.getPath())) {
                    stack.push(category);
                }
            }
            return CONTINUE;
        }
               
        /**
         * Pop categories from the stack as appropriate.
         */
        @Override
        public FileVisitResult postVisitDirectory(Path dir, IOException exc) {
            for (TestCategory category : TestCategory.values()) {
                var relativePathName = srcBasePath.relativize(dir).toString();
                if (relativePathName.equalsIgnoreCase(category.getPath())) {
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
                // FIXME: issue warning if target doesn't match!
                LFTest test = new LFTest(target, path);
                EList<Diagnostic> errors = r.getErrors();
                if (!errors.isEmpty()) {
                    for (Diagnostic d : errors) {
                        test.issues.append(d.toString());
                    }
                    test.result = Result.PARSE_FAIL;
                } else {
                    Iterator<Reactor> reactors =
                        IteratorExtensions.filter(r.getAllContents(), Reactor.class);

                    if (!IteratorExtensions.exists(reactors,
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
