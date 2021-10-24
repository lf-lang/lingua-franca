package org.lflang.tests;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.GeneratorContext;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.util.RuntimeIOException;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.DefaultErrorReporter;
import org.lflang.FileConfig;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.StandaloneContext;
import org.lflang.tests.Configurators.Configurator;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.util.StringUtil;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

/**
 * Base class for test classes that define JUnit tests.
 *
 * @author Marten Lohstroh <marten@berkeley.edu>
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public abstract class TestBase {

    @Inject
    IResourceValidator validator;
    @Inject
    LFGenerator generator;
    @Inject
    JavaIoFileSystemAccess fileAccess;
    @Inject
    Provider<ResourceSet> resourceSetProvider;


    /** Reference to System.out. */
    private static final PrintStream out = System.out;

    /** Reference to System.err. */
    private static final PrintStream err = System.err;

    /** Execution timeout enforced for all tests. */
    private static final long MAX_EXECUTION_TIME_SECONDS = 60;

    /** Content separator used in test output, 78 characters wide. */
    public static final String THIN_LINE =
        "------------------------------------------------------------------------------" +
            System.lineSeparator();

    /** Content separator used in test output, 78 characters wide. */
    public static final String THICK_LINE =
        "==============================================================================" +
            System.lineSeparator();

    /** The targets for which to run the tests. */
    private final List<Target> targets;
    /**
     * Whether the goal is to only computer code coverage, in which we cut down
     * on verbosity of our error reporting.
     */
    protected boolean codeCovOnly;



    /**
     * An enumeration of test levels.
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public enum TestLevel {VALIDATION, CODE_GEN, BUILD, EXECUTION}
    /**
     * A collection messages often used throughout the test package.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public static class Message {
        /* Reasons for not running tests. */
        public static final String NO_WINDOWS_SUPPORT = "Not (yet) supported on Windows.";
        public static final String NO_CPP_SUPPORT = "Not supported by reactor-cpp.";
        public static final String NO_RUST_SUPPORT = "Not supported by reactor-rust.";
        public static final String NO_TS_SUPPORT = "Not supported by reactor-ts.";
        public static final String NOT_FOR_CODE_COV = "Unlikely to help improve code coverage.";
        public static final String ALWAYS_MULTITHREADED = "The reactor-ccp runtime is always multithreaded.";
        public static final String NO_THREAD_SUPPORT = "Target does not support the 'threads' property.";

        /* Descriptions of collections of tests. */
        public static final String DESC_SERIALIZATION = "Run serialization tests (threads = 0).";
        public static final String DESC_AS_FEDERATED = "Run non-federated tests in federated mode.";
        public static final String DESC_FEDERATED = "Run federated tests.";
        public static final String DESC_CONCURRENT = "Run concurrent tests.";
        public static final String DESC_TARGET_SPECIFIC = "Run target-specific tests (threads = 0)";
        public static final String DESC_AS_CCPP = "Running C tests as CCpp.";
        public static final String DESC_FOUR_THREADS = "Run non-concurrent and non-federated tests (threads = 4).";
    }

    /** Constructor for test classes that test a single target. */
    protected TestBase(Target first) {
        this(Collections.singletonList(first));
    }

    /** Special ctor for the code coverage test */
    protected TestBase(List<Target> targets) {
        assertFalse(targets.isEmpty(), "empty target list");
        this.targets = Collections.unmodifiableList(targets);
        TestRegistry.initialize();
    }

    /**
     * Run selected tests for a given target and configurator up to the specified level.
     *
     * @param target The target to run tests for.
     * @param selected A predicate that given a test category returns whether
     * it should be included in this test run or not.
     * @param configurator  A procedure for configuring the tests.
     * @param level The level of testing to be performed during this run.
     * @param copy Whether or not to work on copies of tests in the test.
     * registry.
     */
    protected final void runTestsAndPrintResults(Target target,
                                                 Predicate<TestCategory> selected,
                                                 Configurator configurator,
                                                 TestLevel level,
                                                 boolean copy) {
        var categories = Arrays.stream(TestCategory.values()).filter(selected)
                .collect(Collectors.toList());
        for (var category : categories) {
            System.out.println(category.getHeader());
            var tests = TestRegistry.getRegisteredTests(target, category, copy);
            try {
                validateAndRun(tests, configurator, level);
            } catch (IOException e) {
                throw new RuntimeIOException(e);
            }
            System.out
                    .println(TestRegistry.getCoverageReport(target, category));
            if (!this.codeCovOnly) {
                checkAndReportFailures(tests);
            }
        }
    }

    /**
     * Run tests in the given selection for all targets enabled in this class.
     *
     * @param description A string that describes the collection of tests.
     * @param selected A predicate that given a test category returns whether
     * it should be included in this test run or not.
     * @param configurator A procedure for configuring the tests.
     * @param level The level of testing to be performed during this run.
     * @param copy Whether or not to work on copies of tests in the test.
     * registry.
     */
    protected void runTestsForTargets(String description,
                                      Predicate<TestCategory> selected,
                                      Configurator configurator,
                                      TestLevel level,
                                      boolean copy) {
        for (Target target : this.targets) {
            runTestsFor(List.of(target), description, selected,
                        configurator, level, copy);
        }
    }

    /**
     * Run tests in the given selection for a subset of given targets.
     *
     * @param subset The subset of targets to run the selected tests for.
     * @param description A string that describes the collection of tests.
     * @param selected A predicate that given a test category returns whether
     * it should be included in this test run or not.
     * @param configurator A procedure for configuring the tests.
     * @param level The level of testing to be performed during this run.
     * @param copy Whether or not to work on copies of tests in the test.
     * registry.
     */
    protected void runTestsFor(List<Target> subset,
                               String description,
                               Predicate<TestCategory> selected,
                               Configurator configurator,
                               TestLevel level,
                               boolean copy) {
        for (Target target : subset) {
            printTestHeader(target, description);
            runTestsAndPrintResults(target, selected, configurator, level, copy);
        }
    }

    /**
     * Determine whether the current platform is Windows.
     * @return true if the current platform is Windwos, false otherwise.
     */
    protected static boolean isWindows() {
        String OS = System.getProperty("os.name").toLowerCase();
        return OS.contains("win");
    }

    /**
     * End output redirection.
     */
    private static void restoreOutputs() {
        System.out.flush();
        System.err.flush();
        System.setOut(out);
        System.setErr(err);
    }

    /**
     * Redirect outputs to the given tests for recording.
     *
     * @param test The test to redirect outputs to.
     */
    private static void redirectOutputs(LFTest test) {
        System.setOut(new PrintStream(test.out));
        System.setErr(new PrintStream(test.err));
    }


    /**
     * Run a test, print results on stderr.
     *
     * @param test      Test case.
     * @param testClass The test class that will execute the test. This is target-specific,
     *                  it may provide some target-specific configuration. We pass a class
     *                  and not a new instance because this method needs to ensure the object
     *                  is properly injected, and so, it needs to control its entire lifecycle.
     * @param level     Level to which to run the test.
     */
    public static void runSingleTestAndPrintResults(LFTest test, Class<? extends TestBase> testClass, TestLevel level) {
        Injector injector = new LFStandaloneSetup(new LFRuntimeModule()).createInjectorAndDoEMFRegistration();
        TestBase runner;
        try {
            @SuppressWarnings("unchecked")
            Constructor<? extends TestBase> constructor = (Constructor<? extends TestBase>) testClass.getConstructors()[0];
            runner = constructor.newInstance();
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException(e);
        }
        injector.injectMembers(runner);

        Set<LFTest> tests = Set.of(test);
        try {
            runner.validateAndRun(tests, t -> true, level);
        } catch (IOException e) {
            throw new RuntimeIOException(e);
        }
        checkAndReportFailures(tests);
    }

    /**
     * Print a header that describes a collection of tests, followed by a reason
     * for skipping the tests.
     *
     * @param description A string the describes the collection of tests.
     * @param reason A string that describes why the tests are not performed.
     */
    protected void printSkipMessage(String description, String reason) {
        for (var target : this.targets) {
           printTestHeader(target, description);
           System.out.println("Warning! Skipping because: " + reason);
        }
    }

    /**
     * Print a header that describes a collection of tests.
     * @param target The target for which the tests are being performed.
     * @param description A string the describes the collection of tests.
     */
    protected static void printTestHeader(Target target, String description) {
        System.out.print(TestBase.THICK_LINE);
        System.out.println("Target: " + target);
        if (description.startsWith("Description: ")) {
            System.out.println(description);
        } else {
            System.out.println("Description: " + description);
        }
        System.out.println(TestBase.THICK_LINE);
    }

    /**
     * Iterate over given tests and evaluate their outcome, report errors if
     * there are any.
     *
     * @param tests The tests to inspect the results of.
     */
    private static void checkAndReportFailures(Set<LFTest> tests) {
        var passed = tests.stream().filter(it -> !it.hasFailed()).count();

        System.out.print(THIN_LINE);
        System.out.println("Passing: " + passed + "/" + tests.size());
        System.out.print(THIN_LINE);

        for (var test : tests) {
            System.out.print(test.reportErrors());
        }
        for (LFTest lfTest : tests) {
            assertSame(Result.TEST_PASS, lfTest.result);
        }
    }

    /**
     * Configure a test by applying the given configurator and return a
     * generator context. Also, if the given level is less than
     * `TestLevel.BUILD`, add a `no-compile` flag to the generator context. If
     * the configurator was not applied successfully, throw an AssertionError.
     *
     * @param test the test to configure.
     * @param configurator The configurator to apply to the test.
     * @param level The level of testing in which the generator context will be
     * used.
     * @return a generator context with a fresh resource, unaffected by any AST
     * transformation that may have occured in other tests.
     * @throws IOException if there is any file access problem
     */
    private GeneratorContext configure(LFTest test, Configurator configurator, TestLevel level) throws IOException {
        var context = new StandaloneContext();
        // Update file config, which includes a fresh resource that has not
        // been tampered with using AST transformations.
        context.setCancelIndicator(CancelIndicator.NullImpl);
        context.setArgs(new Properties());
        context.setPackageRoot(test.packageRoot);
        context.setHierarchicalBin(true);
        context.setReporter(new DefaultErrorReporter());
        
        var r = resourceSetProvider.get().getResource(
            URI.createFileURI(test.srcFile.toFile().getAbsolutePath()),
            true);

        if (r.getErrors().size() > 0) {
            test.result = Result.PARSE_FAIL;
            throw new AssertionError("Test did not parse correctly.");
        }

        fileAccess.setOutputPath(context.getPackageRoot().resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
        test.fileConfig = new FileConfig(r, fileAccess, context);

        // Set the no-compile flag the test is not supposed to reach the build stage.
        if (level.compareTo(TestLevel.BUILD) < 0 || this.codeCovOnly) {
            context.getArgs().setProperty("no-compile", "");
        }

        addExtraLfcArgs(context.getArgs());

        // Update the test by applying the configuration. E.g., to carry out an AST transformation.
        if (configurator != null && !configurator.configure(test)) {
            test.result = Result.CONFIG_FAIL;
            throw new AssertionError("Test configuration unsuccessful.");
        }

        return context;
    }

    /**
     * Validate the given test. Throw an AssertionError if validation failed.
     */
    private void validate(LFTest test, GeneratorContext context) {
        // Validate the resource and store issues in the test object.
        try {
            var issues = validator.validate(test.fileConfig.resource,
                                            CheckMode.ALL, context.getCancelIndicator());
            if (issues != null && !issues.isEmpty()) {
                String issuesToString = issues.stream().map(Objects::toString).collect(Collectors.joining(System.lineSeparator()));
                test.issues.append(issuesToString);
                if (issues.stream().anyMatch(it -> it.getSeverity() == Severity.ERROR)) {
                    test.result = Result.VALIDATE_FAIL;
                }
            }
        } catch (Exception e) {
            test.result = Result.VALIDATE_FAIL;
        }
        if (test.result == Result.VALIDATE_FAIL) {
            throw new AssertionError("Validation unsuccessful.");
        }
    }


    /**
     * Override to add some LFC arguments to all runs of this test class.
     */
    protected void addExtraLfcArgs(Properties args) {
        // to be overridden
    }


    /**
     * Invoke the code generator for the given test.
     * @param test The test to generate code for.
     */
    private void generateCode(LFTest test) {
        if (test.fileConfig.resource != null) {
            generator.doGenerate(test.fileConfig.resource, fileAccess, test.fileConfig.context);
            if (generator.errorsOccurred()) {
                test.result = Result.CODE_GEN_FAIL;
                throw new AssertionError("Code generation unsuccessful.");
            }
        }
    }


    /**
     * Given an indexed test, execute it and label the test as failing if it
     * did not execute, took too long to execute, or executed but exited with
     * an error code.
     */
    private void execute(LFTest test) {
        final ProcessBuilder pb = getExecCommand(test);
        if (pb == null) {
            return;
        }
        try {
            var p = pb.start();
            var stdout = test.execLog.recordStdOut(p);
            var stderr = test.execLog.recordStdErr(p);
            if (!p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS)) {
                stdout.interrupt();
                stderr.interrupt();
                p.destroyForcibly();
                test.result = Result.TEST_TIMEOUT;
            } else {
                if (p.exitValue() == 0) {
                    test.result = Result.TEST_PASS;
                } else {
                    test.result = Result.TEST_FAIL;
                }
            }

        } catch (Exception e) {
            test.result = Result.TEST_FAIL;
        }
    }

    /**
     * Return a preconfigured ProcessBuilder for the command
     * that should be used to execute the test program.
     * @param test The test to get the execution command for.
     */
    private ProcessBuilder getExecCommand(LFTest test) {
        final var nameWithExtension = test.srcFile.getFileName().toString();
        final var nameOnly = nameWithExtension.substring(0, nameWithExtension.lastIndexOf('.'));

        switch (test.target) {
        case C:
        case CPP:
        case Rust:
        case CCPP: {
            var binPath = test.fileConfig.binPath;
            var binaryName = nameOnly;
            if (test.target == Target.Rust) {
                // rust binaries uses snake_case
                binaryName = StringUtil.camelToSnakeCase(binaryName);
            }
            // Adjust binary extension if running on Window
            if (System.getProperty("os.name").startsWith("Windows")) {
                binaryName += ".exe";
            }

            var fullPath = binPath.resolve(binaryName);
            if (Files.exists(fullPath)) {
                // Running the command as .\binary.exe does not work on Windows for
                // some reason... Thus we simply pass the full path here, which
                // should work across all platforms
                return new ProcessBuilder(fullPath.toString()).directory(binPath.toFile());
            } else {
                test.issues.append(fullPath).append(": No such file or directory.").append(System.lineSeparator());
                test.result = Result.NO_EXEC_FAIL;
                return null;
            }
        }
        case Python: {
            var srcGen = test.fileConfig.getSrcGenPath();
            var fullPath = srcGen.resolve(nameOnly + ".py");
            if (Files.exists(fullPath)) {
                return new ProcessBuilder("python3", fullPath.getFileName().toString())
                    .directory(srcGen.toFile());
            } else {
                test.result = Result.NO_EXEC_FAIL;
                test.issues.append("File: ").append(fullPath).append(System.lineSeparator());
                return null;
            }
        }
        case TS: {
            var binPath = test.fileConfig.binPath;
            var binaryName = nameOnly;
            // Adjust binary extension if running on Window
            if (System.getProperty("os.name").startsWith("Windows")) {
                binaryName += ".exe";
            }
            var fullPath = binPath.resolve(binaryName);
            if (Files.exists(fullPath)) {
                // If execution script exists, run it.
                return new ProcessBuilder(fullPath.toString()).directory(binPath.toFile());
            }
            // If execution script does not exist, run .js directly.
            var dist = test.fileConfig.getSrcGenPath().resolve("dist");
            var file = dist.resolve(nameOnly + ".js");
            if (Files.exists(file)) {
                return new ProcessBuilder("node", file.toString());
            } else {
                test.result = Result.NO_EXEC_FAIL;
                test.issues.append("File: ").append(file).append(System.lineSeparator());
                return null;
            }
        }
        default:
            throw new AssertionError("unreachable");
        }
    }

    /**
     * Validate and run the given tests, using the specified configuratator and level.
     *
     * While performing tests, this method prints a header that reaches completion
     * once all tests have been run.
     *
     * @param tests A set of tests to run.
     * @param configurator A procedure for configuring the tests.
     * @param level The level of testing.
     * @throws IOException If initial file configuration fails
     */
    private void validateAndRun(Set<LFTest> tests, Configurator configurator, TestLevel level) throws IOException {
        final var x = 78f / tests.size();
        var marks = 0;
        var done = 0;

        for (var test : tests) {
            try {
                redirectOutputs(test);
                var context = configure(test, configurator, level);
                validate(test, context);
                if (level.compareTo(TestLevel.CODE_GEN) >= 0) {
                    generateCode(test);
                }
                if (!this.codeCovOnly && level == TestLevel.EXECUTION) {
                    execute(test);
                } else if (test.result == Result.UNKNOWN) {
                    test.result = Result.TEST_PASS;
                }

            } catch (AssertionError e) {
                // Do not report assertion errors. They are pretty printed
                // during reporting.
            } catch (Exception e) {
                test.issues.append(e.getMessage());
            } finally {
                restoreOutputs();
            }
            done++;
            while (Math.floor(done * x) >= marks && marks < 78) {
                System.out.print("=");
                marks++;
            }
        }
        while (marks < 78) {
            System.out.print("=");
            marks++;
        }

        System.out.print(System.lineSeparator());
    }
}
