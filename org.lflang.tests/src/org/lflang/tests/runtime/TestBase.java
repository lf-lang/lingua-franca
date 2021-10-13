package org.lflang.tests.runtime;

import static org.junit.jupiter.api.Assertions.assertSame;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
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
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.ASTUtils;
import org.lflang.DefaultErrorReporter;
import org.lflang.FileConfig;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.StandaloneContext;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

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
    protected List<Target> targets = new ArrayList<Target>();
    
    /** Whether the goal is to only computer code coverage, in which we cut down on verbosity of our error reporting. */
    protected boolean codeCovOnly;
    
    private static Predicate<LFTest> makeSingleThreaded = it -> {
        it.getContext().getArgs().setProperty("threads", "0");
        return true;
    };

    
    /**
     * Force the instantiation of the test registry.
     */
    protected TestBase() {
        this(true);
    }

    private TestBase(boolean initRegistry) {
        if (initRegistry) {
            TestRegistry.initialize();
        }
    }

    // Tests.
    
    protected void runTestsForTargets(String description,
            Predicate<TestCategory> selection, Predicate<LFTest> configuration,
            TestLevel level, boolean copy) {
        for (Target target : this.targets) {
            printTestHeader(target, description);
            runTestsAndPrintResults(target, selection, configuration, level,
                    copy);
        }
    }
    
    protected void runTestsFor(List<Target> subset, String description,
            Predicate<TestCategory> selection, Predicate<LFTest> configuration,
            TestLevel level, boolean copy) {
        for (Target target : subset) {
            printTestHeader(target, description);
            runTestsAndPrintResults(target, selection, configuration, level,
                    copy);
        }
    }
    
    @Test
    public void runExampleTests() {
        runTestsForTargets("Description: Run example tests.",
                TestCategory.EXAMPLE_TEST::equals, t -> true,
                TestLevel.EXECUTION, false);
    }
    
    @Test
    public void validateExamples() {
        runTestsForTargets("Description: Validate examples.",
                TestCategory.EXAMPLE::equals, t -> true, TestLevel.VALIDATION,
                false);
    }
    
    @Test
    public void runGenericTests() {
        runTestsForTargets("Description: Run generic tests (threads = 0).",
                TestCategory.GENERIC::equals, makeSingleThreaded,
                TestLevel.EXECUTION, false);
    }

    @Test
    public void runTargetSpecificTests() {
        runTestsForTargets("Description: Run target-specific tests (threads = 0).",
                TestCategory.TARGET::equals, makeSingleThreaded,
                TestLevel.EXECUTION, false);
    }

    @Test
    public void runMultiportTests() {
        runTestsForTargets("Description: Run multiport tests (threads = 0).",
                TestCategory.MULTIPORT::equals, makeSingleThreaded,
                TestLevel.EXECUTION, false);
    }
    
    @Test
    public void runSerializationTests() {
        runTestsForTargets("Description: Run serialization tests (threads = 0).",
                TestCategory.SERIALIZATION::equals, makeSingleThreaded,
                TestLevel.EXECUTION, false);
    }

    @Test
    public void runAsFederated() {
        EnumSet<TestCategory> categories = EnumSet.allOf(TestCategory.class);
        categories.removeAll(EnumSet.of(TestCategory.CONCURRENT,
                                        TestCategory.FEDERATED,
                                        TestCategory.EXAMPLE,
                                        TestCategory.EXAMPLE_TEST,
                                        // FIXME: also run the multiport tests once these are supported.
                                        TestCategory.MULTIPORT));

        runTestsFor(Arrays.asList(Target.C), Message.DESC_AS_FEDERATED,
                                categories::contains,
                                it -> ASTUtils.makeFederated(it.fileConfig.resource),
                                TestLevel.EXECUTION,
                                true);
    }


    @Test
    public void runConcurrentTests() {
        runTestsForTargets("Description: Run concurrent tests.",
                TestCategory.CONCURRENT::equals, t -> true, TestLevel.EXECUTION,
                false);

    }

    @Test
    public void runFederatedTests() {
        runTestsForTargets("Description: Run federated tests.",
                TestCategory.FEDERATED::equals, t -> true, TestLevel.EXECUTION,
                false);
    }


    /** Returns true if the operating system is Windows. */
    protected static boolean isWindows() {
        String OS = System.getProperty("os.name").toLowerCase();
        if (OS.indexOf("win") >= 0) { return true; }
        return false;
    }

    //
    private static void restoreOutputs() {
        System.out.flush();
        System.err.flush();
        System.setOut(out);
        System.setErr(err);
    }


    private static void redirectOutputs(LFTest test) {
        System.setOut(new PrintStream(test.out));
        System.setErr(new PrintStream(test.err));
    }


    protected final void runTestsAndPrintResults(Target target,
            Predicate<TestCategory> selection, Predicate<LFTest> configuration,
            TestLevel level,
            boolean copy) {
        var categories = Arrays.stream(TestCategory.values()).filter(selection)
                .collect(Collectors.toList());
        for (var category : categories) {
            System.out.println(category.getHeader());
            var tests = TestRegistry.getRegisteredTests(target, category, copy);
            try {
                validateAndRun(tests, configuration, level);
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

    public static void runSingleTestAndPrintResults(LFTest test, TestLevel level) {
        Injector injector = new LFStandaloneSetup(new LFRuntimeModule()).createInjectorAndDoEMFRegistration();
        TestBase runner = new TestBase(false) {};
        injector.injectMembers(runner);

        Set<LFTest> tests = Set.of(test);
        try {
            runner.validateAndRun(tests, t -> true, level);
        } catch (IOException e) {
            throw new RuntimeIOException(e);
        }
        checkAndReportFailures(tests);
    }

    protected void printSkipMessage(String description, String reason) {
        for (var target : this.targets) {
           printTestHeader(target, description);
           System.out.println("Warning! Skipping because: " + reason);
        }
    }

    protected static void printTestHeader(Target target, String description) {
        System.out.print(TestBase.THICK_LINE);
        System.out.println("Target: " + target);
        System.out.println("Description: " + description);
        System.out.println(TestBase.THICK_LINE);
    }


    private static void checkAndReportFailures(Set<LFTest> registered) {
        var passed = registered.stream().filter(it -> !it.hasFailed()).count();

        System.out.print(THIN_LINE);
        System.out.println("Passing: " + passed + "/" + registered.size());
        System.out.print(THIN_LINE);

        for (var test : registered) {
            System.out.print(test.reportErrors());
        }
        for (LFTest lfTest : registered) {
            assertSame(Result.TEST_PASS, lfTest.result);
        }
    }


    private GeneratorContext configure(LFTest test,
            Predicate<LFTest> configuration, TestLevel level) throws IOException {
        
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
            throw new RuntimeException(
                    "Test did not parse correctly, so it cannot be configured.");
        }
        
        fileAccess.setOutputPath(context.getPackageRoot().resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
        test.fileConfig = new FileConfig(r, fileAccess, context);

        // Set the no-compile flag the test is not supposed to reach the build stage.
        if (level.compareTo(TestLevel.BUILD) < 0 || this.codeCovOnly) {
            context.getArgs().setProperty("no-compile", "");
        }

        addExtraLfcArgs(context.getArgs());
        
        // Update the test by applying the configuration. E.g., to carry out an AST transformation.
        if (configuration != null && !configuration.test(test)) {
            test.result = Result.CONFIG_FAIL;
            throw new RuntimeException(
                    "Test configuration could not be applied successfully.");
        }
        
        return context;
    }
    
    private boolean validate(LFTest test, GeneratorContext context) {
        // Validate the resource and store issues in the test object.
        try {
            var issues = validator.validate(test.fileConfig.resource,
                                            CheckMode.ALL, context.getCancelIndicator());
            if (issues != null && !issues.isEmpty()) {
                String issuesToString = issues.stream().map(Objects::toString).collect(Collectors.joining(System.lineSeparator()));
                test.issues.append(issuesToString);
                if (issues.stream().anyMatch(it -> it.getSeverity() == Severity.ERROR)) {
                    test.result = Result.VALIDATE_FAIL;
                    return false;
                }
            }
        } catch (Exception e) {
            test.result = Result.VALIDATE_FAIL;
            return false;
        }

        return true;
    }


    /**
     * Override to add some LFC arguments to all runs of this test class.
     */
    protected void addExtraLfcArgs(Properties args) {
        // to be overridden
    }


    /**
     * Invoke the code generator for the given test.
     *
     */
    private void generateCode(LFTest test) {
        if (test.fileConfig.resource != null) {
            generator.doGenerate(test.fileConfig.resource, fileAccess, test.fileConfig.context);
            if (generator.errorsOccurred()) {
                test.result = Result.CODE_GEN_FAIL;
                throw new RuntimeException("Errors occurred during code generation.");
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
            var stdout = test.exec.recordStdOut(p);
            var stderr = test.exec.recordStdErr(p);
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
     * Returns a preconfigured ProcessBuilder for the command
     * that should be used to execute the test program.
     */
    private ProcessBuilder getExecCommand(LFTest test) {
        final var nameWithExtension = test.srcFile.getFileName().toString();
        final var nameOnly = nameWithExtension.substring(0, nameWithExtension.lastIndexOf('.'));

        switch (test.target) {
        case C:
        case CPP:
        case CCPP: {
            var binPath = test.fileConfig.binPath;
            var binaryName = nameOnly;
            // Adjust binary extension if running on Window
            if (System.getProperty("os.name").startsWith("Windows")) {
                binaryName = nameOnly + ".exe";
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


    private void validateAndRun(Set<LFTest> tests, Predicate<LFTest> configuration, TestLevel level) throws IOException { // FIXME change this into Consumer
        final var x = 78f / tests.size();
        var marks = 0;
        var done = 0;
        
        for (var test : tests) {
            try {
                redirectOutputs(test);
                var context = configure(test, configuration, level);
                validate(test, context);
                if (level.compareTo(TestLevel.CODE_GEN) >= 0) {
                    generateCode(test);
                }
                if (!this.codeCovOnly && level == TestLevel.EXECUTION) {
                    execute(test);
                } else if (test.result == Result.UNKNOWN) {
                    test.result = Result.TEST_PASS;
                }

            } catch (Exception e) {
                // System.out.println(e.getMessage());
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
    
    public enum TestLevel {VALIDATION, CODE_GEN, BUILD, EXECUTION};
    public class Message {
        public final static String NO_WINDOWS_SUPPORT = "Not (yet) supported on Windows.";
        public final static String NO_CPP_SUPPORT = "Not supported by reactor-cpp.";
        public final static String ALWAYS_MULTITHREADED = "The reactor-ccp runtime is always multithreaded.";
        public final static String DESC_SERIALIZATION = "Run serialization tests (threads = 0).";
        public final static String DESC_AS_FEDERATED = "Run non-federated tests in federated mode.";
        public final static String DESC_FEDERATED = "Run federated tests.";
        public final static String DESC_TARGET_SPECIFIC = "Run target-specific tests (threads = 0)";
        public final static String DESC_AS_CCPP = "Running C tests as CCpp.";
        public final static String DESC_FOUR_THREADS = "Run non-concurrent and non-federated tests (threads = 4).";
    }

}
