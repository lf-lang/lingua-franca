package org.lflang.tests.runtime;

import static org.junit.jupiter.api.Assertions.assertSame;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Objects;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.GeneratorDelegate;
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
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.generator.StandaloneContext;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Inject;
import com.google.inject.Provider;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public abstract class TestBase {

    @Inject
    IResourceValidator validator;
    @Inject
    GeneratorDelegate generator;
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

    /** Line separator used for ending lines in terminal output. */
    public static final String NEW_LINE = System.lineSeparator();

    /** Content separator used in test output, 78 characters wide. */
    public static final String DIVIDER =
        "+----------------------------------------------------------------------------+" + NEW_LINE;

    /** Content separator used in test output, 78 characters wide. */
    public static final String THIN_LINE =
        "------------------------------------------------------------------------------" + NEW_LINE;

    /** Content separator used in test output, 78 characters wide. */
    public static final String THICK_LINE = "====================================================" + NEW_LINE;

    /** Content separator used in test output, 78 characters wide. */
    public static final String EDGE_LINE =
        "+--------------------------------------------------------------------=-------+" + NEW_LINE;

    /** Static description of test that runs non-federated tests as federated ones. */
    public static final String RUN_AS_FEDERATED_DESC = "Description: Run non-federated tests in federated mode.";

    /** The current target for which tests are being run. */
    protected Target target;

    /** Whether or not to check/report on the result of the program under test. */
    protected boolean check = true;

    /** Whether to execute the program under test. */
    protected boolean run = true;

    /**
     * Whether to build/compile the produced target code or not.
     */
    protected boolean build = true;


    /**
     * Force the instantiation of the test registry.
     */
    protected TestBase() {
        TestRegistry.initialize();
    }

    // Tests.


    @Test
    public void runExampleTests() {
        printTestHeader("Description: Run example tests.");
        runTestsAndPrintResults(target, TestCategory.EXAMPLE_TEST::equals, null, false);
    }


    @Test
    public void compileExamples() {
        printTestHeader("Description: Compile examples.");
        this.run = false;

        runTestsAndPrintResults(target, TestCategory.EXAMPLE::equals, t -> true, false);
        this.run = false;
    }


    private void runUnthreaded(TestCategory generic) {
        runTestsAndPrintResults(target,
                                generic::equals,
                                it -> {
                                    it.getContext().getArgs().setProperty("threads", "0");
                                    return true;
                                },
                                false);
    }


    @Test
    public void runGenericTests() {
        printTestHeader("Description: Run generic tests (threads = 0).");
        runUnthreaded(TestCategory.GENERIC);
    }


    @Test
    public void runTargetSpecificTests() {
        printTestHeader("Description: Run target-specific tests (threads = 0).");
        runUnthreaded(TestCategory.TARGET);
    }


    @Test
    public void runMultiportTests() {
        printTestHeader("Description: Run multiport tests (threads = 0).");
        runUnthreaded(TestCategory.MULTIPORT);
    }


    @Test
    public void runAsFederated() {
        printTestHeader(RUN_AS_FEDERATED_DESC);

        EnumSet<TestCategory> categories = EnumSet.allOf(TestCategory.class);
        categories.removeAll(EnumSet.of(TestCategory.CONCURRENT,
                                        TestCategory.FEDERATED,
                                        TestCategory.EXAMPLE,
                                        TestCategory.EXAMPLE_TEST,
                                        // FIXME: also run the multiport tests once these are supported.
                                        TestCategory.MULTIPORT));

        runTestsAndPrintResults(target,
                                categories::contains,
                                it -> ASTUtils.makeFederated(it.fileConfig.resource),
                                true);
    }


    @Test
    public void runConcurrentTests() {
        printTestHeader("Description: Run concurrent tests.");
        runTestsAndPrintResults(target, TestCategory.CONCURRENT::equals, null, false);
    }


    @Test
    public void runFederatedTests() {
        printTestHeader("Description: Run federated tests.");
        runTestsAndPrintResults(target, TestCategory.FEDERATED::equals, null, false);
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


    protected final void runTestsAndPrintResults(Target target, Predicate<TestCategory> selection, Predicate<LFTest> configuration, boolean copy) {
        var categories = Arrays.stream(TestCategory.values()).filter(selection).collect(Collectors.toList());
        for (var category : categories) {
            System.out.println(category.getHeader());
            var tests = TestRegistry.getRegisteredTests(target, category, copy);
            try {
                validateAndRun(tests, configuration);
            } catch (IOException e) {
                throw new RuntimeIOException(e);
            }
            System.out.println((Object) TestRegistry.getCoverageReport(target, category));
            if (check) {
                checkAndReportFailures(tests);
            }
        }
    }


    protected void printTestHeader(String description) {
        System.out.print(TestBase.THICK_LINE);
        System.out.println("Target: " + this.target);
        System.out.println(description);
        System.out.println(TestBase.THICK_LINE);
    }


    private void checkAndReportFailures(Set<LFTest> registered) {
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


    private boolean configureAndValidate(LFTest test, Predicate<LFTest> configuration) throws IOException {

        if (test.result == Result.PARSE_FAIL) {
            // Abort is parsing was unsuccessful.
            return false;
        }

        redirectOutputs(test);

        var context = new StandaloneContext();
        // Update file config, which includes a fresh resource that has not
        // been tampered with using AST transformations.
        context.setCancelIndicator(CancelIndicator.NullImpl);
        context.setArgs(new Properties());
        context.setPackageRoot(test.packageRoot);
        context.setHierarchicalBin(true);

        var r = resourceSetProvider.get().getResource(
            URI.createFileURI(test.srcFile.toFile().getAbsolutePath()),
            true);

        if (r.getErrors().size() > 0) {
            test.result = Result.PARSE_FAIL;
            restoreOutputs();
            return false;
        }
        fileAccess.setOutputPath(context.getPackageRoot().resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
        test.fileConfig = new FileConfig(r, fileAccess, context);

        // Set the no-compile flag if appropriate.
        if (!this.build) {
            context.getArgs().setProperty("no-compile", "");
        }

        addExtraLfcArgs(context.getArgs());

        // Validate the resource and store issues in the test object.
        try {
            var issues = validator.validate(test.fileConfig.resource,
                                            CheckMode.ALL, context.getCancelIndicator());
            if (issues != null && !issues.isEmpty()) {
                String issuesToString = issues.stream().map(Objects::toString).collect(Collectors.joining(NEW_LINE));
                test.issues.append(issuesToString);
                if (issues.stream().anyMatch(it -> it.getSeverity() == Severity.ERROR)) {
                    test.result = Result.VALIDATE_FAIL;
                    restoreOutputs();
                    return false;
                }
            }
        } catch (Exception e) {
            test.result = Result.VALIDATE_FAIL;
            restoreOutputs();
            return false;
        }

        // Update the test by applying the configuration. E.g., to carry out an AST transformation.
        if (configuration != null && !configuration.test(test)) {
            test.result = Result.CONFIG_FAIL;
            restoreOutputs();
            return false;
        }

        restoreOutputs();
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
     * @return True if code was generated successfully, false otherwise.
     */
    private boolean generateCode(LFTest test) {
        if (test.fileConfig.resource != null) {
            redirectOutputs(test);
            try {
                generator.generate(test.fileConfig.resource, fileAccess, test.fileConfig.context);
            } catch (Exception e) {
                e.printStackTrace();
                test.issues.append(e.getMessage());
                test.result = Result.CODE_GEN_FAIL;
                restoreOutputs();
                return false;
            }

            restoreOutputs();
            return true;
        }
        return false;
    }


    /**
     * Given an indexed test, execute it and label the test as failing if it
     * did not execute, took too long to execute, or executed but exited with
     * an error code.
     */
    private void execute(LFTest test) {
        ProcessBuilder pb = null;
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
                pb = new ProcessBuilder(fullPath.toString());
                pb.directory(binPath.toFile());
            } else {
                test.issues.append(fullPath).append(": No such file or directory.").append(NEW_LINE);
                test.result = Result.NO_EXEC_FAIL;
            }
        }
        case Python: {
            var srcGen = test.fileConfig.getSrcGenPath();
            var fullPath = srcGen.resolve(nameOnly + ".py");
            if (Files.exists(fullPath)) {
                pb = new ProcessBuilder("python3", fullPath.toFile().getName());
                pb.directory(srcGen.toFile());
            } else {
                test.result = Result.NO_EXEC_FAIL;
                if (pb != null) {
                    test.issues.append("Process builder: ").append(pb).append(NEW_LINE);
                }
                test.issues.append("File: ").append(fullPath).append(NEW_LINE);
            }
        }
        case TS: {
            var dist = test.fileConfig.getSrcGenPath().resolve("dist");
            var file = dist.resolve(nameOnly + ".js");
            if (Files.exists(file)) {
                pb = new ProcessBuilder("node", file.toString());
            } else {
                test.result = Result.NO_EXEC_FAIL;
                if (pb != null) {
                    test.issues.append("Process builder: ").append(pb).append(NEW_LINE);
                }
                test.issues.append("File: ").append(file).append(NEW_LINE);
            }
        }
        }
        if (pb != null) {
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

        } else {
            test.result = Result.NO_EXEC_FAIL;
        }

    }


    private void validateAndRun(Set<LFTest> tests, Predicate<LFTest> configuration) throws IOException { // FIXME change this into Consumer
        final var x = 78f / tests.size();
        var marks = 0;
        var done = 0;
        for (var test : tests) {
            if (configureAndValidate(test, configuration) && generateCode(test)) {
                if (run) {
                    execute(test);
                } else if (test.result == Result.UNKNOWN) {
                    test.result = Result.TEST_PASS;
                }
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

        System.out.print(NEW_LINE);
    }
}
