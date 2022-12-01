package org.lflang.tests;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.Thread.UncaughtExceptionHandler;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Path;
import java.io.File;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.IGeneratorContext;
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
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.MainContext;
import org.lflang.tests.Configurators.Configurator;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

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
     * An enumeration of test levels.
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public enum TestLevel {VALIDATION, CODE_GEN, BUILD, EXECUTION}

    /**
     * Static function for converting a path to its associated test level.
     * @author Anirudh Rengarajan <arengarajan@berkeley.edu>
     */
    public static TestLevel pathToLevel(Path path) {
        while(path.getParent() != null) {
            String name = path.getFileName().toString();
            for (var category: TestCategory.values()) {
                if (category.name().equalsIgnoreCase(name)) {
                    return category.level;
                }
            }
            path = path.getParent();
        }
        return TestLevel.EXECUTION;
    }

    /**
     * A collection messages often used throughout the test package.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     *
     */
    public static class Message {
        /* Reasons for not running tests. */
        public static final String NO_WINDOWS_SUPPORT = "Not (yet) supported on Windows.";
        public static final String NO_SINGLE_THREADED_SUPPORT = "Target does not support single-threaded execution.";
        public static final String NO_FEDERATION_SUPPORT = "Target does not support federated execution.";
        public static final String NO_DOCKER_SUPPORT = "Target does not support the 'docker' property.";
        public static final String NO_DOCKER_TEST_SUPPORT = "Docker tests are only supported on Linux.";
        public static final String NO_GENERICS_SUPPORT = "Target does not support generic types.";

        /* Descriptions of collections of tests. */
        public static final String DESC_SERIALIZATION = "Run serialization tests.";
        public static final String DESC_GENERIC = "Run generic tests.";
        public static final String DESC_TYPE_PARMS = "Run tests for reactors with type parameters.";
        public static final String DESC_MULTIPORT = "Run multiport tests.";
        public static final String DESC_AS_FEDERATED = "Run non-federated tests in federated mode.";
        public static final String DESC_FEDERATED = "Run federated tests.";
        public static final String DESC_DOCKER = "Run docker tests.";
        public static final String DESC_DOCKER_FEDERATED = "Run docker federated tests.";
        public static final String DESC_CONCURRENT = "Run concurrent tests.";
        public static final String DESC_TARGET_SPECIFIC = "Run target-specific tests";
        public static final String DESC_ARDUINO = "Running Arduino tests.";
        public static final String DESC_AS_CCPP = "Running C tests as CCpp.";
        public static final String DESC_SINGLE_THREADED = "Run non-concurrent and non-federated tests with threading = off.";
        public static final String DESC_SCHED_SWAPPING = "Running with non-default runtime scheduler ";
        public static final String DESC_ROS2 = "Running tests using ROS2.";
        public static final String DESC_MODAL = "Run modal reactor tests.";

        /* Missing dependency messages */
        public static final String MISSING_DOCKER = "Executable 'docker' not found or 'docker' daemon thread not running";
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
                                                 boolean copy) {
        var categories = Arrays.stream(TestCategory.values()).filter(selected)
                .collect(Collectors.toList());
        for (var category : categories) {
            System.out.println(category.getHeader());
            var tests = TestRegistry.getRegisteredTests(target, category, copy);
            try {
                validateAndRun(tests, configurator, category.level);
            } catch (IOException e) {
                throw new RuntimeIOException(e);
            }
            System.out.println(TestRegistry.getCoverageReport(target, category));
            checkAndReportFailures(tests);
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
                                      boolean copy) {
        for (Target target : this.targets) {
            runTestsFor(List.of(target), description, selected,
                        configurator, copy);
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
     * @param copy Whether to work on copies of tests in the test.
     * registry.
     */
    protected void runTestsFor(List<Target> subset,
                               String description,
                               Predicate<TestCategory> selected,
                               Configurator configurator,
                               boolean copy) {
        for (Target target : subset) {
            printTestHeader(target, description);
            runTestsAndPrintResults(target, selected, configurator, copy);
        }
    }

    /**
     * Whether to enable {@link #runWithThreadingOff()}.
     */
    protected boolean supportsSingleThreadedExecution() {
        return false;
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
     * Determine whether the current platform is MacOS.
     * @return true if the current platform is MacOS, false otherwise.
     */
    protected static boolean isMac() {
        String OS = System.getProperty("os.name").toLowerCase();
        return OS.contains("mac");
    }

    /**
     * Determine whether the current platform is Linux.
     * @return true if the current platform is Linux, false otherwise.
     */
    protected static boolean isLinux() {
        String OS = System.getProperty("os.name").toLowerCase();
        return OS.contains("linux");
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
        System.setOut(new PrintStream(test.getOutputStream()));
        System.setErr(new PrintStream(test.getOutputStream()));
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
            test.reportErrors();
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
    private LFGeneratorContext configure(LFTest test, Configurator configurator, TestLevel level) throws IOException {
        var props = new Properties();
        var sysProps = System.getProperties();
        // Set the external-runtime-path property if it was specified.
        if (sysProps.containsKey("runtime")) {
            var rt = sysProps.get("runtime").toString();
            if (!rt.isEmpty()) {
                props.setProperty(BuildParm.EXTERNAL_RUNTIME_PATH.getKey(), rt);
                System.out.println("Using runtime: " + sysProps.get("runtime").toString());
            }
        } else {
            System.out.println("Using default runtime.");
        }

        var context = new MainContext(
            LFGeneratorContext.Mode.STANDALONE, CancelIndicator.NullImpl, (m, p) -> {}, props, true,
            fileConfig -> new DefaultErrorReporter()
        );

        var r = resourceSetProvider.get().getResource(
            URI.createFileURI(test.srcFile.toFile().getAbsolutePath()),
            true);

        if (r.getErrors().size() > 0) {
            test.result = Result.PARSE_FAIL;
            throw new AssertionError("Test did not parse correctly.");
        }

        fileAccess.setOutputPath(FileConfig.findPackageRoot(test.srcFile, s -> {}).resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
        test.context = context;
        test.fileConfig = new FileConfig(r, FileConfig.getSrcGenRoot(fileAccess), context.useHierarchicalBin());

        // Set the no-compile flag the test is not supposed to reach the build stage.
        if (level.compareTo(TestLevel.BUILD) < 0) {
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
    private void validate(LFTest test, IGeneratorContext context) {
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
        args.setProperty("build-type", "Test");
        args.setProperty("logging", "Debug");
    }


    /**
     * Invoke the code generator for the given test.
     * @param test The test to generate code for.
     */
    private GeneratorResult generateCode(LFTest test) {
        GeneratorResult result = GeneratorResult.NOTHING;
        if (test.fileConfig.resource != null) {
            generator.doGenerate(test.fileConfig.resource, fileAccess, test.context);
            result = test.context.getResult();
            if (generator.errorsOccurred()) {
                test.result = Result.CODE_GEN_FAIL;
                throw new AssertionError("Code generation unsuccessful.");
            }
        }
        return result;
    }


    /**
     * Given an indexed test, execute it and label the test as failing if it
     * did not execute, took too long to execute, or executed but exited with
     * an error code.
     */
    private void execute(LFTest test, GeneratorResult generatorResult) {
        final List<ProcessBuilder> pbList = getExecCommand(test, generatorResult);
        if (pbList.isEmpty()) {
            return;
        }
        try {
            for (ProcessBuilder pb : pbList) {
                var p = pb.start();
                var stdout = test.execLog.recordStdOut(p);
                var stderr = test.execLog.recordStdErr(p);

                var stdoutException = new AtomicReference<Throwable>(null);
                var stderrException = new AtomicReference<Throwable>(null);

                stdout.setUncaughtExceptionHandler((thread, throwable) -> stdoutException.set(throwable));
                stderr.setUncaughtExceptionHandler((thread, throwable) -> stderrException.set(throwable));

                if (!p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS)) {
                    stdout.interrupt();
                    stderr.interrupt();
                    p.destroyForcibly();
                    test.result = Result.TEST_TIMEOUT;
                    return;
                } else {
                    if (stdoutException.get() != null || stderrException.get() != null) {
                        test.result = Result.TEST_EXCEPTION;
                        test.execLog.buffer.setLength(0);
                        if (stdoutException.get() != null) {
                            test.execLog.buffer.append("Error during stdout handling:\n");
                            appendStackTrace(stdoutException.get(), test.execLog.buffer);
                        }
                        if (stderrException.get() != null) {
                            test.execLog.buffer.append("Error during stderr handling:\n");
                            appendStackTrace(stderrException.get(), test.execLog.buffer);
                        }
                        return;
                    }
                    if (p.exitValue() != 0) {
                        test.result = Result.TEST_FAIL;
                        test.exitValue = Integer.toString(p.exitValue());
                        return;
                    }
                }
            }
        } catch (Exception e) {
            test.result = Result.TEST_EXCEPTION;
            // Add the stack trace to the test output
            appendStackTrace(e, test.execLog.buffer);
            return;
        }
        test.result = Result.TEST_PASS;
        // clear the log if the test succeeded to free memory
        test.execLog.clear();
    }

    static private void appendStackTrace(Throwable t, StringBuffer buffer) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        t.printStackTrace(pw);
        buffer.append(sw);
    }

    /**
     * Return the content of the bash script used for testing docker option in federated execution.
     * @param dockerFiles A list of paths to docker files.
     * @param dockerComposeFilePath The path to the docker compose file.
     */
    private String getDockerRunScript(List<Path> dockerFiles, Path dockerComposeFilePath) {
        var dockerComposeCommand = DockerGeneratorBase.getDockerComposeCommand();
        StringBuilder shCode = new StringBuilder();
        shCode.append("#!/bin/bash\n");
        shCode.append("pids=\"\"\n");
        shCode.append(String.format("%s run -f %s --rm -T rti &\n",
            dockerComposeCommand, dockerComposeFilePath));
        shCode.append("pids+=\"$!\"\nsleep 3\n");
        for (Path dockerFile : dockerFiles) {
            var composeServiceName = dockerFile.getFileName().toString().replace(".Dockerfile", "");
            shCode.append(String.format("%s run -f %s --rm -T %s &\n",
                dockerComposeCommand,
                dockerComposeFilePath,
                composeServiceName));
            shCode.append("pids+=\" $!\"\n");
        }
        shCode.append("for p in $pids; do\n");
        shCode.append("    if wait $p; then\n");
        shCode.append("        :\n");
        shCode.append("    else\n");
        shCode.append("        exit 1\n");
        shCode.append("    fi\n");
        shCode.append("done\n");
        return shCode.toString();
    }

    /**
     * Returns true if docker exists, false otherwise.
     */
    private boolean checkDockerExists() {
        LFCommand checkCommand = LFCommand.get("docker", List.of("info"));
        return checkCommand.run() == 0;
    }

    /**
     * Return a list of ProcessBuilders used to test the docker option under non-federated execution.
     * See the following for references on the instructions called:
     * docker build: https://docs.docker.com/engine/reference/commandline/build/
     * docker run: https://docs.docker.com/engine/reference/run/
     * docker image: https://docs.docker.com/engine/reference/commandline/image/
     *
     * @param test The test to get the execution command for.
     */
    private List<ProcessBuilder> getNonfederatedDockerExecCommand(LFTest test) {
        if (!checkDockerExists()) {
            System.out.println(Message.MISSING_DOCKER);
            return List.of(new ProcessBuilder("exit", "1"));
        }
        var srcGenPath = test.fileConfig.getSrcGenPath();
        var dockerComposeFile = FileUtil.globFilesEndsWith(srcGenPath, "docker-compose.yml").get(0);
        var dockerComposeCommand = DockerGeneratorBase.getDockerComposeCommand();
        return List.of(new ProcessBuilder(dockerComposeCommand, "-f", dockerComposeFile.toString(), "rm", "-f"),
                       new ProcessBuilder(dockerComposeCommand, "-f", dockerComposeFile.toString(), "up", "--build"),
                       new ProcessBuilder(dockerComposeCommand, "-f", dockerComposeFile.toString(), "down", "--rmi", "local"));
    }

    /**
     * Return a list of ProcessBuilders used to test the docker option under federated execution.
     * @param test The test to get the execution command for.
     */
    private List<ProcessBuilder> getFederatedDockerExecCommand(LFTest test) {
        if (!checkDockerExists()) {
            System.out.println(Message.MISSING_DOCKER);
            return List.of(new ProcessBuilder("exit", "1"));
        }
        var srcGenPath = test.fileConfig.getSrcGenPath();
        List<Path> dockerFiles = FileUtil.globFilesEndsWith(srcGenPath, ".Dockerfile");
        try {
            File testScript = File.createTempFile("dockertest", null);
            testScript.deleteOnExit();
            if (!testScript.setExecutable(true)) {
                throw new IOException("Failed to make test script executable");
            }
            FileWriter fileWriter = new FileWriter(testScript.getAbsoluteFile(), true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
            var dockerComposeFile = FileUtil.globFilesEndsWith(srcGenPath, "docker-compose.yml").get(0);
            bufferedWriter.write(getDockerRunScript(dockerFiles, dockerComposeFile));
            bufferedWriter.close();
            return List.of(new ProcessBuilder(testScript.getAbsolutePath()));
        } catch (IOException e) {
            return List.of(new ProcessBuilder("exit", "1"));
        }
    }

    /**
     * Return a list of preconfigured ProcessBuilder(s) for the command(s)
     * that should be used to execute the test program.
     * @param test The test to get the execution command for.
     */
    private List<ProcessBuilder> getExecCommand(LFTest test, GeneratorResult generatorResult) {
        var srcBasePath = test.fileConfig.srcPkgPath.resolve("src");
        var relativePathName = srcBasePath.relativize(test.fileConfig.srcPath).toString();

        // special case to test docker file generation
        if (relativePathName.equalsIgnoreCase(TestCategory.DOCKER.getPath())) {
            return getNonfederatedDockerExecCommand(test);
        } else if (relativePathName.equalsIgnoreCase(TestCategory.DOCKER_FEDERATED.getPath())) {
            return getFederatedDockerExecCommand(test);
        } else {
            LFCommand command = generatorResult.getCommand();
            if (command == null) {
                test.result = Result.NO_EXEC_FAIL;
                test.issues.append("File: ").append(generatorResult.getExecutable()).append(System.lineSeparator());
            }
            return command == null ? List.of() : List.of(
                new ProcessBuilder(command.command()).directory(command.directory())
            );
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
                GeneratorResult result = GeneratorResult.NOTHING;
                if (level.compareTo(TestLevel.CODE_GEN) >= 0) {
                    result = generateCode(test);
                }
                if (level == TestLevel.EXECUTION) {
                    execute(test, result);
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
