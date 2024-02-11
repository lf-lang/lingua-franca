package org.lflang.tests;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.resource.Resource.Diagnostic;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.util.RuntimeIOException;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.lflang.DefaultMessageReporter;
import org.lflang.FileConfig;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.generator.GeneratorArguments;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.MainContext;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.tests.Configurators.Configurator;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers.Transformer;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

/**
 * Base class for test classes that define tests that parse and build LF files from the {@link
 * TestRegistry}.
 *
 * @author Marten Lohstroh
 */
public abstract class TestBase extends LfInjectedTestBase {

  @Inject IResourceValidator validator;
  @Inject LFGenerator generator;
  @Inject JavaIoFileSystemAccess fileAccess;
  @Inject Provider<ResourceSet> resourceSetProvider;

  @Inject TestRegistry testRegistry;

  /** Execution timeout enforced for all tests. */
  private static final long MAX_EXECUTION_TIME_SECONDS = 300;

  /** Content separator used in test output, 78 characters wide. */
  public static final String THIN_LINE =
      "------------------------------------------------------------------------------"
          + System.lineSeparator();

  /** Content separator used in test output, 78 characters wide. */
  public static final String THICK_LINE =
      "=============================================================================="
          + System.lineSeparator();

  /** The targets for which to run the tests. */
  private final List<Target> targets;

  /**
   * An enumeration of test levels.
   *
   * @author Marten Lohstroh
   */
  public enum TestLevel {
    BUILD,
    EXECUTION
  }

  /**
   * Static function for converting a path to its associated test level.
   *
   * @author Anirudh Rengarajan
   */
  public static TestLevel pathToLevel(Path path) {
    path = path.getParent();
    while (path != null) {
      final var name = path.getFileName();
      for (var category : TestCategory.values()) {
        if (name != null && name.toString().equalsIgnoreCase(category.name())) {
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
   * @author Marten Lohstroh
   */
  public static class Message {
    /* Reasons for not running tests. */
    public static final String NO_WINDOWS_SUPPORT = "Not (yet) supported on Windows.";
    public static final String NO_SINGLE_THREADED_SUPPORT =
        "Target does not support single-threaded execution.";
    public static final String NO_FEDERATION_SUPPORT =
        "Target does not support federated execution.";
    public static final String NO_ENCLAVE_SUPPORT = "Targeet does not support the enclave feature.";
    public static final String NO_DOCKER_SUPPORT = "Target does not support the 'docker' property.";
    public static final String NO_DOCKER_TEST_SUPPORT = "Docker tests are only supported on Linux.";

    /* Descriptions of collections of tests. */
    public static final String DESC_SERIALIZATION = "Run serialization tests.";
    public static final String DESC_BASIC = "Run basic tests.";
    public static final String DESC_GENERICS = "Run generics tests.";
    public static final String DESC_MULTIPORT = "Run multiport tests.";
    public static final String DESC_AS_FEDERATED = "Run non-federated tests in federated mode.";
    public static final String DESC_FEDERATED = "Run federated tests.";
    public static final String DESC_FEDERATED_WITH_RUST_RTI = "Run federated tests with Rust RTI.";
    public static final String DESC_DOCKER = "Run docker tests.";
    public static final String DESC_DOCKER_FEDERATED = "Run docker federated tests.";
    public static final String DESC_ENCLAVE = "Run enclave tests.";
    public static final String DESC_CONCURRENT = "Run concurrent tests.";
    public static final String DESC_TARGET_SPECIFIC = "Run target-specific tests";
    public static final String DESC_ARDUINO = "Running Arduino tests.";
    public static final String DESC_ZEPHYR = "Running Zephyr tests.";
    public static final String DESC_AS_CCPP = "Running C tests as CCpp.";
    public static final String DESC_SINGLE_THREADED =
        "Run non-concurrent and non-federated tests with in single-threaded mode.";
    public static final String DESC_SCHED_SWAPPING = "Running with non-default runtime scheduler ";
    public static final String DESC_ROS2 = "Running tests using ROS2.";
    public static final String DESC_MODAL = "Run modal reactor tests.";
    public static final String DESC_VERIFIER = "Run verifier tests.";
  }

  /** Constructor for test classes that test a single target. */
  protected TestBase(Target first) {
    this(Collections.singletonList(first));
  }

  /** Special ctor for the code coverage test */
  protected TestBase(List<Target> targets) {
    assertFalse(targets.isEmpty(), "empty target list");
    this.targets = Collections.unmodifiableList(targets);
  }

  /**
   * Run selected tests for a given target and configurator up to the specified level.
   *
   * @param target The target to run tests for.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected final void runTestsAndPrintResults(
      Target target,
      Predicate<TestCategory> selected,
      TestLevel level,
      Transformer transformer,
      Configurator configurator,
      boolean copy) {
    var categories = Arrays.stream(TestCategory.values()).filter(selected).toList();
    for (var category : categories) {
      System.out.println(category.getHeader());
      var tests = testRegistry.getRegisteredTests(target, category, copy);
      try {
        validateAndRun(tests, transformer, configurator, level);
      } catch (IOException e) {
        throw new RuntimeIOException(e);
      }
      System.out.println(testRegistry.getCoverageReport(target, category));
      checkAndReportFailures(tests);
    }
  }

  /**
   * Run selected tests for a given target and configurator up to the specified level.
   *
   * @param target The target to run tests for.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected final void runTestsAndPrintResultsWithRustRti(
      Target target,
      Predicate<TestCategory> selected,
      TestLevel level,
      Transformer transformer,
      Configurator configurator,
      boolean copy) {
    var categories = Arrays.stream(TestCategory.values()).filter(selected).toList();
    for (var category : categories) {
      System.out.println(category.getHeader());
      var tests = testRegistry.getRegisteredTests(target, category, copy);
      try {
        validateAndRunWithRustRti(tests, transformer, configurator, level);
      } catch (IOException e) {
        throw new RuntimeIOException(e);
      }
      System.out.println(testRegistry.getCoverageReport(target, category));
      checkAndReportFailures(tests);
    }
  }

  /**
   * Run tests in the given selection for all targets enabled in this class.
   *
   * @param description A string that describes the collection of tests.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected void runTestsForTargets(
      String description,
      Predicate<TestCategory> selected,
      Transformer transformer,
      Configurator configurator,
      TestLevel level,
      boolean copy) {
    for (Target target : this.targets) {
      runTestsFor(List.of(target), description, selected, transformer, configurator, level, copy);
    }
  }

  /**
   * Run tests in the given selection for all targets enabled in this class.
   *
   * @param description A string that describes the collection of tests.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected void runTestsForTargetsWithRustRti(
      String description,
      Predicate<TestCategory> selected,
      Transformer transformer,
      Configurator configurator,
      TestLevel level,
      boolean copy) {
    for (Target target : this.targets) {
      runTestsForRustRti(
          List.of(target), description, selected, transformer, configurator, level, copy);
    }
  }

  /**
   * Run tests in the given selection for a subset of given targets.
   *
   * @param subset The subset of targets to run the selected tests for.
   * @param description A string that describes the collection of tests.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected void runTestsFor(
      List<Target> subset,
      String description,
      Predicate<TestCategory> selected,
      Transformer transformer,
      Configurator configurator,
      TestLevel level,
      boolean copy) {
    for (Target target : subset) {
      printTestHeader(target, description);
      runTestsAndPrintResults(target, selected, level, transformer, configurator, copy);
    }
  }

  /**
   * Run tests in the given selection for a subset of given targets.
   *
   * @param subset The subset of targets to run the selected tests for.
   * @param description A string that describes the collection of tests.
   * @param selected A predicate that given a test category returns whether it should be included in
   *     this test run or not.
   * @param configurator A procedure for configuring the tests.
   * @param copy Whether to work on copies of tests in the test. registry.
   */
  protected void runTestsForRustRti(
      List<Target> subset,
      String description,
      Predicate<TestCategory> selected,
      Transformer transformer,
      Configurator configurator,
      TestLevel level,
      boolean copy) {
    for (Target target : subset) {
      printTestHeader(target, description);
      runTestsAndPrintResultsWithRustRti(target, selected, level, transformer, configurator, copy);
    }
  }

  /** Whether to enable threading. */
  protected boolean supportsSingleThreadedExecution() {
    return false;
  }

  /**
   * Determine whether the current platform is Windows.
   *
   * @return true if the current platform is Windwos, false otherwise.
   */
  protected static boolean isWindows() {
    String OS = System.getProperty("os.name").toLowerCase();
    return OS.contains("win");
  }

  /**
   * Determine whether the current platform is MacOS.
   *
   * @return true if the current platform is MacOS, false otherwise.
   */
  protected static boolean isMac() {
    String OS = System.getProperty("os.name").toLowerCase();
    return OS.contains("mac");
  }

  /**
   * Determine whether the current platform is Linux.
   *
   * @return true if the current platform is Linux, false otherwise.
   */
  protected static boolean isLinux() {
    String OS = System.getProperty("os.name").toLowerCase();
    return OS.contains("linux");
  }

  /**
   * Run a test, print results on stderr.
   *
   * @param test Test case.
   * @param testClass The test class that will execute the test. This is target-specific, it may
   *     provide some target-specific configuration. We pass a class and not a new instance because
   *     this method needs to ensure the object is properly injected, and so, it needs to control
   *     its entire lifecycle.
   * @param level Level to which to run the test.
   */
  public static void runSingleTestAndPrintResults(
      LFTest test, Class<? extends TestBase> testClass, TestLevel level) {
    Injector injector =
        new LFStandaloneSetup(new LFRuntimeModule()).createInjectorAndDoEMFRegistration();
    TestBase runner;
    try {
      @SuppressWarnings("unchecked")
      Constructor<? extends TestBase> constructor =
          (Constructor<? extends TestBase>) testClass.getConstructors()[0];
      runner = constructor.newInstance();
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
      throw new IllegalStateException(e);
    }
    injector.injectMembers(runner);

    Set<LFTest> tests = Set.of(test);
    try {
      runner.validateAndRun(tests, Transformers::noChanges, Configurators::noChanges, level);
    } catch (IOException e) {
      throw new RuntimeIOException(e);
    }
    checkAndReportFailures(tests);
  }

  /**
   * Print a header that describes a collection of tests.
   *
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
   * Iterate over given tests and evaluate their outcome, report errors if there are any.
   *
   * @param tests The tests to inspect the results of.
   */
  private static void checkAndReportFailures(Set<LFTest> tests) {
    var passed = tests.stream().filter(LFTest::hasPassed).toList();
    var s = new StringBuffer();
    s.append(THIN_LINE);
    s.append(String.format("Passing: %d/%d%n", passed.size(), tests.size()));
    s.append(THIN_LINE);
    passed.forEach(
        test ->
            s.append("Passed: ")
                .append(test)
                .append(
                    String.format(
                        " in %.2f seconds%n", test.getExecutionTimeNanoseconds() / 1.0e9)));
    s.append(THIN_LINE);
    System.out.print(s);

    for (var test : tests) {
      test.reportErrors();
    }
    for (LFTest lfTest : tests) {
      assertTrue(lfTest.hasPassed());
    }
  }

  /**
   * Prepare a test by applying the given transformer and configurator. If either of them was not
   * applied successfully, throw an AssertionError.
   *
   * @param test the test to configure.
   * @param transformer The transformer to apply to the test.
   * @param configurator The configurator to apply to the test.
   */
  private void prepare(LFTest test, Transformer transformer, Configurator configurator)
      throws TestError {

    var resource = FileConfig.getResource(test.getSrcPath().toFile(), resourceSetProvider);

    if (resource.getErrors().size() > 0) {
      String message =
          resource.getErrors().stream()
              .map(Diagnostic::toString)
              .collect(Collectors.joining(System.lineSeparator()));
      throw new TestError(message, Result.PARSE_FAIL);
    }

    fileAccess.setOutputPath(
        FileConfig.findPackageRoot(test.getSrcPath(), s -> {})
            .resolve(FileConfig.DEFAULT_SRC_GEN_DIR)
            .toString());
    var context =
        new MainContext(
            LFGeneratorContext.Mode.STANDALONE,
            CancelIndicator.NullImpl,
            (m, p) -> {},
            getGeneratorArguments(),
            resource,
            fileAccess,
            fileConfig -> new DefaultMessageReporter());

    // Update the test by applying the transformation.
    if (transformer != null) {
      if (!transformer.transform(resource)) {
        throw new TestError("Test transformation unsuccessful.", Result.TRANSFORM_FAIL);
      }
    }

    // Reload the context because properties may have changed as part of the transformation.
    test.loadContext(context);

    applyDefaultConfiguration(test.getContext().getTargetConfig());

    // Update the configuration using the supplied configurator.
    if (configurator != null) {
      if (!configurator.configure(test.getContext().getTargetConfig())) {
        throw new TestError("Test configuration unsuccessful.", Result.CONFIG_FAIL);
      }
    }
  }

  /** Return a URI pointing to an external runtime if there is one, {@code null} otherwise. */
  private URI getExternalRuntimeUri() {
    var sysProps = System.getProperties();
    URI uri = null;
    // Set the external-runtime-path property if it was specified.
    if (sysProps.containsKey("runtime")) {
      var rt = sysProps.get("runtime").toString();
      if (!rt.isEmpty()) {
        try {
          uri = new URI(rt);
        } catch (URISyntaxException e) {
          throw new RuntimeException(e);
        }
        System.out.println("Using runtime: " + sysProps.get("runtime").toString());
      }
    } else {
      System.out.println("Using default runtime.");
    }
    return uri;
  }

  /** Return generator arguments suitable for testing. */
  protected GeneratorArguments getGeneratorArguments() {
    return new GeneratorArguments(
        false,
        getExternalRuntimeUri(), // Passed in as parameter to Gradle.
        true, // To avoid name clashes in the bin directory.
        null,
        false,
        false,
        null,
        List.of());
  }

  /** Validate the given test. Throw an TestError if validation failed. */
  private void validate(LFTest test) throws TestError {
    // Validate the resource and store issues in the test object.
    try {
      var context = test.getContext();
      var issues =
          validator.validate(
              context.getFileConfig().resource, CheckMode.ALL, context.getCancelIndicator());
      if (issues != null && !issues.isEmpty()) {
        if (issues.stream().anyMatch(it -> it.getSeverity() == Severity.ERROR)) {
          String message =
              issues.stream()
                  .map(Objects::toString)
                  .collect(Collectors.joining(System.lineSeparator()));
          throw new TestError(message, Result.VALIDATE_FAIL);
        }
      }
    } catch (TestError e) {
      throw e;
    } catch (Throwable e) {
      throw new TestError("Exception during validation.", Result.VALIDATE_FAIL, e);
    }
  }

  /** Adjust target configuration for all runs of this test class. */
  protected void applyDefaultConfiguration(TargetConfig config) {
    if (!config.isSet(BuildTypeProperty.INSTANCE)) {
      config.set(BuildTypeProperty.INSTANCE, BuildType.TEST);
    }
    LoggingProperty.INSTANCE.override(config, LogLevel.DEBUG);
  }

  /**
   * Invoke the code generator for the given test.
   *
   * @param test The test to generate code for.
   */
  private void generateCode(LFTest test) throws TestError {
    if (test.getFileConfig().resource == null) {
      test.getContext().finish(GeneratorResult.NOTHING);
    }
    try {
      generator.doGenerate(test.getFileConfig().resource, fileAccess, test.getContext());
    } catch (Throwable e) {
      throw new TestError("Code generation unsuccessful.", Result.CODE_GEN_FAIL, e);
    }
    if (generator.errorsOccurred()) {
      throw new TestError("Code generation unsuccessful.", Result.CODE_GEN_FAIL);
    }
  }

  /**
   * Invoke the code generator for the given test.
   *
   * @param test The test to generate code for.
   */
  private void generateCodeForRustRti(LFTest test) throws TestError {
    if (test.getFileConfig().resource == null) {
      test.getContext().finish(GeneratorResult.NOTHING);
    }
    try {
      generator.doGenerateForRustRTI(test.getFileConfig().resource, fileAccess, test.getContext());
    } catch (Throwable e) {
      throw new TestError("Code generation unsuccessful.", Result.CODE_GEN_FAIL, e);
    }
    if (generator.errorsOccurred()) {
      throw new TestError("Code generation unsuccessful.", Result.CODE_GEN_FAIL);
    }
  }

  /**
   * Given an indexed test, execute it and label the test as failing if it did not execute, took too
   * long to execute, or executed but exited with an error code.
   */
  private void execute(LFTest test) throws TestError {
    final var pb = getExecCommand(test);
    try {
      var p = pb.start();
      var stdout = test.recordStdOut(p);
      var stderr = test.recordStdErr(p);

      var stdoutException = new AtomicReference<Throwable>(null);
      var stderrException = new AtomicReference<Throwable>(null);

      stdout.setUncaughtExceptionHandler((thread, throwable) -> stdoutException.set(throwable));
      stderr.setUncaughtExceptionHandler((thread, throwable) -> stderrException.set(throwable));

      stderr.start();
      stdout.start();
      long t0 = System.nanoTime();
      var timeout = !p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS);
      test.setExecutionTimeNanoseconds(System.nanoTime() - t0);
      stdout.interrupt();
      stderr.interrupt();
      if (timeout) {
        p.destroy();
        throw new TestError(Result.TEST_TIMEOUT);
      } else {
        if (stdoutException.get() != null || stderrException.get() != null) {
          StringBuilder sb = new StringBuilder();
          if (stdoutException.get() != null) {
            sb.append("Error during stdout handling:%n");
            sb.append(stackTraceToString(stdoutException.get()));
          }
          if (stderrException.get() != null) {
            sb.append("Error during stderr handling:%n");
            sb.append(stackTraceToString(stderrException.get()));
          }
          throw new TestError(sb.toString(), Result.TEST_EXCEPTION);
        }
        if (p.exitValue() != 0) {
          String message = "Exit code: " + p.exitValue();
          if (p.exitValue() == 139) {
            // The java ProcessBuilder and Process interface does not allow us to reliably retrieve
            // stderr and stdout
            // from a process that segfaults. We can only print a message indicating that the putput
            // is incomplete.
            message +=
                System.lineSeparator()
                    + "This exit code typically indicates a segfault. In this case, the execution"
                    + " output is likely missing or incomplete.";
          }
          throw new TestError(message, Result.TEST_FAIL);
        }
      }
    } catch (TestError e) {
      throw e;
    } catch (Throwable e) {
      e.printStackTrace();
      throw new TestError("Exception during test execution.", Result.TEST_EXCEPTION, e);
    }
  }

  public static String stackTraceToString(Throwable t) {
    StringWriter sw = new StringWriter();
    PrintWriter pw = new PrintWriter(sw);
    t.printStackTrace(pw);
    pw.flush();
    pw.close();
    return sw.toString();
  }

  /** Bash script that is used to execute docker tests. */
  private static final String DOCKER_RUN_SCRIPT =
      """
            #!/bin/bash

            # exit when any command fails
            set -e

            docker compose -f "$1" rm -f
            docker compose -f "$1" up --build | tee docker_log.txt
            docker compose -f "$1" down --rmi local

            errors=`grep -E "exited with code [1-9]" docker_log.txt | cat`
            rm docker_log.txt

            if [[ $errors ]]; then
                echo "===================================================================="
                echo "ERROR: One or multiple containers exited with a non-zero exit code."
                echo "       See the log above for details. The following containers failed:"
                echo $errors
                exit 1
            fi

            exit 0
            """;

  /** Path to a bash script containing DOCKER_RUN_SCRIPT. */
  private static Path dockerRunScript = null;

  /**
   * Return the path to a bash script containing DOCKER_RUN_SCRIPT.
   *
   * <p>If the script does not yet exist, it is created.
   */
  private static synchronized Path getDockerRunScript() throws TestError {
    if (dockerRunScript != null) {
      return dockerRunScript;
    }

    try {
      var file = File.createTempFile("run_docker_test", "sh");
      file.deleteOnExit();
      file.setExecutable(true);
      var path = file.toPath();
      try (BufferedWriter writer = Files.newBufferedWriter(path)) {
        writer.write(DOCKER_RUN_SCRIPT);
      }
      dockerRunScript = path;
    } catch (IOException e) {
      throw new TestError("IO Error during test preparation.", Result.TEST_EXCEPTION, e);
    }

    return dockerRunScript;
  }

  /** Throws TestError if docker does not exist. Does nothing otherwise. */
  private void checkDockerExists() throws TestError {
    if (LFCommand.get("docker", List.of()) == null) {
      throw new TestError("Executable 'docker' not found", Result.NO_EXEC_FAIL);
    }
    if (LFCommand.get("docker-compose", List.of()) == null) {
      throw new TestError("Executable 'docker-compose' not found", Result.NO_EXEC_FAIL);
    }
  }

  /**
   * Return a ProcessBuilder used to test the docker execution.
   *
   * @param test The test to get the execution command for.
   */
  private ProcessBuilder getDockerExecCommand(LFTest test) throws TestError {
    checkDockerExists();
    var srcGenPath = test.getFileConfig().getSrcGenPath();
    var dockerComposeFile = FileUtil.globFilesEndsWith(srcGenPath, "docker-compose.yml").get(0);
    return new ProcessBuilder(getDockerRunScript().toString(), dockerComposeFile.toString());
  }

  /**
   * Return a preconfigured ProcessBuilder for executing the test program.
   *
   * @param test The test to get the execution command for.
   */
  private ProcessBuilder getExecCommand(LFTest test) throws TestError {

    var srcBasePath = test.getFileConfig().srcPkgPath.resolve("src");
    var relativePathName = srcBasePath.relativize(test.getFileConfig().srcPath).toString();

    // special case to test docker file generation
    if (relativePathName.equalsIgnoreCase(TestCategory.DOCKER.getPath())
        || relativePathName.equalsIgnoreCase(TestCategory.DOCKER_FEDERATED.getPath())) {
      return getDockerExecCommand(test);
    } else {
      LFCommand command = test.getFileConfig().getCommand();
      if (command == null) {
        throw new TestError("File: " + test.getFileConfig().getExecutable(), Result.NO_EXEC_FAIL);
      }
      return new ProcessBuilder(command.command()).directory(command.directory());
    }
  }

  /**
   * Validate and run the given tests, using the specified configuratator and level.
   *
   * <p>While performing tests, this method prints a header that reaches completion once all tests
   * have been run.
   *
   * @param tests A set of tests to run.
   * @param transformer A procedure for transforming the tests.
   * @param configurator A procedure for configuring the tests.
   * @param level The level of testing.
   * @throws IOException If initial file configuration fails
   */
  private void validateAndRun(
      Set<LFTest> tests, Transformer transformer, Configurator configurator, TestLevel level)
      throws IOException {
    var done = 1;

    System.out.println(THICK_LINE);

    for (var test : tests) {
      System.out.println(
          "Running: " + test.toString() + " (" + (int) (done / (float) tests.size() * 100) + "%)");
      try {
        test.redirectOutputs();
        prepare(test, transformer, configurator);
        validate(test);
        generateCode(test);
        if (level == TestLevel.EXECUTION) {
          execute(test);
        }
        test.markPassed();
      } catch (TestError e) {
        test.handleTestError(e);
      } catch (Throwable e) {
        test.handleTestError(
            new TestError("Unknown exception during test execution", Result.TEST_EXCEPTION, e));
      } finally {
        test.restoreOutputs();
      }
      done++;
    }

    System.out.print(System.lineSeparator());
  }

  /**
   * Validate and run the given tests, using the specified configuratator and level.
   *
   * <p>While performing tests, this method prints a header that reaches completion once all tests
   * have been run.
   *
   * @param tests A set of tests to run.
   * @param transformer A procedure for transforming the tests.
   * @param configurator A procedure for configuring the tests.
   * @param level The level of testing.
   * @throws IOException If initial file configuration fails
   */
  private void validateAndRunWithRustRti(
      Set<LFTest> tests, Transformer transformer, Configurator configurator, TestLevel level)
      throws IOException {
    var done = 1;

    System.out.println(THICK_LINE);

    for (var test : tests) {
      System.out.println(
          "Running: " + test.toString() + " (" + (int) (done / (float) tests.size() * 100) + "%)");
      try {
        test.redirectOutputs();
        prepare(test, transformer, configurator);
        validate(test);
        generateCodeForRustRti(test);
        if (level == TestLevel.EXECUTION) {
          execute(test);
        }
        test.markPassed();
      } catch (TestError e) {
        test.handleTestError(e);
      } catch (Throwable e) {
        test.handleTestError(
            new TestError("Unknown exception during test execution", Result.TEST_EXCEPTION, e));
      } finally {
        test.restoreOutputs();
      }
      done++;
    }

    System.out.print(System.lineSeparator());
  }
}
