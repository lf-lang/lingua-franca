package org.lflang.tests.runtime;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.LFTest;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestError;
import org.lflang.util.LFCommand;

/**
 * Integration tests for command-line parameter override functionality.
 *
 * <p>Verifies that main reactor parameters can be overridden via command-line arguments and that
 * the overridden values propagate correctly to child reactors, including timers and deadlines.
 *
 * @author Edward A. Lee
 */
public class CCliTest extends TestBase {

  private static List<String> cliArgs = List.of();

  public CCliTest() {
    super(Target.C);
  }

  @Override
  protected ProcessBuilder getExecCommand(LFTest test) throws TestError {
    LFCommand command = test.getFileConfig().getCommand();
    if (command == null) {
      throw new TestError("File: " + test.getFileConfig().getExecutable(), Result.NO_EXEC_FAIL);
    }
    List<String> cmdList = new ArrayList<>(command.command());
    cmdList.addAll(cliArgs);
    return new ProcessBuilder(cmdList).directory(command.directory());
  }

  /**
   * Test that --period, --expected, and --value override timer behavior and double parameter.
   * With defaults (period=1s, expected=6, value=3.14159), overriding to period=500ms changes
   * the firing count to 11, and overriding value=2.71828 changes the double sent on each output.
   */
  @Test
  public void testCommandLineParameterOverride() {
    cliArgs = List.of("--period", "500", "msec", "--expected", "11", "--value", "2.71828");
    Path testFile = Path.of("test/C/src/CommandLineParam.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }

  /**
   * Test that --min_delay and --min_spacing override action timing. With defaults (min_delay=1ns,
   * min_spacing=10ns), overriding to min_delay=10us and min_spacing=100us changes when the action
   * fires. The test verifies that elapsed times match the overridden values.
   */
  @Test
  public void testCommandLineActionOverride() {
    cliArgs = List.of("--min_delay", "10", "us", "--min_spacing", "100", "us");
    Path testFile = Path.of("test/C/src/CommandLineAction.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }

  /**
   * Test that --deadline_time and --execution_time override deadline behavior. With defaults
   * (execution_time=100ms, deadline_time=50ms) a deadline violation occurs. The CLI override
   * sets execution_time=10ms and deadline_time=200ms, so NO violation occurs, proving both
   * overrides took effect.
   */
  @Test
  public void testCommandLineDeadlineOverride() {
    cliArgs =
        List.of(
            "--execution_time",
            "10",
            "msec",
            "--deadline_time",
            "500",
            "msec",
            "--expect_violation",
            "0");
    Path testFile = Path.of("test/C/src/CommandLineDeadline.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }

  /**
   * Test that --value and --use_default override string and bool parameters.
   * With default use_default=false and value="Hello, world!", overriding value to "Goodbye!"
   * should cause the Print reactor to expect "Goodbye!".
   * Running with --use_default true should ignore the value parameter and use the hardcoded default.
   */
  @Test
  public void testCommandLineStringBoolOverride() {
    cliArgs = List.of("--value", "Goodbye!", "--use_default", "false");
    Path testFile = Path.of("test/C/src/CommandLineStringBool.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }

  /**
   * Test that CLI overrides propagate through the launch script in federated execution. With
   * defaults (execution_time=100ms, deadline_time=50ms) a deadline violation occurs. The CLI
   * override sets execution_time=10ms and deadline_time=500ms with expect_violation=0, so
   * NO violation occurs, proving the launch script forwards arguments to federates.
   */
  @Test
  public void testCommandLineDeadlineFederatedOverride() {
    cliArgs =
        List.of(
            "--execution_time",
            "10",
            "msec",
            "--deadline_time",
            "500",
            "msec",
            "--expect_violation",
            "0");
    Path testFile =
        Path.of("test/C/src/federated/CommandLineDeadlineFederated.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }
}
