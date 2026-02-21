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

  /** Test that --period and --expected override timer behavior. */
  @Test
  public void testCommandLineParameterOverride() {
    cliArgs = List.of("--period", "500", "msec", "--expected", "11");
    Path testFile = Path.of("test/C/src/CommandLineParam.lf").toAbsolutePath();
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
    cliArgs = List.of("--execution_time", "10", "msec", "--deadline_time", "500", "msec",
        "--expect_violation", "0");
    Path testFile = Path.of("test/C/src/CommandLineDeadline.lf").toAbsolutePath();
    LFTest test = new LFTest(testFile);
    runSingleTestAndPrintResults(test, CCliTest.class, TestLevel.EXECUTION);
  }
}
