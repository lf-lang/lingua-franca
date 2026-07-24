package org.lflang.tests.runtime;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.LFTest;
import org.lflang.tests.LFTest.Result;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestError;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;
import org.lflang.util.LFCommand;

/**
 * Collection of tests for the Patmos target.
 *
 * <p>Uses {@code pasim} (ISA simulator) rather than cycle-accurate {@code patemu}. Busy-waiting for
 * physical time on {@code patemu} is pathologically slow in CI and previously appeared as hangs
 * unless sleep loops performed I/O (e.g. debug {@code printf}s).
 *
 * @ingroup Tests
 */
public class CPatmosTest extends TestBase {

  /** Patmos ISA simulator; prefer over cycle-accurate {@code patemu} for smoke tests. */
  private static final String SIMULATOR = "pasim";

  public CPatmosTest() {
    super(Target.C);
  }

  @Override
  protected ProcessBuilder getExecCommand(LFTest test) throws TestError {
    LFCommand command = test.getFileConfig().getCommand();
    if (command == null) {
      throw new TestError("File: " + test.getFileConfig().getExecutable(), Result.NO_EXEC_FAIL);
    }

    List<String> fullCommand = new ArrayList<>();
    fullCommand.add(SIMULATOR);
    fullCommand.addAll(command.command());

    ProcessBuilder processBuilder = new ProcessBuilder(fullCommand).directory(command.directory());

    // Patmos tools are installed under ~/t-crest/local/bin by the CI setup action.
    String simPath = System.getProperty("user.home") + "/t-crest/local/bin";
    processBuilder
        .environment()
        .put("PATH", simPath + ":" + processBuilder.environment().get("PATH"));

    return processBuilder;
  }

  @Test
  public void runPatmosUnthreadedTests() {
    Assumptions.assumeTrue(isLinux(), "Patmos tests only run on Linux");
    super.runTestsFor(
        List.of(Target.C),
        Message.DESC_PATMOS,
        TestCategory.PATMOS::equals,
        Transformers::noChanges,
        Configurators::makePatmosCompatibleUnthreaded,
        TestLevel.EXECUTION,
        false);
  }
}
