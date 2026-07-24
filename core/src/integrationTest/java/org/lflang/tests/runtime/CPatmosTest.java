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
 * <p>Runs under cycle-accurate {@code patemu} (not {@code pasim}): with the toolchain used in CI,
 * {@code pasim} is known to fail on binaries that {@code patemu} accepts. Physical busy-waits on
 * {@code patemu} are avoided for these smoke tests via {@code fast: true} in {@link
 * Configurators#makePatmosCompatible}.
 *
 * @ingroup Tests
 */
public class CPatmosTest extends TestBase {

  /** Cycle-accurate Patmos emulator used by CI smoke tests. */
  private static final String SIMULATOR = "patemu";

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
