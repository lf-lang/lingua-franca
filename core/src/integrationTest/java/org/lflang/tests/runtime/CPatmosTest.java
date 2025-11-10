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
 * @ingroup Tests
 */
public class CPatmosTest extends TestBase {

  public CPatmosTest() {
    super(Target.C);
  }

  @Override
  protected ProcessBuilder getExecCommand(LFTest test) throws TestError {
    final String SIMULATOR = "patemu";

    System.out.println("DEBUG: Entering getExecCommand");

    LFCommand command = test.getFileConfig().getCommand();
    if (command == null) {
      System.err.println("ERROR: Command is null");
      throw new TestError("File: " + test.getFileConfig().getExecutable(), Result.NO_EXEC_FAIL);
    }
    // Print which simulator command will be used
    ProcessBuilder whichSim = new ProcessBuilder("which", SIMULATOR);
    try {
      Process proc = whichSim.start();
      int exitCode = proc.waitFor();
      if (exitCode == 0) {
        try (java.io.BufferedReader reader =
            new java.io.BufferedReader(new java.io.InputStreamReader(proc.getInputStream()))) {
          String simLocation = reader.readLine();
          System.out.println("DEBUG: Simulator found at: " + simLocation);
        }
      } else {
        System.out.println("DEBUG: Simulator not found in PATH.");
      }
    } catch (Exception e) {
      System.out.println("DEBUG: Exception while locating simulator: " + e.getMessage());
    }

    List<String> fullCommand = new ArrayList<>();
    fullCommand.add(SIMULATOR); // Prepend simulator to the command
    fullCommand.addAll(command.command()); // Add the rest of the command

    System.out.println(
        "DEBUG: Full command constructed: "
            + fullCommand
            + " in directory: "
            + command.directory());

    // Run the full command to check if it executes correctly
    try {
      Process testProc =
          new ProcessBuilder(fullCommand).directory(command.directory()).inheritIO().start();
      int testExit = testProc.waitFor();
      System.out.println("DEBUG: Test command exited with code: " + testExit);
    } catch (Exception e) {
      System.out.println("DEBUG: Exception while running full command: " + e.getMessage());
    }

    // Create the ProcessBuilder
    ProcessBuilder processBuilder = new ProcessBuilder(fullCommand).directory(command.directory());

    // Add the directory containing simulator to the PATH environment variable
    String simPath = System.getProperty("user.home") + "/t-crest/local/bin";
    processBuilder
        .environment()
        .put("PATH", simPath + ":" + processBuilder.environment().get("PATH"));

    System.out.println(
        "DEBUG: ProcessBuilder command: "
            + processBuilder.command()
            + " directory: "
            + processBuilder.directory());

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
