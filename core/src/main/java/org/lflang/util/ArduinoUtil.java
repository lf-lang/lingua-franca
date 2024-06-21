package org.lflang.util;

import java.io.IOException;
import java.util.List;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.PlatformProperty;

/**
 * Utilities for Building using Arduino CLI.
 *
 * <p>We take in a Generator Context, Command Factory, and Error Reporter and make subsequent calls
 * to arduino-cli given a FileConfig and TargetConfig.
 *
 * <p>This should be used immediately after CodeGen to compile if the user provides a board type
 * within their LF file. If the user also provides a port with flash enabled, we will also attempt
 * to upload the compiled sketch directly to the board.
 */
public class ArduinoUtil {

  private LFGeneratorContext context;
  private GeneratorCommandFactory commandFactory;
  private MessageReporter messageReporter;

  public ArduinoUtil(
      LFGeneratorContext context,
      GeneratorCommandFactory commandFactory,
      MessageReporter messageReporter) {
    this.context = context;
    this.commandFactory = commandFactory;
    this.messageReporter = messageReporter;
  }

  /** Return true if arduino-cli exists, false otherwise. */
  private boolean checkArduinoCLIExists() {
    LFCommand checkCommand = LFCommand.get("arduino-cli", List.of("version"));
    return checkCommand != null && checkCommand.run() == 0;
  }

  /**
   * Generate an LF style command for Arduino compilation based on FQBN
   *
   * @param fileConfig
   * @param targetConfig
   * @return LFCommand to compile an Arduino program given an FQBN.
   * @throws IOException
   */
  private LFCommand arduinoCompileCommand(FileConfig fileConfig, TargetConfig targetConfig)
      throws IOException {
    if (!checkArduinoCLIExists()) {
      throw new IOException("Must have arduino-cli installed to auto-compile.");
    }

    var srcGenPath = fileConfig.getSrcGenPath();
    String board =
        targetConfig.get(PlatformProperty.INSTANCE).board().setByUser()
            ? targetConfig.get(PlatformProperty.INSTANCE).board().value()
            : "arduino:avr:leonardo";

    String compileDefs =
        (targetConfig.get(PlatformProperty.INSTANCE).board().value().contains("mbed")
                ? ""
                : "-DLF_SINGLE_THREADED")
            + " -DPLATFORM_ARDUINO"
            + " -DINITIAL_EVENT_QUEUE_SIZE=10"
            + " -DINITIAL_REACT_QUEUE_SIZE=10";

    return commandFactory.createCommand(
        "arduino-cli",
        List.of(
            "compile",
            "-b",
            board,
            "--build-property",
            "compiler.c.extra_flags=" + compileDefs,
            "--build-property",
            "compiler.cpp.extra_flags=" + compileDefs,
            srcGenPath.toString()),
        null);
  }

  /**
   * Compiles (and possibly auto-flashes) an Arduino program once code generation is completed.
   *
   * @param fileConfig
   * @param targetConfig
   */
  public void buildArduino(FileConfig fileConfig, TargetConfig targetConfig) {
    messageReporter.nowhere().info("Retrieving Arduino Compile Script");
    try {
      LFCommand command =
          arduinoCompileCommand(fileConfig, targetConfig); // Use ProcessBuilder for finer control.
      int retCode = 0;
      retCode = command.run(context.getCancelIndicator());
      if (retCode != 0 && context.getMode() == LFGeneratorContext.Mode.STANDALONE) {
        messageReporter.nowhere().error("arduino-cli failed with error code " + retCode);
        throw new IOException("arduino-cli failure");
      }
    } catch (IOException e) {
      Exceptions.sneakyThrow(e);
    }
    messageReporter
        .nowhere()
        .info(
            "SUCCESS: Compiling generated code for "
                + fileConfig.name
                + " finished with no errors.");
    if (targetConfig.get(PlatformProperty.INSTANCE).flash().value()) {
      if (targetConfig.get(PlatformProperty.INSTANCE).port().value() != null) {
        messageReporter.nowhere().info("Invoking flash command for Arduino");
        LFCommand flash =
            commandFactory.createCommand(
                "arduino-cli",
                List.of(
                    "upload",
                    "-b",
                    targetConfig.get(PlatformProperty.INSTANCE).board().value(),
                    "-p",
                    targetConfig.get(PlatformProperty.INSTANCE).port().value()),
                fileConfig.getSrcGenPath());
        if (flash == null) {
          messageReporter.nowhere().error("Could not create arduino-cli flash command.");
        }
        int flashRet = flash.run();
        if (flashRet != 0) {
          messageReporter
              .nowhere()
              .error("arduino-cli flash command failed with error code " + flashRet);
        } else {
          messageReporter.nowhere().info("SUCCESS: Flashed board using arduino-cli");
        }
      } else {
        messageReporter.nowhere().error("Need to provide a port on which to automatically flash.");
      }
    } else {
      messageReporter.nowhere().info("********");
      messageReporter
          .nowhere()
          .info(
              "To flash your program, run the following command to see information about the board"
                  + " you plugged in:\n\n"
                  + "\tarduino-cli board list\n\n"
                  + "Grab the FQBN and PORT from the command and run the following command in the"
                  + " generated sources directory:\n\n"
                  + "\tarduino-cli upload -b <FQBN> -p <PORT>\n");
    }
  }
}
