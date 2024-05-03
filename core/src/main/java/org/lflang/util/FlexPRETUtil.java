package org.lflang.util;

import java.io.IOException;
import java.util.Collections;

import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.TargetConfig;

public class FlexPRETUtil {
  private LFGeneratorContext context;
  private GeneratorCommandFactory commandFactory;
  private MessageReporter messageReporter;
  
  public FlexPRETUtil(
      LFGeneratorContext context,
      GeneratorCommandFactory commandFactory,
      MessageReporter messageReporter) {
    this.context = context;
    this.commandFactory = commandFactory;
    this.messageReporter = messageReporter;
  }

  public void flashTarget(FileConfig fileConfig, TargetConfig config) {
    var binPath = fileConfig.binPath;
    var exeName = fileConfig.name;
    
    /**
     * FlexPRET's SDK generates a runnable bash script. We just need to run it.
     */
    var cmd = binPath + "/" + exeName;
    var args = Collections.<String> emptyList();
    
    try {
      LFCommand flash = commandFactory.createCommand(cmd, args);
      
      /**
       * Check if environment variable FP_SDK_FPGA_INTERFACE_PROGRAM is set.
       * If it is set, we should let the user know it is not supported by LF.
       * 
       * We cannot support it because FP_SDK_FPGA_INTERFACE_PROGRAM will lead to
       * a [Y/n] prompt, and the LFCommands do not have a way of getting the
       * prompt response.
       * 
       * When FP_SDK_FPGA_INTERFACE_PROGRAM is not set, there is no prompt.
       */
      var envInterfaceProgram = System.getenv("FP_SDK_FPGA_INTERFACE_PROGRAM");
      
      /**
       * FP_SDK_FPGA_INTERFACE_PROGRAM is only relevant when the user has specified
       * board = fpga and flash = true. So only print the info message if that
       * is the case.
       */
      var platform = config.get(PlatformProperty.INSTANCE);
      var interfaceProgramRelevant = platform.board().value().equals("fpga")
                                  && platform.flash().value() == true;
 
      if (interfaceProgramRelevant && envInterfaceProgram != null) {
        messageReporter
              .nowhere()
              .info("FP_SDK_FPGA_INTERFACE_PROGRAM not supported by LF. It will have no effect.");
      }

      /**
       * Set the variable to nothing for the flash command, which will avoid the 
       * [Y/n] prompt
       */
      flash.setEnvironmentVariable("FP_SDK_FPGA_INTERFACE_PROGRAM", "");
      if (flash != null) {
        int ret = flash.run(context.getCancelIndicator());
        if (ret != 0) {
          messageReporter
              .nowhere()
              .error("Command " + cmd + " failed with exit code: " + ret);
          throw new IOException("Could not flash");
        }  
      }
    } catch (IOException e) {
      Exceptions.sneakyThrow(e);
    } catch (NullPointerException e) {
      Exceptions.sneakyThrow(e);
    }
  }
}
