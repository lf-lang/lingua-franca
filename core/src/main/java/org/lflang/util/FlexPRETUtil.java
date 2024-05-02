package org.lflang.util;

import java.io.IOException;
import java.util.Collections;

import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;

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

  public void flashTarget(FileConfig fileConfig) {
    var binPath = fileConfig.binPath;
    var exeName = fileConfig.name;
    
    /**
     * FlexPRET's SDK generates a runnable bash script. We just need to run it.
     */
    var cmd = binPath + "/" + exeName;
    var args = Collections.<String> emptyList();
    
    try {
      commandFactory.setVerbose();
      LFCommand flash = commandFactory.createCommand(cmd, args);
      flash.setVerbose();
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
