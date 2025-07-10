package org.lflang.util;

import java.util.Collections;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;
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

    /** FlexPRET's SDK generates a runnable bash script. We just need to run it. */
    var cmd = binPath + "/" + exeName;
    var args = Collections.<String>emptyList();

    LFCommand flash = commandFactory.createCommand(cmd, args);

    if (flash != null) {
      int ret = flash.run(context.getCancelIndicator());
      if (ret != 0) {
        messageReporter
            .nowhere()
            .error(
                "Command " + cmd + " failed with exit code: " + ret + ". Is the target connected?");
      }
    }
  }
}
