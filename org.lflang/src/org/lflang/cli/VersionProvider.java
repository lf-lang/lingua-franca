package org.lflang.cli;

import org.lflang.LocalStrings;
import picocli.CommandLine.IVersionProvider;
import picocli.CommandLine.Model.CommandSpec;
import picocli.CommandLine.Spec;

/*
 * Dynamically provides version information to the Lingua Franca CLI.
 * Picocli will instantiate this class and invoke it to collect version
 * information.
 *
 * @author Atharva Patil
 */
class VersionProvider implements IVersionProvider {
  /*
   * Here, picocli will inject the CommandSpec (full command hierarchy) of the
   * command that uses this version provider. This allows this version
   * provider to be reused among multiple commands.
   */
  @Spec CommandSpec spec;

  // Method invoked by picocli to get the version info.
  public String[] getVersion() {
    return new String[] {
      // "lfc", "lff", etc.
      spec.qualifiedName() + " " + LocalStrings.VERSION
    };
  }
}
