package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CCompiler;
import org.lflang.generator.c.CFileConfig;
import org.lflang.generator.cpp.CppGenerator;
import org.lflang.generator.cpp.CppPlatformGenerator;
import org.lflang.target.Target;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Marten Lohstroh
 */
public class CppDockerGenerator extends CDockerGenerator {

  private CppPlatformGenerator platformGenerator;

  /** Construct a new Docker generator. */
  public CppDockerGenerator(LFGeneratorContext context, CppPlatformGenerator platformGenerator) {
    super(context);
    this.platformGenerator = platformGenerator;
  }

  @Override
  protected List<String> defaultBuildCommands() {
    try {
      return List.of(
          "mkdir -p bin",
          String.format(
              "%s -DCMAKE_INSTALL_BINDIR=./bin -S src-gen -B bin", platformGenerator),
          "cd bin",
          "make all",
          "cd ..");
    } catch (IOException e) {
      context
          .getErrorReporter()
          .nowhere()
          .error("Unable to create file configuration for Docker container");
      throw new RuntimeException(e);
    }
  }

}
