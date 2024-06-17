package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.cpp.CppPlatformGenerator;

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
    // FIXME
    return List.of();
  }
}
