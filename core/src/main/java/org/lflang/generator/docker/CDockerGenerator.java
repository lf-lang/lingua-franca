package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CCompiler;
import org.lflang.generator.c.CFileConfig;
import org.lflang.target.Target;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author Hou Seng Wong
 * @author Marten Lohstroh
 */
public class CDockerGenerator extends DockerGenerator {

  /**
   * The constructor for the base docker file generation class.
   *
   * @param context The context of the code generator.
   */
  public CDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  public static final String DEFAULT_BASE_IMAGE = "alpine:latest";

  @Override
  public String defaultImage() {
    return DEFAULT_BASE_IMAGE;
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("./bin/" + context.getFileConfig().name);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    var config = context.getTargetConfig();
    var compiler = config.target == Target.CCPP ? "g++" : "gcc";
    if (builderBase().equals(defaultImage())) {
      return "RUN set -ex && apk add --no-cache %s musl-dev cmake make linux-headers".formatted(compiler);
    } else {
      return "# (Skipping installation of build dependencies; custom base image.)";
    }
  }

  @Override
  protected List<String> defaultBuildCommands() {
    try {
      var ccompile =
          new CCompiler(
              context.getTargetConfig(),
              new CFileConfig(
                  context.getFileConfig().resource,
                  Path.of("/lingua-franca", context.getFileConfig().name),
                  false),
              context.getErrorReporter(),
              context.getTargetConfig().target == Target.CCPP);
      return List.of(
          "mkdir -p bin",
          String.format(
              "%s -DCMAKE_INSTALL_BINDIR=./bin -S src-gen -B bin", ccompile.compileCmakeCommand()),
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
