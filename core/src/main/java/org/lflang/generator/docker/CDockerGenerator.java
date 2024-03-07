package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.Target;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author Hou Seng Wong
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

  /** Generate the contents of the docker file. */
  @Override
  protected String generateDockerFileContent() {
    var lfModuleName = context.getFileConfig().name;
    var config = context.getTargetConfig();
    var compileCommand = generateCompileCommand();
    var compiler = config.target == Target.CCPP ? "g++" : "gcc";
    var baseImage = baseImage();
    return String.join(
        "\n",
        "# For instructions, see: https://www.lf-lang.org/docs/handbook/containerized-execution",
        "FROM " + baseImage + " AS builder",
        "WORKDIR /lingua-franca/" + lfModuleName,
        generateRunForBuildDependencies(),
        "COPY . src-gen",
        compileCommand,
        "",
        "FROM " + baseImage,
        "WORKDIR /lingua-franca",
        "RUN mkdir bin",
        "COPY --from=builder /lingua-franca/"
            + lfModuleName
            + "/bin/"
            + lfModuleName
            + " ./bin/"
            + lfModuleName,
        "",
        "# Use ENTRYPOINT not CMD so that command-line arguments go through",
        "ENTRYPOINT [\"./bin/" + lfModuleName + "\"]",
        "");
  }

  @Override
  protected String generateRunForBuildDependencies() {
    var config = context.getTargetConfig();
    var compiler = config.target == Target.CCPP ? "g++" : "gcc";
    if (baseImage().equals(defaultImage())) {
      return """
          # Install build dependencies
          RUN set -ex && apk add --no-cache %s musl-dev cmake make
          """
          .formatted(compiler);
    } else {
      return """
          # Check for build dependencies
          RUN which make && which cmake && which %s
          """
          .formatted(compiler);
    }
  }

  @Override
  protected List<String> defaultBuildCommands() {
    return List.of(
        "set -ex",
        "mkdir -p bin",
        "cmake -DCMAKE_INSTALL_BINDIR=./bin -S src-gen -B bin",
        "cd bin",
        "make all");
  }
}
