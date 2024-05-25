package org.lflang.generator.docker;

import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CCompiler;
import org.lflang.target.Target;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.util.StringUtil;

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
    var compileCommand =
        IterableExtensions.isNullOrEmpty(config.get(BuildCommandsProperty.INSTANCE))
            ? generateCompileCommand()
            : StringUtil.joinObjects(config.get(BuildCommandsProperty.INSTANCE), " ");
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
          # Optional user specified run command
          %s
          """
          .formatted(compiler, userRunCommand());
    } else {
      return """
          # Optional user specified run command
          %s
          # Check for build dependencies
          RUN which make && which cmake && which %s
          """
          .formatted(userRunCommand(), compiler);
    }
  }

  /** Return the default compile command for the C docker container. */
  protected String generateCompileCommand() {
    var ccompile =
        new CCompiler(
            context.getTargetConfig(),
            context.getFileConfig(),
            context.getErrorReporter(),
            context.getTargetConfig().target == Target.C);
    return String.join(
        "\n",
        "RUN set -ex && \\",
        "mkdir bin && \\",
        String.format(
            "%s -DCMAKE_INSTALL_BINDIR=./bin -S src-gen -B bin && \\",
            ccompile.compileCmakeCommand()),
        "cd bin && \\",
        "make all");
  }
}
