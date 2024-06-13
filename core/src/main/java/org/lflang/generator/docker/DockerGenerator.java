package org.lflang.generator.docker;

import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.DockerProperty.DockerOptions;
import org.lflang.util.StringUtil;

/**
 * A class for generating docker files.
 *
 * @author Marten Lohstroh
 * @author Hou Seng Wong
 */
public abstract class DockerGenerator {

  /** Configuration for interactions with the filesystem. */
  protected final LFGeneratorContext context;

  /**
   * The constructor for the base docker file generation class.
   *
   * @param context The context of the code generator.
   */
  public DockerGenerator(LFGeneratorContext context) {
    this.context = context;
  }

  /** Generate the contents of a Dockerfile. */
  protected abstract String generateDockerFileContent();

  /** Return a RUN command for installing/checking build dependencies. */
  protected abstract String generateRunForInstallingDeps();

  /** Return the default compile commands for the C docker container. */
  protected abstract List<String> defaultBuildCommands();

  /** Return the commands used to build */
  protected List<String> getBuildCommands() {
    var customBuildCommands = context.getTargetConfig().get(BuildCommandsProperty.INSTANCE);
    if (customBuildCommands != null && !customBuildCommands.isEmpty()) {
      return customBuildCommands;
    }
    return defaultBuildCommands();
  }

  /** Return the command that sources the pre-build script, if there is one. */
  protected List<String> getPreBuildCommand() {
    var script = context.getTargetConfig().get(DockerProperty.INSTANCE).preBuildScript();
    if (!script.isEmpty()) {
      return List.of("source src-gen/" + script);
    }
    return List.of();
  }

  /** Return the command that sources the post-build script, if there is one. */
  protected List<String> getPostBuildCommand() {
    var script = context.getTargetConfig().get(DockerProperty.INSTANCE).postBuildScript();
    if (!script.isEmpty()) {
      return List.of("source src-gen/" + script);
    }
    return List.of();
  }

  /** Return the Docker RUN command used for building. */
  protected String generateRunForBuild() {
    return "RUN "
        + StringUtil.joinObjects(
            Stream.of(
                    List.of("set -ex"),
                    getPreBuildCommand(),
                    getBuildCommands(),
                    getPostBuildCommand())
                .flatMap(java.util.Collection::stream)
                .collect(Collectors.toList()),
            " \\\n\t&& ");
  }

  protected String generateEntryPoint() {
    return "ENTRYPOINT ["
        + getEntryPointCommands().stream()
            .map(cmd -> "\"" + cmd + "\"")
            .collect(Collectors.joining(","))
        + "]";
  }

  protected String generateCopyOfExecutable() {
    var lfModuleName = context.getFileConfig().name;
    return "COPY --from=builder /lingua-franca/%s/bin/%s ./bin/%s"
        .formatted(lfModuleName, lfModuleName, lfModuleName);
  }

  protected String generateCopyOfScript() {
    var script = context.getTargetConfig().get(DockerProperty.INSTANCE).preRunScript();
    if (!script.isEmpty()) {
      return "COPY --from=builder /lingua-franca/%s/src-gen/%s ./scripts/%s"
          .formatted(context.getFileConfig().name, script, script);
    }
    return "# (No pre-run script provided.)";
  }

  protected List<String> getEntryPointCommands() {
    var script = context.getTargetConfig().get(DockerProperty.INSTANCE).preRunScript();
    if (!script.isEmpty()) {
      return Stream.concat(
              List.of(DockerOptions.DEFAULT_SHELL, "-c", "source scripts/" + script).stream(),
              defaultEntryPoint().stream())
          .toList();
    }
    return defaultEntryPoint();
  }

  public abstract List<String> defaultEntryPoint();

  /** Return the default base image. */
  public abstract String defaultImage();

  protected String builderBase() {
    return baseImage(context.getTargetConfig().get(DockerProperty.INSTANCE).builderBase());
  }

  protected String runnerBase() {
    return baseImage(context.getTargetConfig().get(DockerProperty.INSTANCE).runnerBase());
  }

  /** Return the selected base image, or the default one if none was selected. */
  private String baseImage(String name) {
    if (name != null && !name.isEmpty()) {
      return name;
    }
    return defaultImage();
  }

  /**
   * Produce a DockerData object, which bundles all information needed to output a Dockerfile.
   *
   * @return docker data created based on the context in this instance
   */
  public DockerData generateDockerData() {
    return generateDockerData(context.getFileConfig().getSrcGenPath());
  }

  /**
   * Return a new {@code DockerData} object that can be used to generate a Dockerfile in the
   * directory indicated by the given path.
   *
   * @param path The directory in which to place the generated Dockerfile.
   */
  public DockerData generateDockerData(Path path) {
    var name = context.getFileConfig().name;
    var dockerFileContent = generateDockerFileContent();
    return new DockerData(name, path.resolve("Dockerfile"), dockerFileContent, context);
  }

  public static DockerGenerator dockerGeneratorFactory(LFGeneratorContext context) {
    var target = context.getTargetConfig().target;
    return switch (target) {
      case C, CCPP -> new CDockerGenerator(context);
      case TS -> new TSDockerGenerator(context);
      case Python -> new PythonDockerGenerator(context);
      case CPP, Rust ->
          throw new IllegalArgumentException("No Docker support for " + target + " yet.");
    };
  }
}
