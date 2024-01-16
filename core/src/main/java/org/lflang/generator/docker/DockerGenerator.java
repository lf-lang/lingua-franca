package org.lflang.generator.docker;

import java.nio.file.Path;
import org.lflang.generator.LFGeneratorContext;

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

  /**
   * Produce a DockerData object, which bundles all information needed to output a Dockerfile.
   *
   * @return docker data created based on the context in this instance
   */
  public DockerData generateDockerData() {
    return generateDockerData(context.getFileConfig().getSrcGenPath());
  }

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
      case CPP, Rust -> throw new IllegalArgumentException(
          "No Docker support for " + target + " yet.");
    };
  }
}
