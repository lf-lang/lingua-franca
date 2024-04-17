package org.lflang.generator.docker;

import java.nio.file.Path;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;

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
  protected abstract String generateRunForBuildDependencies();

  /** Return the default base image. */
  public abstract String defaultImage();

  /** Return the selected base image, or the default one if none was selected. */
  public String baseImage() {
    var baseImage = context.getTargetConfig().get(DockerProperty.INSTANCE).from();
    if (baseImage != null && !baseImage.isEmpty()) {
      return baseImage;
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
      case CPP, Rust -> throw new IllegalArgumentException(
          "No Docker support for " + target + " yet.");
    };
  }
}
