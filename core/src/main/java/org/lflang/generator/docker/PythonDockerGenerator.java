package org.lflang.generator.docker;

import org.lflang.generator.LFGeneratorContext;

/**
 * Generates the docker file related code for the Python target.
 *
 * @author Hou Seng Wong
 */
public class PythonDockerGenerator extends CDockerGenerator {
  public static final String DEFAULT_BASE_IMAGE = "python:3.10-slim";

  public PythonDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  @Override
  public String defaultImage() {
    return DEFAULT_BASE_IMAGE;
  }

  @Override
  protected String generateRunForBuildDependencies() {
    if (baseImage().equals(defaultImage())) {
      return """
             # Install build dependencies
             RUN set -ex && apt-get update && apt-get install -y python3-pip && pip install cmake
             """;
    } else {
      return """
             # Check for build dependencies
             RUN which make && which cmake && which gcc
             """;
    }
  }

  /** Generates the contents of the docker file. */
  @Override
  protected String generateDockerFileContent() {
    return String.join(
        "\n",
        "# For instructions, see:"
            + " https://www.lf-lang.org/docs/handbook/containerized-execution?target=py",
        "FROM " + baseImage(),
        "WORKDIR /lingua-franca/" + context.getFileConfig().name,
        generateRunForBuildDependencies(),
        "COPY . src-gen",
        super.generateCompileCommand(),
        "ENTRYPOINT [\"python3\", \"-u\", \"src-gen/" + context.getFileConfig().name + ".py\"]");
  }
}
