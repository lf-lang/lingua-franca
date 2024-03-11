package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 */
public class TSDockerGenerator extends DockerGenerator {

  /** Construct a new Docker generator. */
  public TSDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  /** Return the content of the docker file for [tsFileName]. */
  public String generateDockerFileContent() {
    return """
        |FROM %s
        |WORKDIR /linguafranca/$name
        |%s
        |COPY . .
        |ENTRYPOINT ["node", "dist/%s.js"]
        """
        .formatted(baseImage(), generateRunForInstallingDeps(), context.getFileConfig().name);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    return "RUN which node && node --version";
  }

  @Override
  protected List<String> defaultBuildCommands() {
    return List.of("pnpm install"); // FIXME: actually build using docker, not natively.
  }

  @Override
  public String defaultImage() {
    return "node:alpine";
  }
}
