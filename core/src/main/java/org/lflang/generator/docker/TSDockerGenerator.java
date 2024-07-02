package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 * @author Marten Lohstroh
 */
public class TSDockerGenerator extends DockerGenerator {

  /** Construct a new Docker generator. */
  public TSDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  @Override
  protected String generateCopyOfExecutable() {
    var lfModuleName = context.getFileConfig().name;
    return "COPY --from=builder /lingua-franca/%s .".formatted(lfModuleName);
  }

  @Override
  protected String generateRunForMakingExecutableDir() {
    return "RUN mkdir dist";
  }

  @Override
  protected String generateCopyForSources() {
    return "COPY . .";
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("node", "dist/%s.js".formatted(context.getFileConfig().name));
  }

  @Override
  protected String generateRunForInstallingDeps() {
    return "RUN apk add git && npm install -g pnpm";
  }

  @Override
  protected List<String> defaultBuildCommands() {
    return List.of("pnpm install", "pnpm run build");
  }

  @Override
  public String defaultImage() {
    return "node:alpine";
  }
}
