package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;

/**
 * Generate the docker file code for the Rust target.
 *
 * @author Eggsrael / Adam Bonnet
 * @ingroup Docker
 */
public class RustDockerGenerator extends DockerGenerator {

  public RustDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    return "RUN set -ex & apk add --no-cache musl-dev";
  }

  @Override
  protected List<String> defaultBuildCommands() {
    String executableName = context.getFileConfig().name;
    return List.of(
        "cd src-gen",
        "cargo build --release",
        "mkdir ../bin",
        "mv target/release/" + camelToSnakeCase(executableName) + " ../bin/" + executableName);
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("./bin/" + context.getFileConfig().name);
  }

  @Override
  public String defaultImage() {
    return "rust:alpine";
  }


  /**
   *  Converts name stored in the file config to snake_case
   *  Cargo's default executable output is snake_case so this needed to access the executable.
   *
   *  <p>Regex test/edit: https://regexr.com/8ndjc
   *
   */
  private String camelToSnakeCase(String fileName) {
    return fileName.replaceAll("(?<=[a-z0-9])([A-Z])", "_$1").toLowerCase();
  }
}
