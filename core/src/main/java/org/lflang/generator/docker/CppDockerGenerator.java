package org.lflang.generator.docker;

import java.util.List;
import java.util.stream.Stream;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.cpp.CppPlatformGenerator;

/**
 * Generates the docker file related code for the C++ target.
 *
 * @author Marten Lohstroh
 */
public class CppDockerGenerator extends DockerGenerator {

  private CppPlatformGenerator platformGenerator;

  /** Construct a new Docker generator. */
  public CppDockerGenerator(LFGeneratorContext context, CppPlatformGenerator platformGenerator) {
    super(context);
    this.platformGenerator = platformGenerator;
  }

  @Override
  protected String generateCopyForSources() {
    return "COPY src-gen src-gen";
  }

  public static final String DEFAULT_BASE_IMAGE = "alpine:latest";

  @Override
  public String defaultImage() {
    return DEFAULT_BASE_IMAGE;
  }

  @Override
  protected String generateRunForInstallingDeps() {
    if (builderBase().equals(defaultImage())) {
      return "RUN set -ex && apk add --no-cache g++ musl-dev cmake make && apk add --no-cache"
          + " --update --repository=https://dl-cdn.alpinelinux.org/alpine/v3.16/main/"
          + " libexecinfo-dev";
    } else {
      return "# (Skipping installation of build dependencies; custom base image.)";
    }
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("./bin/" + context.getFileConfig().name);
  }

  @Override
  protected String generateCopyOfExecutable() {
    return String.join(
        "\n",
        super.generateCopyOfExecutable(),
        "COPY --from=builder /usr/local/lib /usr/local/lib",
        "COPY --from=builder /usr/lib /usr/lib",
        "COPY --from=builder /lingua-franca .");
  }

  @Override
  protected List<String> defaultBuildCommands() {
    var mkdirCommand = List.of("mkdir", "-p", "build");
    return Stream.concat(
            Stream.of(mkdirCommand),
            platformGenerator
                .getBuildCommands(List.of("-DREACTOR_CPP_LINK_EXECINFO=ON"), false)
                .stream())
        .map(DockerGenerator::argListToCommand)
        .toList();
  }
}
