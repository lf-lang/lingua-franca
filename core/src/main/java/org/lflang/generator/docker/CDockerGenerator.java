package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CCompiler;
import org.lflang.generator.c.CFileConfig;
import org.lflang.target.Target;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author Hou Seng Wong
 * @author Marten Lohstroh
 * @ingroup Docker
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

  @Override
  protected String builderBase() {
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST) {
      return "sst-builder";
    }

    return super.builderBase();
  }

  @Override
  protected String generateAdditionalArguments() {
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST) {
      return String.join(
          "\n",
          "FROM " + defaultImage() + " AS sst-builder",
          "RUN set -ex && apk add --no-cache gcc musl-dev cmake make openssl-dev",
          "COPY /sst-src/ /sst-src/",
          "WORKDIR /sst-build",
          "RUN cmake -DBUILD_TESTING=OFF /sst-src && make && make install");
    }
    return "";
  }

  @Override
  protected String generateCopyOfCredentials() {
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST) {
      return "COPY /sst/ ./sst/";
    }

    return "";
  }

  @Override
  public List<String> defaultEntryPoint() {
    var name = context.getFileConfig().name;
    var isSST =
        context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    if (isSST) {
      return List.of("./bin/" + name, "-sst", "./sst/" + name + ".config");
    }
    return List.of("./bin/" + context.getFileConfig().name);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    var config = context.getTargetConfig();
    var compiler = config.target == Target.CCPP ? "g++" : "gcc";
    var isSST =
        context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;

    if (isSST) {
      return "# (Dependencies already installed in sst-builder stage)";
    }

    if (builderBase().equals(defaultImage())) {
      return "RUN set -ex && apk add --no-cache %s musl-dev cmake make".formatted(compiler);
    } else {
      return "# (Skipping installation of build dependencies; custom base image.)";
    }
  }

  @Override
  protected List<String> defaultBuildCommands() {
    try {
      var ccompile =
          new CCompiler(
              context.getTargetConfig(),
              new CFileConfig(
                  context.getFileConfig().resource,
                  Path.of("/lingua-franca", context.getFileConfig().name),
                  false),
              context.getErrorReporter(),
              context.getTargetConfig().target == Target.CCPP);
      var isSST =
          context.getTargetConfig().get(CommunicationModeProperty.INSTANCE)
              == CommunicationMode.SST;
      var commFlag = isSST ? " -DCOMM_TYPE=SST" : "";
      var srcDir = "src-gen";
      return List.of(
          "mkdir -p bin",
          String.format(
              "%s -DCMAKE_INSTALL_BINDIR=./bin%s -S %s -B bin",
              ccompile.compileCmakeCommand(), commFlag, srcDir),
          "cd bin",
          "make all",
          "cd ..");
    } catch (IOException e) {
      context
          .getErrorReporter()
          .nowhere()
          .error("Unable to create file configuration for Docker container");
      throw new RuntimeException(e);
    }
  }
}
