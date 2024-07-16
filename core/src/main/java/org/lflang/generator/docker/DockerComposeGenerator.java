package org.lflang.generator.docker;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.apache.commons.text.StringEscapeUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

/**
 * Code generator for docker-compose configurations.
 *
 * @author Marten Lohstroh
 * @author Steven Wong
 */
public class DockerComposeGenerator {

  /** Context of the code generator. */
  protected final LFGeneratorContext context;

  public DockerComposeGenerator(LFGeneratorContext context) {
    this.context = context;
  }

  /**
   * Return a string that represents the network portion of the docker-compose configuration.
   *
   * @param networkName Name of the default network
   */
  protected String generateDockerNetwork(String networkName) {
    return """
           networks:
               default:
                   name: "%s"
           """
        .formatted(networkName);
  }

  /**
   * Return a string that represents the services portion of the docker-compose configuration.
   *
   * @param services A list of docker data representing the services to render
   */
  protected String generateDockerServices(List<DockerData> services) {
    return """
           services:
           %s
           """
        .formatted(
            services.stream().map(this::getServiceDescription).collect(Collectors.joining("\n")));
  }

  /** Turn given docker data into a string. */
  protected String getServiceDescription(DockerData data) {
    return """
               %s:
                   build:
                       context: "%s"
                   container_name: "%s"
                   tty: true
                   extra_hosts:
                     - "host.docker.internal:host-gateway"
                   environment:
                     - "LF_TELEGRAF_HOST_NAME=${LF_TELEGRAF_HOST_NAME:-host.docker.internal}"
                   %s
           """
        .formatted(
            getServiceName(data),
            getBuildContext(data),
            getContainerName(data),
            getEnvironmentFile());
  }

  private String getEnvironmentFile() {
    var file = context.getTargetConfig().get(DockerProperty.INSTANCE).envFile();
    if (!file.isEmpty()) {
      return "env_file: \"%s\"".formatted(StringEscapeUtils.escapeXSI(file));
    }
    return "";
  }

  /** Return the name of the service represented by the given data. */
  protected String getServiceName(DockerData data) {
    return "main";
  }

  /** Return the name of the service represented by the given data. */
  protected String getBuildContext(DockerData data) {
    return ".";
  }

  /** Return the name of the container for the given data. */
  protected String getContainerName(DockerData data) {
    return data.serviceName;
  }

  /**
   * Write the docker-compose.yml file with a default network called "lf".
   *
   * @param services A list of all the services.
   */
  public void writeDockerComposeFile(List<DockerData> services) throws IOException {
    writeDockerComposeFile(services, "lf");
  }

  /**
   * Write the docker-compose.yml file.
   *
   * @param services A list of all the services to include.
   * @param networkName The name of the network to which docker will connect the services.
   */
  public void writeDockerComposeFile(List<DockerData> services, String networkName)
      throws IOException {
    var dockerComposeDir = context.getFileConfig().getSrcGenPath();
    var contents =
        String.join(
            "\n", this.generateDockerServices(services), this.generateDockerNetwork(networkName));
    FileUtil.writeToFile(contents, dockerComposeDir.resolve("docker-compose.yml"));
    var dockerConfigFile = context.getTargetConfig().get(DockerProperty.INSTANCE).dockerConfigFile();
    if(!dockerConfigFile.isEmpty()){
      var found = FileUtil.findInPackage(Path.of(dockerConfigFile), context.getFileConfig());
      if (found != null) {
        var destination = dockerComposeDir.resolve("docker-compose-override.yml");
        FileUtil.copyFile(found, destination);
        this.context.getErrorReporter().nowhere().info("Docker compose override file copied to " + destination);
      }
    }
    var envFile = context.getTargetConfig().get(DockerProperty.INSTANCE).envFile();
    if (!envFile.isEmpty()) {
      var found = FileUtil.findInPackage(Path.of(envFile), context.getFileConfig());
      if (found != null) {
        var destination = dockerComposeDir.resolve(found.getFileName());
        FileUtil.copyFile(found, destination);
        this.context
            .getErrorReporter()
            .nowhere()
            .info("Environment file written to " + destination);
      }
    }
  }

  /**
   * Build using docker compose.
   *
   * @return {@code true} if successful,{@code false} otherwise.
   */
  public boolean build() {
    return Objects.requireNonNull(
                LFCommand.get(
                    "docker",
                    List.of("compose", "build"),
                    false,
                    context.getFileConfig().getSrcGenPath()))
            .run()
        == 0;
  }

  /** Create a launcher script that invokes Docker. */
  public void createLauncher() {
    var fileConfig = context.getFileConfig();
    var packageRoot = fileConfig.srcPkgPath;
    var srcGenPath = fileConfig.getSrcGenPath();
    var binPath = fileConfig.binPath;
    FileUtil.createDirectoryIfDoesNotExist(binPath.toFile());
    var file = binPath.resolve(fileConfig.name).toFile();

    final var relPath =
        FileUtil.toUnixString(fileConfig.binPath.relativize(fileConfig.getOutPath()));

    var script =
        """
        #!/bin/bash
        set -euo pipefail
        cd $(dirname "$0")
        cd "%s/%s"
        docker compose -f docker-compose.yml %s up --abort-on-container-failure
        """
            .formatted(
                relPath,
                packageRoot.relativize(srcGenPath),
                Files.exists(fileConfig.getSrcGenPath().resolve("docker-compose-override.yml"))
                    ? "-f docker-compose-override.yml"
                    : "");
    var messageReporter = context.getErrorReporter();
    try {
      var writer = new BufferedWriter(new FileWriter(file));
      writer.write(script);
      writer.close();
    } catch (IOException e) {
      messageReporter
          .nowhere()
          .warning("Unable to write launcher to " + file.getAbsolutePath() + " with error: " + e);
    }

    if (!file.setExecutable(true, false)) {
      messageReporter.nowhere().warning("Unable to make launcher script executable.");
    }
  }

  /**
   * Build, unless building was disabled.
   *
   * @return {@code false} if building failed, {@code true} otherwise
   */
  public boolean buildIfRequested() {
    if (!context.getTargetConfig().get(DockerProperty.INSTANCE).noBuild()) {
      if (build()) {
        createLauncher();
      } else context.getErrorReporter().nowhere().error("Docker build failed.");
      return false;
    }
    return true;
  }
}
