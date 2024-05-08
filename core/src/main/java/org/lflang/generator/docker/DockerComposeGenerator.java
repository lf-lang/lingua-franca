package org.lflang.generator.docker;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.lflang.generator.LFGeneratorContext;
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
            version: "3.9"
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
            """
        .formatted(getServiceName(data), getBuildContext(data), getContainerName(data));
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
    var contents =
        String.join(
            "\n", this.generateDockerServices(services), this.generateDockerNetwork(networkName));
    FileUtil.writeToFile(
        contents, context.getFileConfig().getSrcGenPath().resolve("docker-compose.yml"));
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
    var script =
        """
        #!/bin/bash
        set -euo pipefail
        cd $(dirname "$0")
        cd ..
        cd "%s"
        docker compose up
        """
            .formatted(packageRoot.relativize(srcGenPath));
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
}
