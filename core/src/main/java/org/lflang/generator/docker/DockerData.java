package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.lflang.FileConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.util.FileUtil;

/**
 * Build configuration of a docker service.
 *
 * @author Marten Lohstroh
 */
public class DockerData {
  /** The absolute path to the docker file. */
  private final Path dockerFilePath;

  /** The content of the docker file to be generated. */
  private final String dockerFileContent;

  /** The name of the service. */
  public final String serviceName;

  private final LFGeneratorContext context;

  public DockerData(
      String serviceName,
      Path dockerFilePath,
      String dockerFileContent,
      LFGeneratorContext context) {
    this.context = context;

    if (!dockerFilePath.toFile().isAbsolute()) {
      throw new RuntimeException("Cannot use relative docker file path in DockerData instance");
    }
    this.serviceName = serviceName;
    this.dockerFilePath = dockerFilePath;
    this.dockerFileContent = dockerFileContent;
  }

  /** Write a docker file based on this data. */
  public void writeDockerFile() throws IOException {
    Files.deleteIfExists(dockerFilePath);
    FileUtil.writeToFile(dockerFileContent, dockerFilePath);
    context.getErrorReporter().nowhere().info("Dockerfile written to " + dockerFilePath);
  }

  /** Copy the pre-build, post-build, and pre-run scripts, if specified */
  public void copyScripts(LFGeneratorContext context) throws IOException {
    var prop = context.getTargetConfig().get(DockerProperty.INSTANCE);
    copyScripts(
        context.getFileConfig(),
        List.of(prop.preBuildScript(), prop.postBuildScript(), prop.preRunScript()));
  }

  /** Copy the given list of scripts */
  private void copyScripts(FileConfig fileConfig, List<String> scripts) throws IOException {
    for (var script : scripts) {
      if (!script.isEmpty()) {
        var found = FileUtil.findInPackage(Path.of(script), fileConfig);
        if (found != null) {
          var destination = dockerFilePath.getParent().resolve(found.getFileName());
          FileUtil.copyFile(found, destination);
          this.context.getErrorReporter().nowhere().info("Script written to " + destination);
        }
      }
    }
  }
}
