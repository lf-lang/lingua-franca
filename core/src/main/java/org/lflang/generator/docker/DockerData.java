package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
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
    var pbs = context.getTargetConfig().get(DockerProperty.INSTANCE).preBuildScript();
    if (!pbs.isEmpty()) {
      var found = FileUtil.findInPackage(Path.of(pbs), context.getFileConfig()); // FIXME: the FileConfig points to fed-gen and the file hasn't been copied there
      if (found != null) {
        System.out.println(">>>>>>>>>>>>>>>>>>>!!!!!!!!!: " + found.toString());
        System.out.println(dockerFilePath.getParent().resolve(found.getFileName()));
        FileUtil.copyFile(found, dockerFilePath.getParent().resolve(found.getFileName()));
      }
    }

    Files.deleteIfExists(dockerFilePath);
    FileUtil.writeToFile(dockerFileContent, dockerFilePath);
    context.getErrorReporter().nowhere().info("Dockerfile written to " + dockerFilePath);
  }
}
