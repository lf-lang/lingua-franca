package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;

import org.lflang.util.FileUtil;

/**
 * Build configuration of a docker service.
 *
 * @author Marten Lohstroh
 */
public class DockerData {
    /**
     * The absolute path to the docker file.
     */
    private final Path dockerFilePath;

    /**
     * The content of the docker file to be generated.
     */
    private final String dockerFileContent;

    /**
     * The name of the service.
     */
    public final String serviceName;

    public DockerData(
        String serviceName,
        Path dockerFilePath,
        String dockerFileContent
    ) {

        if (!dockerFilePath.toFile().isAbsolute()) {
            throw new RuntimeException("Cannot use relative docker file path in DockerData instance");
        }
        this.serviceName = serviceName;
        this.dockerFilePath = dockerFilePath;
        this.dockerFileContent = dockerFileContent;
    }

    /**
     * Write a docker file based on this data.
     */
    public void writeDockerFile() throws IOException {
        if (dockerFilePath.toFile().exists()) {
            dockerFilePath.toFile().delete();
        }
        FileUtil.writeToFile(dockerFileContent, dockerFilePath);
        System.out.println("Dockerfile written to " + dockerFilePath);
    }
}
