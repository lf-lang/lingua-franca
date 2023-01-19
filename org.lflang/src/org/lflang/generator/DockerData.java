package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;

import org.lflang.util.FileUtil;

/**
 *
 */
public class DockerData {
    /**
     * The absolute path to the docker file.
     */
    private Path filePath;
    /**
     * The content of the docker file to be generated.
     */
    private String fileContent;

    /**
     * The build context of the docker container.
     */
    public final String dockerContext;

    public final String containerName;

    public final String serviceName;

    public DockerData(
        String serviceName,
        String containerName,
        Path dockerFilePath,
        String dockerFileContent,
        String dockerContext
    ) {
        this.serviceName = serviceName;
        this.containerName = containerName;

        if (dockerFilePath == null || dockerFileContent == null ||
            dockerContext == null) {
            throw new RuntimeException("Missing fields in DockerData instance");
        }
        if (!dockerFilePath.toFile().isAbsolute()) {
            throw new RuntimeException("Non-absolute docker file path in DockerData instance");
        }
        filePath = dockerFilePath;
        fileContent = dockerFileContent;
        this.dockerContext = dockerContext;
    }

    public Path getFilePath() { return filePath; }
    public String getFileContent() { return fileContent; }
    public String getDockerContext() { return dockerContext; }

    /**
     * Write a docker file based on this data.
     */
    public void writeDockerFile() throws IOException {
        var dockerFilePath = this.getFilePath();
        if (dockerFilePath.toFile().exists()) {
            dockerFilePath.toFile().delete();
        }
        FileUtil.writeToFile(this.getFileContent(), dockerFilePath);
        System.out.println("Dockerfile written to " + this.getFilePath());
    }
}
