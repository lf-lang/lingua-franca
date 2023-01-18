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
     * The name of the docker compose service for the LF module.
     */
    private String composeServiceName;
    /**
     * The build context of the docker container.
     */
    private String dockerContext;

    private final String containerName;

    private final String serviceName;

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
    public String getComposeServiceName() { return composeServiceName; }
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
        System.out.println("Dockerfile for "+this.getComposeServiceName()+" written to "+this.getFilePath());
    }

    /**
     * Return a service description for the "services" section of the docker-compose.yml file.
     */
    public String getServiceDescription() {
        var tab = " ".repeat(4);
        StringBuilder svc = new StringBuilder();
        svc.append(tab + serviceName +":\n");
        svc.append(tab + tab + "build:\n");
        svc.append(tab.repeat(3) + "context: " + this.getDockerContext());
        svc.append("\n"+tab+tab+"command: -i 1");
        svc.append("\n"+tab+tab+"container_name: " + containerName);
        return svc.toString();
    }
}
