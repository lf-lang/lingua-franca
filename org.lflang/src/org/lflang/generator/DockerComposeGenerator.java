package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.util.FileUtil;

/**
 * Code generator for docker-compose configurations.
 *
 * @author Marten Lohstroh
 * @author Steven Wong
 */
public class DockerComposeGenerator {

    /**
     * Path to the docker-compose.yml file.
     */
    protected final Path path;

    public DockerComposeGenerator(LFGeneratorContext context) {
        this.path = context.getFileConfig().getSrcGenPath().resolve("docker-compose.yml");
    }

    /**
     * Return a string that represents the network portion of the docker-compose configuration.
     * @param networkName Name of the default network
     */
    protected String generateDockerNetwork(String networkName) {
        return """
            networks:
                default:
                    name: "%s"
            """.formatted(networkName);
    }

    /**
     * Return a string that represents the services portion of the docker-compose configuration.
     * @param services A list of docker data representing the services to render
     */
    protected String generateDockerServices(List<DockerData> services) {
        return """
            version: "3.9" 
            services:
            %s
            """.formatted(services.stream().map(
                data -> getServiceDescription(data)
            ).collect(Collectors.joining("\n")));
    }

    /**
     * Return the command to build and run using the docker-compose configuration.
     */
    public String getUsageInstructions() {
        return """
            #####################################
            To build and run:
                pushd %s && docker compose up --build
            To return to the current working directory afterwards:
                popd
            #####################################
            """.formatted(path.getParent());
    }

    /**
     * Turn given docker data into a string.
     */
    protected String getServiceDescription(DockerData data) {
        return """
                %s:
                    build:
                        context: "%s"
                    container_name: "%s"
            """.formatted(getServiceName(data), getBuildContext(data), getContainerName(data));
    }

    /**
     * Return the name of the service represented by the given data.
     */
    protected String getServiceName(DockerData data) {
        return "main";
    }

    /**
     * Return the name of the service represented by the given data.
     */
    protected String getBuildContext(DockerData data) {
        return ".";
    }

    /**
     * Return the name of the container for the given data.
     */
    protected String getContainerName(DockerData data) {
        return data.serviceName;
    }

    /**
     * Write the docker-compose.yml file with a default network called "lf".
     * @param services A list of all the services.
     */
    public void writeDockerComposeFile(List<DockerData> services) throws IOException {
        writeDockerComposeFile(services, "lf");
    }

    /**
     * Write the docker-compose.yml file.
     * @param services A list of all the services to include.
     * @param networkName The name of the network to which docker will connect the services.
     */
    public void writeDockerComposeFile(
        List<DockerData> services,
        String networkName
    ) throws IOException {
        var contents = String.join("\n",
            this.generateDockerServices(services),
            this.generateDockerNetwork(networkName));
        FileUtil.writeToFile(contents, path);
        System.out.println(getUsageInstructions());
    }
}
