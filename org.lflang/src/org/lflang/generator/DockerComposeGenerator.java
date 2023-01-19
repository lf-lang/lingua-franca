package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.util.FileUtil;

public class DockerComposeGenerator {

    protected final Path dockerComposeFilePath;

    public DockerComposeGenerator(LFGeneratorContext context) {
        this.dockerComposeFilePath = context.getFileConfig().getSrcGenPath().resolve("docker-compose.yml");
    }

    /**
     *
     * @param networkName
     * @return
     */
    protected String generateDockerNetwork(String networkName) {
        return String.join("\n",
            "networks:",
            "    default:",
            "        name: "+networkName
        );
    }

    protected String generateDockerServices(List<DockerData> services) {
        return String.join("\n",
            "version: \"3.9\"",
            "services:",
            services.stream().map(
                data -> toString(data)
            ).collect(Collectors.joining("\n"))
        );
    }

    /**
     * Get the command to build the docker images using the compose file.
     *
     * @return The build command printed to the user as to how to build a docker image
     *         using the generated docker file.
     */
    public String getUsageInstructions() {
        return String.join("\n",
            "#####################################",
            "To build:",
            "    pushd " + dockerComposeFilePath.getParent() + " && docker compose build",
            "Then, to launch:",
            "    docker compose up",
            "To return to the current working directory:",
            "    popd",
            "#####################################"
        );
    }

    /**
     * Turn given docker data into a string.
     */
    protected String toString(DockerData data) {
        return data.getServiceDescription(false);
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
     * @param services A list of all the services.
     * @param networkName The name of the network to which docker will connect the containers.
     */
    public void writeDockerComposeFile(
        List<DockerData> services,
        String networkName
    ) throws IOException {
        var contents = String.join("\n",
            this.generateDockerServices(services),
            this.generateDockerNetwork(networkName));
        FileUtil.writeToFile(contents, dockerComposeFilePath);
        System.out.println(getUsageInstructions());
    }
}
