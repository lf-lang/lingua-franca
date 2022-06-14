package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.util.FileUtil;

/**
 * The base class for docker file related code generation.
 *
 * The design of abstractions is as follows:
 *
 * Docker-facing API
 * - This ("DockerGeneratorBase") class defines a "DockerData" class
 *   that specifies the information it needs to generate docker files and
 *   docker compose files for any target. This is the docker-facing
 *   API.
 *
 * Target Code Generator-facing API
 * - Each target-specific docker generator implements this class and
 *   defines in themselves a class that implements the "GeneratorData"
 *   interface of this class. This is the target code generator-facing API.
 *
 * The purpose of this abstraction design is to contain all
 * docker-specific information inside docker generator classes and
 * prevent docker-related information from polluting target code generators.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
abstract public class DockerGeneratorBase {
    /**
     * The docker compose services representing each federate.
     * Ideally, this would be a list of Strings instead of a StringBuilder.
     */
    protected StringBuilder composeServices;

    /**
     * A docker file will be generated for each lingua franca module.
     * This maps the name of the LF module to the data related to the docker
     * file for that module.
     */
    protected List<DockerData> dockerDataList;

    /**
     * Indicates whether or not the program is federated.
     */
    protected final boolean isFederated;

    /**
     * In federated execution, the host of the rti.
     */
    protected String host = null;

    /**
     * Generates the docker file related code for the Python target.
     * The type specified in the following javadoc refers to the
     * type of the object stored in `moduleNameToData.get(lfModuleName)`
     */
    protected class DockerData {
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
        private String buildContext;

        public DockerData(
            Path dockerFilePath,
            String dockerFileContent,
            String dockerBuildContext
        ) {
            if (dockerFilePath == null || dockerFileContent == null ||
                    dockerBuildContext == null) {
                throw new RuntimeException("Missing fields in DockerData instance");
            }
            if (!dockerFilePath.toFile().isAbsolute()) {
                throw new RuntimeException("Non-absolute docker file path in DockerData instance");
            }
            if (!dockerFilePath.toString().endsWith(".Dockerfile")) {
                throw new RuntimeException(
                    "Docker file path does not end with \".Dockerfile\" in DockerData instance");
            }
            filePath = dockerFilePath;
            fileContent = dockerFileContent;
            composeServiceName = filePath.getFileName().toString().replace(".Dockerfile", "").toLowerCase();
            buildContext = dockerBuildContext;
        }

        public Path getFilePath() { return filePath; }
        public String getFileContent() { return fileContent; }
        public String getComposeServiceName() { return composeServiceName; }
        public String getBuildContext() { return buildContext; }
    }

    /**
     * The interface for data from the code generator.
     *
     * Target-specific docker generators can have a class
     * that implements this interface to specify
     * what kinds of generator-related data is needed
     * during docker file generation.
     */
    public interface GeneratorData {}

    /**
     * The constructor for the base docker file generation class.
     * @param isFederated True if federated execution. False otherwise.
     */
    public DockerGeneratorBase(boolean isFederated) {
        dockerDataList = new ArrayList<>();
        composeServices = new StringBuilder();
        this.isFederated = isFederated;
    }

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData class.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData class
     */
    abstract protected DockerData generateDockerData(GeneratorData generatorData);

    /**
     * Add a file to the list of docker files to generate.
     *
     * @param generatorData Data from the code generator.
     */
    public void addFile(GeneratorData generatorData) {
        DockerData dockerData = generateDockerData(generatorData);
        dockerDataList.add(dockerData);
        appendFederateToDockerComposeServices(dockerData);
    }

    /**
     * Write the docker files generated for the federates added using `addFederate` so far.
     *
     * @param dockerComposeFilePath The path where the docker compose file will be written.
     */
    public void writeDockerFiles(Path dockerComposeFilePath) throws IOException {
        if (!dockerComposeFilePath.getFileName().toString().equals("docker-compose.yml")) {
            throw new RuntimeException(
                "Docker compose file must have the name \"docker-compose.yml\"");
        }
        for (DockerData dockerData : dockerDataList) {
            writeDockerFile(dockerData);
            System.out.println(
                getDockerBuildCommandMsg(
                    dockerComposeFilePath, dockerData));
        }

        System.out.println(getDockerComposeUpMsg(dockerComposeFilePath));
        if (isFederated && host != null) {
            appendRtiToDockerComposeServices(
                "lflang/rti:rti",
                host
            );
        }
        writeFederatesDockerComposeFile(dockerComposeFilePath, "lf");
    }

    /**
     * Writes the docker file given the docker data.
     *
     * @param dockerData The docker data as specified in the DockerData class.
     */
    private void writeDockerFile(DockerData dockerData) throws IOException {
        var dockerFilePath = dockerData.getFilePath();
        if (dockerFilePath.toFile().exists()) {
            dockerFilePath.toFile().delete();
        }
        FileUtil.writeToFile(dockerData.getFileContent(), dockerFilePath);
    }

    /**
     * Get the command for docker compose depending on the OS.
     */
    public static String getDockerComposeCommand() {
        String OS = System.getProperty("os.name").toLowerCase();
        return (OS.indexOf("nux") >= 0) ? "docker-compose" : "docker compose";
    }

    /**
     * Get the command to build the docker images using the compose file.
     * @param dockerComposeFilePath The directory where the docker compose file is generated.
     * @param dockerData The docker data as specified in the DockerData class.
     * @return The build command printed to the user as to how to build a docker image
     *         using the generated docker file.
     */
    private String getDockerBuildCommandMsg(
        Path dockerComposeFilePath,
        DockerData dockerData
    ) {
        return String.join("\n",
            "Dockerfile for "+dockerData.getComposeServiceName()+" written to "+dockerData.getFilePath(),
            "#####################################",
            "To build the docker image, go to "+dockerComposeFilePath.getParent()+" and run:",
            "",
            "    "+getDockerComposeCommand()+" build "+dockerData.getComposeServiceName(),
            "",
            "#####################################"
        );
    }

    /**
     * Get the command to launch all containers using the compose file
     * @param dockerComposeFilePath The directory where the docker compose file is generated.
     */
    private String getDockerComposeUpMsg(Path dockerComposeFilePath) {
        return String.join("\n",
            "#####################################",
            "To launch the docker container(s), go to "+dockerComposeFilePath.getParent()+" and run:",
            "",
            "    "+getDockerComposeCommand()+" up",
            "",
            "#####################################"
        );
    }

    /**
     * Write the docker-compose.yml for orchestrating the federates.
     * @param dockerComposeFilePath The directory where the docker compose file is generated.
     * @param networkName The name of the network to which docker will connect the containers.
     */
    private void writeFederatesDockerComposeFile(
        Path dockerComposeFilePath,
        String networkName
    ) throws IOException {
        var contents = new CodeBuilder();
        contents.pr(String.join("\n",
            "version: \"3.9\"",
            "services:",
            composeServices.toString(),
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        ));
        FileUtil.writeToFile(contents.toString(), dockerComposeFilePath);
    }

    /**
     * Append a service to the "services" section of the docker-compose.yml file.
     * @param dockerData The docker data as specified in the DockerData class.
     */
    private void appendFederateToDockerComposeServices(
        DockerData dockerData
    ) {
        var tab = " ".repeat(4);
        composeServices.append(tab+dockerData.getComposeServiceName()+":\n");
        composeServices.append(tab+tab+"build:\n");
        composeServices.append(tab+tab+tab+"context: "+dockerData.getBuildContext()+"\n");
        composeServices.append(tab+tab+tab+"dockerfile: "+dockerData.getFilePath()+"\n");
        if (isFederated) {
            composeServices.append(tab+tab+"command: -i 1\n");
        }
    }

    /**
     * Append the RTI to the "services" section of the docker-compose.yml file.
     *
     * @param rtiImageName The name of the docker image of the RTI.
     * @param hostName The name of the host to put the RTI docker container.
     */
    private void appendRtiToDockerComposeServices(String rtiImageName, String hostName) {
        var tab = " ".repeat(4);
        composeServices.append(tab+"rti:\n");
        composeServices.append(tab+tab+"image: "+rtiImageName+"\n");
        composeServices.append(tab+tab+"hostname: "+hostName+"\n");
        composeServices.append(tab+tab+"command: -i 1 -n "+dockerDataList.size()+"\n");
    }

     /**
     * Set the `host` of the container
     * that launches the RTI.
     * @param host The host to set.
     */
    public void setHost(Object host) {
        if (host != null) {
            setHost(host.toString());
        }
    }

    /**
     * Set the `host` of the container
     * that launches the RTI.
     * @param host The host to set.
     */
    public void setHost(String host) {
        this.host = host;
    }
}
