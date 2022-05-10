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
 * - This ("DockerGeneratorBase") class defines a "DockerData" Enum
 *   that specifies the information it needs to generate docker files and
 *   docker compose files for any target. This is the docker-facing
 *   API.
 *
 * Target Code Generator-facing API
 * - Each target-specific docker generator implements this class and
 *   defines in themselves an Enum that implements the "GeneratorData"
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
        /**
         * The name of the docker container.
         */
        private String containerName;

        public DockerData(
            Path dockerFilePath,
            String dockerFileContent,
            String dockerComposeServiceName,
            String dockerBuildContext,
            String dockerContainerName
        ) {
            filePath = dockerFilePath;
            fileContent = dockerFileContent;
            composeServiceName = dockerComposeServiceName;
            buildContext = dockerBuildContext;
            containerName = dockerContainerName;
        }

        public Path getFilePath() { return filePath; }
        public String getFileContent() { return fileContent; }
        public String getComposeServiceName() { return composeServiceName; }
        public String getBuildContext() { return buildContext; }
        public String getContainerName() { return containerName; }
    }

    /**
     * The interface for data from the code generator.
     *
     * Target-specific docker generators can have an Enum
     * field that implements this interface to specify
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
     * specified in the DockerData Enum.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData Enum
     */
    abstract protected DockerData generateDockerData(GeneratorData generatorData);

    /**
     * Add a federate to the list of federates to generate docker files for.
     *
     * @param generatorData Data from the code generator.
     */
    public void addFederate(GeneratorData generatorData) {
        DockerData dockerData = generateDockerData(generatorData);
        validateNotNull(dockerData);
        dockerDataList.add(dockerData);
        appendFederateToDockerComposeServices(dockerData);
    }

    /**
     * Write the docker files generated for the federates added using `addFederate` so far.
     *
     * @param dockerComposeFilePath The path where the docker compose file will be written.
     */
    public void writeDockerFiles(Path dockerComposeFilePath) throws IOException {
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
     * @param dockerData The docker data as specified in the DockerData Enum.
     */
    private void writeDockerFile(DockerData dockerData) throws IOException {
        var dockerFilePath = dockerData.getFilePath();
        if (dockerFilePath.toFile().exists()) {
            dockerFilePath.toFile().delete();
        }
        FileUtil.writeToFile(dockerData.getFileContent(), dockerFilePath);
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     */
    private String getDockerComposeCommand() {
        String OS = System.getProperty("os.name").toLowerCase();
        return (OS.indexOf("nux") >= 0) ? "docker-compose" : "docker compose";
    }

    /**
     * Throws a Runtime Exception if any field in "dockerData" is null.
     */
    private void validateNotNull(DockerData dockerData) {
        if (dockerData.getFilePath() == null || dockerData.getFileContent() == null ||
            dockerData.getComposeServiceName() == null ||
            dockerData.getBuildContext() == null ||
            dockerData.getContainerName() == null) {
                throw new RuntimeException("Fields in DockerData cannot be null");
            }
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param dockerComposeFilePath The directory where the docker compose file is generated.
     * @param dockerData The docker data as specified in the DockerData Enum.
     * @return The build command printed to the user as to how to build a docker image
     *         using the generated docker file.
     */
    private String getDockerBuildCommandMsg(
        Path dockerComposeFilePath,
        DockerData dockerData
    ) {
        return String.join("\n",
            "Dockerfile for "+dockerData.getContainerName()+" written to "+dockerData.getFilePath(),
            "#####################################",
            "To build the docker image, go to "+dockerComposeFilePath.getParent()+" and run:",
            "",
            "    "+getDockerComposeCommand()+" build "+dockerData.getComposeServiceName(),
            "",
            "#####################################"
        );
    }

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
     * @param dockerData The docker data as specified in the DockerData Enum.
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
    public void setHost(String host) {
        this.host = host;
    }
}
