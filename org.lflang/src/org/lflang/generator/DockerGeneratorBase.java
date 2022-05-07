package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.util.FileUtil;

/**
 * The base class for docker file related code generation.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class DockerGeneratorBase {
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
    protected Map<String, Map<Key, Object>> moduleNameToData;

    /**
     * Indicates whether or not the program is federated.
     */
    protected final boolean isFederated;

    /**
     * The number of federates this docker generator have added using `addFederate` so far.
     */
    protected int nFederates;

    /**
     * In federated execution, the host of the rti.
     */
    protected String host = null;

    /**
     * Generates the docker file related code for the Python target.
     * The type specified in the following javadoc refers to the
     * type of the object stored in `moduleNameToData.get(lfModuleName)`
     */
    protected enum Key {
        /**
         * A `Path` object that is the absolute path to the docker file.
         */
        DOCKER_FILE_PATH,
        /**
         * A `String` object that is the content of the docker file
         * to be generated.
         */
        DOCKER_FILE_CONTENT,
        /**
         * A `String` object that is the name of the docker compose
         * service for the LF module.
         */
        DOCKER_COMPOSE_SERVICE_NAME,
        /**
         * A `String` object that is the build context of the 
         * docker container.
         */
        DOCKER_BUILD_CONTEXT,
    }

    /**
     * The constructor for the base docker file generation class.
     * @param isFederated True if federated execution. False otherwise.
     */
    public DockerGeneratorBase(boolean isFederated) {
        moduleNameToData = new HashMap<>();
        composeServices = new StringBuilder();
        this.isFederated = isFederated;
        nFederates = 0;
    }

    /**
     * Set the `host` of the container
     * that launches the RTI.
     * @param host The host to set.
     */
    public void setHost(String host) {
        this.host = host;
    }

    /**
     * Add a federate to the list of federates to generate docker files for.
     *
     * @param lfModuleName The module name of the federate.
     * @param federateName The name of the federate's reactor.
     * @param dockerFilePath The path where the docker file will be written.
     * @param targetConfig The target config.
     */
    public void addFederate(
        String lfModuleName,
        String federateName,
        Path dockerFilePath,
        TargetConfig targetConfig
    ) {
        Map<Key, Object> k = new HashMap<>();
        k.put(Key.DOCKER_FILE_PATH, dockerFilePath);
        var dockerFileContent = generateDockerFileContent(lfModuleName);
        k.put(Key.DOCKER_FILE_CONTENT, dockerFileContent);
        k.put(Key.DOCKER_COMPOSE_SERVICE_NAME, isFederated ? federateName : lfModuleName.toLowerCase());
        k.put(Key.DOCKER_BUILD_CONTEXT, isFederated ? federateName : ".");
        moduleNameToData.put(lfModuleName, k);
        nFederates++;
        appendFederateToDockerComposeServices(
            composeServices,
            (String) k.get(Key.DOCKER_COMPOSE_SERVICE_NAME),
            (String) k.get(Key.DOCKER_BUILD_CONTEXT),
            dockerFilePath.getFileName().toString());
    }

    /**
     * Write the docker files generated for the federates added using `addFederate` so far.
     *
     * @param fileConfig The fileConfig.
     *                   fileConfig.srcGenPath is assumed to point at
     *                   where the docker-compose.yml file should be generated.
     */
    public void writeDockerFiles(Path dockerComposeFilePath) throws IOException {
        for (String lfModuleName : moduleNameToData.keySet()) {
            var k = moduleNameToData.get(lfModuleName);
            var dockerFilePath = (Path) k.get(Key.DOCKER_FILE_PATH);
            if (dockerFilePath.toFile().exists()) {
                dockerFilePath.toFile().delete();
            }
            var contents = (String) k.get(Key.DOCKER_FILE_CONTENT);
            FileUtil.writeToFile(contents, dockerFilePath);
            System.out.println(getDockerBuildCommand(
                lfModuleName, dockerFilePath,
                dockerComposeFilePath.getParent(),
                (String) k.get(Key.DOCKER_COMPOSE_SERVICE_NAME)));
        }

        if (isFederated && host != null) {
            appendRtiToDockerComposeServices(
                composeServices,
                "lflang/rti:rti",
                host,
                nFederates
            );
        }
        writeFederatesDockerComposeFile(dockerComposeFilePath, composeServices, "lf");
    }

    /**
     * A template function for generating target-specific docker file content.
     *
     * @param lfModuleName The name of the LF module currently generating.
     */
    protected String generateDockerFileContent(
        String lfModuleName
    ) {
        throw new UnsupportedOperationException("Docker file content is not implemented");
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    public String getDockerComposeCommand() {
        String OS = System.getProperty("os.name").toLowerCase();
        return (OS.indexOf("nux") >= 0) ? "docker-compose" : "docker compose";
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The directory where the docker compose file is generated.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    public String getDockerBuildCommand(
        String lfModuleName,
        Path dockerFilePath,
        Path dockerComposeDir,
        String dockerComposeServiceName
    ) {
        return String.join("\n",
            "Dockerfile for "+lfModuleName+" written to "+dockerFilePath,
            "#####################################",
            "To build the docker image, go to "+dockerComposeDir+" and run:",
            "",
            "    "+getDockerComposeCommand()+" build "+dockerComposeServiceName,
            "",
            "#####################################"
        );
    }

    /**
     * Write the docker-compose.yml for orchestrating the federates.
     * @param the directory to write the docker-compose.yml
     * @param content of the "services" section of the docker-compose.yml
     * @param the name of the network hosting the federation
     */
    private void writeFederatesDockerComposeFile(
        Path dockerComposeFilePath,
        StringBuilder dockerComposeServices,
        String networkName
    ) throws IOException {
        var contents = new CodeBuilder();
        contents.pr(String.join("\n",
            "version: \"3.9\"",
            "services:",
            dockerComposeServices.toString(),
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        ));
        FileUtil.writeToFile(contents.toString(), dockerComposeFilePath);
    }

    /**
     * Append a service to the "services" section of the docker-compose.yml file.
     * @param the content of the "services" section of the docker-compose.yml file.
     * @param the name of the federate to be added to "services".
     * @param the name of the federate's Dockerfile.
     */
    private void appendFederateToDockerComposeServices(StringBuilder dockerComposeServices, String federateName, String context, String dockerFileName) {
        var tab = " ".repeat(4);
        dockerComposeServices.append(tab+federateName+":\n");
        dockerComposeServices.append(tab+tab+"build:\n");
        dockerComposeServices.append(tab+tab+tab+"context: "+context+"\n");
        dockerComposeServices.append(tab+tab+tab+"dockerfile: "+dockerFileName+"\n");
        dockerComposeServices.append(tab+tab+"command: -i 1\n");
    }

    /**
     * Append the RTI to the "services" section of the docker-compose.yml file.
     * @param the content of the "services" section of the docker-compose.yml file.
     * @param the name given to the RTI in the "services" section.
     * @param the tag of the RTI's image.
     * @param the number of federates.
     */
    private void appendRtiToDockerComposeServices(StringBuilder dockerComposeServices, String dockerImageName, String hostName, int n) {
        var tab = " ".repeat(4);
        dockerComposeServices.append(tab+"rti:\n");
        dockerComposeServices.append(tab+tab+"image: "+dockerImageName+"\n");
        dockerComposeServices.append(tab+tab+"hostname: "+hostName+"\n");
        dockerComposeServices.append(tab+tab+"command: -i 1 -n "+n+"\n");
    }
}
