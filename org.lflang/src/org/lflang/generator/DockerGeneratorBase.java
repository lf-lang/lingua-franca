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
     *
     * @author{Hou Seng Wong <housengw@berkeley.edu>}
     */
    protected enum Key {
        DOCKER_FILE_PATH,
        DOCKER_FILE_CONTENT,
        DOCKER_COMPOSE_SERVICE_NAME,
        DOCKER_BUILD_CONTEXT,
    }

    public DockerGeneratorBase(boolean isFederated) {
        moduleNameToData = new HashMap<>();
        composeServices = new StringBuilder();
        this.isFederated = isFederated;
        nFederates = 0;
    }

    public void setHost(String host) {
        this.host = host;
    }

    /**
     * Adds a federate to the list of federates to generate docker files for.
     *
     * @param lfModuleName The module name of the federate.
     * @param federateName The name of the federate's reactor.
     * @param fileConfig The file config.
     *                   fileConfig.srcGenPath is assumed to point at
     *                   where the docker file should be generated.
     * @param targetConfig The target config.
     */
    public void addFederate(
        String lfModuleName,
        String federateName,
        FileConfig fileConfig,
        TargetConfig targetConfig
    ) {
        Map<Key, Object> k = new HashMap<>();
        Path dockerPath = fileConfig.getSrcGenPath().resolve(lfModuleName + ".Dockerfile");
        k.put(Key.DOCKER_FILE_PATH, dockerPath);

        var dockerFileContent = generateDockerFileContent(lfModuleName);
        k.put(Key.DOCKER_FILE_CONTENT, dockerFileContent);
        k.put(Key.DOCKER_COMPOSE_SERVICE_NAME, isFederated ? federateName : lfModuleName.toLowerCase());
        k.put(Key.DOCKER_BUILD_CONTEXT, isFederated ? federateName : ".");
        moduleNameToData.put(lfModuleName, k);
        nFederates++;

        DockerComposeGenerator.appendFederateToDockerComposeServices(
            composeServices,
            (String) k.get(Key.DOCKER_COMPOSE_SERVICE_NAME),
            (String) k.get(Key.DOCKER_BUILD_CONTEXT),
            dockerPath.getFileName().toString());
    }

    /**
     * Write the docker files generated for the federates added using `addFederate` so far.
     *
     * @param fileConfig The fileConfig.
     *                   fileConfig.srcGenPath is assumed to point at
     *                   where the docker-compose.yml file should be generated.
     */
    public void writeDockerFiles(FileConfig fileConfig) throws IOException {
        var dockerComposeFilePath = fileConfig.getSrcGenPath().resolve("docker-compose.yml");
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
            DockerComposeGenerator.appendRtiToDockerComposeServices(
                composeServices,
                "lflang/rti:rti",
                host,
                nFederates
            );
        }
        DockerComposeGenerator.writeFederatesDockerComposeFile(dockerComposeFilePath, composeServices, "lf");
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
}
