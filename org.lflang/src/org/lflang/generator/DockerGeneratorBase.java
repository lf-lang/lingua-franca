package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.util.FileUtil;

public class DockerGeneratorBase {
    protected StringBuilder composeServices;
    protected Map<String, Map<Key, Object>> moduleNameToData;
    protected boolean isFederated;
    protected int nFederates;
    protected String host = null;
    
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
