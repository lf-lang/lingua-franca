package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.lflang.FileConfig;
import org.lflang.federated.generator.FedFileConfig;
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
 * - Each target-specific docker generator extends this class and
 *   defines in themselves a class that implements the "GeneratorData"
 *   interface of this class. This is the target code generator-facing API.
 *
 * The purpose of this abstraction design is to contain all
 * docker-specific information inside docker generator classes and
 * prevent docker-related information from polluting target code generators.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
abstract public class DockerGeneratorBase {

    /**
     * Configuration for interactions with the filesystem.
     */
     protected final LFGeneratorContext context;

     protected final Path dockerComposeFilePath;

    protected abstract DockerData generateDockerData();

    /**
     *
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


        /**
         * Return a service description for the "services" section of the docker-compose.yml file.
         */
        public String getServiceDescription(boolean inFederation) {
            var tab = " ".repeat(4);
            StringBuilder svc = new StringBuilder();
            svc.append(tab + this.getComposeServiceName()+":\n");
            svc.append(tab + tab + "build:\n");
            svc.append(tab.repeat(3) + "context: " + this.getBuildContext()+"\n");
            svc.append(tab.repeat(3) + "dockerfile: " + this.getFilePath()+"\n");

            if (inFederation) {
                svc.append(tab+tab+"command: -i 1\n");
            }
            return svc.toString();
        }
    }


    /**
     * The constructor for the base docker file generation class.
     * @param context The context of the code generator.
     */
    public DockerGeneratorBase(LFGeneratorContext context) {
        this.context = context;
        var fileConfig = context.getFileConfig();
        // FIXME: in FedFileConfig we should probably just have an override of getSrcGenPath instead.
        if (fileConfig instanceof FedFileConfig) {
            this.dockerComposeFilePath = fileConfig.fed.getFedSrcGenPath().resolve("docker-compose.yml");
        } else {
            this.dockerComposeFilePath = fileConfig.getSrcGenPath().resolve("docker-compose.yml");
        }
    }

    /**
     * Write the docker files generated for the federates added using `addFederate` so far.
     */
    public void writeDockerFiles() throws IOException {
        var dockerData = generateDockerData();
        writeDockerFile(dockerData);
        writeDockerComposeFile(dockerData.getServiceDescription(false), "lf");
        System.out.println(getDockerBuildCommandMsg(dockerData));
        System.out.println(getDockerComposeUpMsg());

    }

    /**
     * Writes the docker file given the docker data.
     *
     * @param dockerData The docker data as specified in the DockerData class.
     */
    protected void writeDockerFile(DockerData dockerData) throws IOException {
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
        return (OS.contains("nux")) ? "docker-compose" : "docker compose";
    }

    /**
     * Get the command to build the docker images using the compose file.
     * @param dockerData The docker data as specified in the DockerData class.
     * @return The build command printed to the user as to how to build a docker image
     *         using the generated docker file.
     */
    private String getDockerBuildCommandMsg(
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
     * Get the command to launch all containers using the compose file.
     */
    private String getDockerComposeUpMsg() {
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
     * Write the docker-compose.yml file.
     * @param services Section that lists all the services.
     * @param networkName The name of the network to which docker will connect the containers.
     */
    protected void writeDockerComposeFile(
        String services,
        String networkName
    ) throws IOException {

        var contents = new CodeBuilder();
        contents.pr(String.join("\n",
            "version: \"3.9\"",
            "services:",
            services,
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        ));
        FileUtil.writeToFile(contents.toString(), dockerComposeFilePath);
    }

}
