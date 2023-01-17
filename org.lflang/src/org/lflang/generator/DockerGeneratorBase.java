package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.federated.generator.FedDockerGenerator;
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
 * @author Hou Seng Wong
 * @author Marten Lohstroh
 */
public abstract class DockerGeneratorBase {

    /**
     * Configuration for interactions with the filesystem.
     */
     protected final LFGeneratorContext context;

     protected final Path dockerComposeFilePath;

    public abstract DockerData generateDockerData();

    /**
     * The constructor for the base docker file generation class.
     * @param context The context of the code generator.
     */
    public DockerGeneratorBase(LFGeneratorContext context) {
        this.context = context;
        this.dockerComposeFilePath = context.getFileConfig().getSrcGenPath().resolve("docker-compose.yml");
    }

    //
//    /**
//     * Write the docker files generated for the federates added using `addFederate` so far.
//     */
//    public void writeDockerFiles() throws IOException {
//        var dockerData = generateDockerData();
//        writeDockerComposeFile(dockerData.getServiceDescription(false), "lf");
//        System.out.println(getDockerBuildCommandMsg(dockerData));
//        System.out.println(getDockerComposeUpMsg());
//
//    }


    /**
     * Get the command to build the docker images using the compose file.
     *
     * @return The build command printed to the user as to how to build a docker image
     *         using the generated docker file.
     */
    public String getUsageInstructions() {
        var fileConfig = context.getFileConfig();
        return String.join("\n",
            "#####################################",
            "To build:",
            "    pushd " + dockerComposeFilePath.getParent() + "&& docker compose build",
            "Then, to launch:",
            "    docker compose up",
            "To return to the current working directory:",
            "    popd",
            "#####################################"
        );
    }

    /**
     * Override in FedGenerator
     * @param services
     * @param networkName
     * @return
     */
    protected CodeBuilder generateDockerComposeFile(List<DockerData> services, String networkName) {
        var contents = new CodeBuilder();
        contents.pr(String.join("\n",
            "version: \"3.9\"",
            "services:",
            services.stream().map(
                (data -> data.getServiceDescription(this instanceof FedDockerGenerator))
            ).collect(Collectors.joining("")),
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        ));
        return contents;
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
        var contents = this.generateDockerComposeFile(services, networkName);
        FileUtil.writeToFile(contents.toString(), dockerComposeFilePath);
        System.out.println(getUsageInstructions());
    }

}
