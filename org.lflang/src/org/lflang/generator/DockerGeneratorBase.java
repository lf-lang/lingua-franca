package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.federated.generator.FedDockerGenerator;
import org.lflang.generator.c.CDockerGenerator;
import org.lflang.generator.python.PythonDockerGenerator;
import org.lflang.generator.ts.TSDockerGenerator;
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
     *
     * @param networkName
     * @return
     */
    protected String generateDockerNetwork(String networkName) {
        return String.join("\n",
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        );
    }

    protected String generateDockerServices(List<DockerData> services) {
        return String.join("\n",
                                "version: \"3.9\"",
                                "services:",
                                services.stream().map(
                                    (data -> data.getServiceDescription(this instanceof FedDockerGenerator))
                                ).collect(Collectors.joining("\n"))
        );
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

    public static DockerGeneratorBase dockerGeneratorFactory(LFGeneratorContext context) {
        var target = context.getTargetConfig().target;
        return switch (target) {
            case C, CCPP -> new CDockerGenerator(context);
            case TS -> new TSDockerGenerator(context);
            case Python -> new PythonDockerGenerator(context);
            case CPP, Rust -> throw new IllegalArgumentException("No Docker support for " + target + " yet.");
        };
    }
}
