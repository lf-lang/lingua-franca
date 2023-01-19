package org.lflang.generator;

import org.lflang.FileConfig;
import org.lflang.generator.c.CDockerGenerator;
import org.lflang.generator.python.PythonDockerGenerator;
import org.lflang.generator.ts.TSDockerGenerator;


/**
 * A class for generating docker files.
 *
 * @author Marten Lohstroh
 * @author Hou Seng Wong
 */
public abstract class DockerGenerator {

    /**
     * Configuration for interactions with the filesystem.
     */
     protected final LFGeneratorContext context;

    /**
     * The constructor for the base docker file generation class.
     * @param context The context of the code generator.
     */
    public DockerGenerator(LFGeneratorContext context) {
        this.context = context;

    }

    /**
     * Produce a DockerData object.
     * @return
     */
     protected abstract String generateDockerFileContent();

    /**
     * Produce a DockerData object.
     *
     * If the returned object is to be used in a federated context,
     * pass in the file configuration of the federated generator, null otherwise.
     * @param fileConfig Optional argument to point to a federated file configuration
     * @return docker data created based on the context in this instance
     */
    public DockerData generateDockerData(FileConfig fileConfig) {
        var dockerFilePath = context.getFileConfig().getSrcGenPath().resolve("Dockerfile");
        var dockerFileContent = generateDockerFileContent();

        String buildContext;
        String serviceName;
        String containerName;

        if (fileConfig == null) {
            serviceName = "main";
            buildContext = ".";
            containerName = context.getFileConfig().name;
        } else {
            serviceName = context.getFileConfig().name;
            buildContext = serviceName;
            containerName = fileConfig.name + "-" + serviceName;
        }

        return new DockerData(serviceName, containerName, dockerFilePath, dockerFileContent, buildContext);
    }

    public DockerData generateDockerData() {
        return generateDockerData(null);
    }


    public static DockerGenerator dockerGeneratorFactory(LFGeneratorContext context) {
        var target = context.getTargetConfig().target;
        return switch (target) {
            case C, CCPP -> new CDockerGenerator(context);
            case TS -> new TSDockerGenerator(context);
            case Python -> new PythonDockerGenerator(context);
            case CPP, Rust -> throw new IllegalArgumentException("No Docker support for " + target + " yet.");
        };
    }
}
