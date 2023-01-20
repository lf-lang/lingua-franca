package org.lflang.generator;

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
     * Generate the contents of a Dockerfile.
     */
     protected abstract String generateDockerFileContent();

    /**
     * Produce a DockerData object.
     * If the returned object is to be used in a federated context,
     * pass in the file configuration of the federated generator, null otherwise.
     * @return docker data created based on the context in this instance
     */
    public DockerData generateDockerData() {
        var name = context.getFileConfig().name;
        var dockerFilePath = context.getFileConfig().getSrcGenPath().resolve("Dockerfile");
        var dockerFileContent = generateDockerFileContent();

        return new DockerData(name, dockerFilePath, dockerFileContent);
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
