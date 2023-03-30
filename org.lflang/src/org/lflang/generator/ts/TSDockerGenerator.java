package org.lflang.generator.ts;

import org.lflang.generator.DockerGenerator;
import org.lflang.generator.LFGeneratorContext;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 */
public class TSDockerGenerator extends DockerGenerator {

    /** Construct a new Docker generator. */
    public TSDockerGenerator(LFGeneratorContext context) {
        super(context);
    }

    /**
    * Return the content of the docker file for [tsFileName].
    */
    public String generateDockerFileContent() {
        return """
        |FROM node:alpine
        |WORKDIR /linguafranca/$name
        |COPY . .
        |ENTRYPOINT ["node", "dist/%s.js"]
        """.formatted(context.getFileConfig().name);
    }
}
