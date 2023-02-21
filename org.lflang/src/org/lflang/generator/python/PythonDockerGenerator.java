
package org.lflang.generator.python;

import org.lflang.TargetConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CDockerGenerator;

/**
 * Generates the docker file related code for the Python target.
 *
 * @author Hou Seng Wong
 */
public class PythonDockerGenerator extends CDockerGenerator {
    final String defaultBaseImage = "python:slim";

    public PythonDockerGenerator(LFGeneratorContext context) {
        super(context);
    }

    /**
     * Generates the contents of the docker file.
     */
    @Override
    protected String generateDockerFileContent() {
        var baseImage = defaultBaseImage;
        return String.join("\n",
            "# For instructions, see: https://www.lf-lang.org/docs/handbook/containerized-execution?target=py",
            "FROM "+baseImage,
            "WORKDIR /lingua-franca/"+context.getFileConfig().name,
            "RUN set -ex && apt-get update && apt-get install -y python3-pip && pip install cmake",
            "COPY . src-gen",
            super.generateDefaultCompileCommand(),
            "ENTRYPOINT [\"python3\", \"src-gen/"+context.getFileConfig().name+".py\"]"
        );
    }
}
