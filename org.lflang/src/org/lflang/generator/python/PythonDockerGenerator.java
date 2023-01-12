
package org.lflang.generator.python;

import org.lflang.TargetConfig;
import org.lflang.generator.c.CDockerGenerator;

/**
 * Generates the docker file related code for the Python target.
 *
 * @author Hou Seng Wong
 */
public class PythonDockerGenerator extends CDockerGenerator {
    final String defaultBaseImage = "python:slim";

    public PythonDockerGenerator(boolean isFederated, TargetConfig targetConfig) {
        super(isFederated, false, targetConfig);
    }

    /**
     * Generates the contents of the docker file.
     *
     * @param generatorData Data from the code generator.
     */
    @Override
    protected String generateDockerFileContent(CGeneratorData generatorData) {
        var baseImage = defaultBaseImage;
        return String.join("\n",
            "# For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution",
            "FROM "+baseImage,
            "WORKDIR /lingua-franca/"+generatorData.lfModuleName(),
            "RUN set -ex && apt-get update && apt-get install -y python3-pip && pip install cmake",
            "COPY . src-gen",
            super.generateDefaultCompileCommand(),
            "ENTRYPOINT [\"python3\", \"src-gen/"+generatorData.lfModuleName()+".py\"]"
        );
    }
}
