package org.lflang.generator.python;
import org.lflang.TargetConfig;
import org.lflang.generator.DockerGeneratorBase;

/**
 * Generates the docker file related code for the Python target.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class PythonDockerGenerator extends DockerGeneratorBase {
    TargetConfig targetConfig;
    final String defaultBaseImage = "python:slim";

    public PythonDockerGenerator(boolean isFederated, TargetConfig targetConfig) {
        super(isFederated);
        this.targetConfig = targetConfig;
    }

    /**
     * Generates the contents of the docker file.
     *
     * @param lfModuleName The name of the lingua franca module.
     *                     In unfederated execution, this is fileConfig.name.
     *                     In federated execution, this is typically fileConfig.name + "_" + federate.name
     */
    @Override
    public String generateDockerFileContent(String lfModuleName) {
        var baseImage = defaultBaseImage;
        return String.join("\n",
            "# For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution",
            "FROM "+baseImage,
            "WORKDIR /lingua-franca/"+lfModuleName,
            "RUN set -ex && apt-get update && apt-get install -y python3-pip",
            "COPY . src-gen",
            "RUN cd src-gen && python3 setup.py install && cd ..",
            "ENTRYPOINT [\"python3\", \"src-gen/"+lfModuleName+".py\"]"
        );
    }
}
