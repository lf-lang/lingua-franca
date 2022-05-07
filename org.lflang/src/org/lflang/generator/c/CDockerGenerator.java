package org.lflang.generator.c;

import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.lflang.TargetConfig;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.util.StringUtil;

/**
 * Generates the docker file related code for the C and CCpp target.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CDockerGenerator extends DockerGeneratorBase {
    private boolean CCppMode;
    private TargetConfig targetConfig;
    private final String defaultBaseImage = "alpine:latest";

    public CDockerGenerator(boolean isFederated, boolean CCppMode, TargetConfig targetConfig) {
        super(isFederated);
        this.CCppMode = CCppMode;
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
    protected String generateDockerFileContent(
        String lfModuleName
    ) {
        var compileCommand = IterableExtensions.isNullOrEmpty(targetConfig.buildCommands) ?
                                 generateDefaultCompileCommand() :
                                 StringUtil.joinObjects(targetConfig.buildCommands, " ");
        var compiler = CCppMode ? "g++" : "gcc";
        var baseImage = targetConfig.dockerOptions.from == null ? defaultBaseImage : targetConfig.dockerOptions.from;
        return String.join("\n",
            "# For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution",
            "FROM "+baseImage+" AS builder",
            "WORKDIR /lingua-franca/"+lfModuleName,
            "RUN set -ex && apk add --no-cache "+compiler+" musl-dev cmake make",
            "COPY . src-gen",
            compileCommand,
            "",
            "FROM "+baseImage,
            "WORKDIR /lingua-franca",
            "RUN mkdir bin",
            "COPY --from=builder /lingua-franca/"+lfModuleName+"/bin/"+lfModuleName+" ./bin/"+lfModuleName,
            "",
            "# Use ENTRYPOINT not CMD so that command-line arguments go through",
            "ENTRYPOINT [\"./bin/"+lfModuleName+"\"]"
        );
    }

    /**
     * Returns the default compile command for the C docker container.
     */
    private String generateDefaultCompileCommand() {
        return String.join("\n",
            "RUN set -ex && \\",
            "mkdir bin && \\",
            "cmake -S src-gen -B bin && \\",
            "cd bin && \\",
            "make all"
        );
    }
}
