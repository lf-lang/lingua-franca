package org.lflang.generator.c;

import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.util.StringUtil;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author Hou Seng Wong <housengw@berkeley.edu>
 */
public class Docker extends DockerGeneratorBase {
    private static final String DEFAULT_BASE_IMAGE = "alpine:latest";
    private final boolean CCppMode;
    private final TargetConfig targetConfig;

    /**
     * The interface for data from the C code generator.
     *
     * @param lfModuleName The name of the LF module in CGenerator.
     *                     Typically, this is "fileConfig.name + _ + federate.name"
     *                     in federated execution and "fileConfig.name" in non-federated
     *                     execution.
     * @param federateName The value of "currentFederate.name" in CGenerator.
     * @param fileConfig   The value of "fileConfig" in CGenerator.
     */
    public record CGeneratorData(
        String lfModuleName, String federateName, FileConfig fileConfig
    ) implements GeneratorData {}

    public Docker(boolean isFederated, boolean CCppMode, TargetConfig targetConfig) {
        super(isFederated);
        this.CCppMode = CCppMode;
        this.targetConfig = targetConfig;
    }

    /**
     * Translate data from the code generator to a map.
     *
     * @return data from the code generator put in a Map.
     */
    public CGeneratorData fromData(
        String lfModuleName,
        String federateName,
        FileConfig fileConfig
    ) {
        return new CGeneratorData(lfModuleName, federateName, fileConfig);
    }

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData class.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData class
     */
    @Override
    protected DockerData generateDockerData(GeneratorData generatorData) {
        CGeneratorData cGeneratorData = (CGeneratorData) generatorData;
        var lfModuleName = cGeneratorData.lfModuleName();
        var federateName = cGeneratorData.federateName();
        var fileConfig = cGeneratorData.fileConfig();
        var dockerFilePath = fileConfig.getSrcGenPath().resolve(lfModuleName + ".Dockerfile");
        var dockerFileContent = generateDockerFileContent(cGeneratorData);
        var dockerBuildContext = isFederated ? federateName : ".";
        return new DockerData(dockerFilePath, dockerFileContent,dockerBuildContext);
    }

    /**
     * Generate the contents of the docker file.
     *
     * @param generatorData Data from the code generator.
     */
    protected String generateDockerFileContent(CGeneratorData generatorData) {
        var lfModuleName = generatorData.lfModuleName();
        var compileCommand = IterableExtensions.isNullOrEmpty(targetConfig.buildCommands) ?
                                 generateDefaultCompileCommand(generatorData.fileConfig) :
                                 StringUtil.joinObjects(targetConfig.buildCommands, " ");
        var compiler = CCppMode ? "g++" : "gcc";
        var baseImage = targetConfig.dockerOptions.from == null ? DEFAULT_BASE_IMAGE : targetConfig.dockerOptions.from;
        return String.join("\n",
            "# For instructions, see: https://www.lf-lang.org/docs/handbook/containerized-execution",
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
            "ENTRYPOINT [\"./bin/"+lfModuleName+"\"]",
            ""
        );
    }

    /**
     * Return the default compile command for the C docker container.
     */
    private String generateDefaultCompileCommand(FileConfig fileConfig) {
        return String.join("\n",
            "RUN set -ex && \\",
            "mkdir bin && \\",
            "cmake " + String.join(" ", CCompiler.cmakeOptions(targetConfig, fileConfig))
                + " -S src-gen -B bin && \\",
            "cd bin && \\",
            "make all"
        );
    }
}
