package org.lflang.generator.c;

import java.util.HashMap;
import java.util.Map;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.util.StringUtil;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CDockerGenerator extends DockerGeneratorBase {
    private boolean CCppMode;
    private TargetConfig targetConfig;
    private final String defaultBaseImage = "alpine:latest";

    /**
     * The interface for data from the C code generator.
     */
    protected enum CGeneratorData implements GeneratorData {
        LF_MODULE_NAME,
        FEDERATE_NAME,
        FILE_CONFIG
    }

    public CDockerGenerator(boolean isFederated, boolean CCppMode, TargetConfig targetConfig) {
        super(isFederated);
        this.CCppMode = CCppMode;
        this.targetConfig = targetConfig;
    }

    /**
     * Translate data from the code generator to a map.
     *
     * @return data from the code generator put in a Map.
     */
    public Map<GeneratorData, Object> fromData(
        String lfModuleName,
        String federateName,
        FileConfig fileConfig
    ) {
        Map<GeneratorData, Object> m = new HashMap<>();
        m.put(CGeneratorData.LF_MODULE_NAME, lfModuleName);
        m.put(CGeneratorData.FEDERATE_NAME, federateName);
        m.put(CGeneratorData.FILE_CONFIG, fileConfig);
        return m;
    }

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData Enum.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData Enum
     */
    protected Map<DockerData, Object> generateDockerData(Map<GeneratorData, Object> generatorData) {
        Map<DockerData, Object> dockerData = new HashMap<>();
        var lfModuleName = getLfModuleName(generatorData);
        var federateName = getFederateName(generatorData);
        var fileConfig = getFileConfig(generatorData);
        dockerData.put(DockerData.DOCKER_FILE_PATH, fileConfig.getSrcGenPath().resolve(lfModuleName + ".Dockerfile"));
        dockerData.put(DockerData.DOCKER_FILE_CONTENT, generateDockerFileContent(generatorData));
        dockerData.put(DockerData.DOCKER_COMPOSE_SERVICE_NAME, isFederated ? federateName : lfModuleName.toLowerCase());
        dockerData.put(DockerData.DOCKER_BUILD_CONTEXT, isFederated ? federateName : ".");
        dockerData.put(DockerData.DOCKER_CONTAINER_NAME, isFederated ? federateName : lfModuleName.toLowerCase());
        return dockerData;
    }

    /**
     * Generate the contents of the docker file.
     *
     * @param lfModuleName The name of the lingua franca module.
     *                     In unfederated execution, this is fileConfig.name.
     *                     In federated execution, this is typically fileConfig.name + "_" + federate.name
     */
    private String generateDockerFileContent(Map<GeneratorData, Object> generatorData) {
        var lfModuleName = getLfModuleName(generatorData);
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
     * Return the value of "LF_MODULE_NAME" in generatorData.
     */
    private String getLfModuleName(Map<GeneratorData, Object> generatorData) {
        return (String) generatorData.get(CGeneratorData.LF_MODULE_NAME);
    }

    /**
     * Return the value of "FEDERATE_NAME" in generatorData.
     */
    private String getFederateName(Map<GeneratorData, Object> generatorData) {
        return (String) generatorData.get(CGeneratorData.FEDERATE_NAME);
    }

    /**
     * Return the value of "FILE_CONFIG" in generatorData.
     */
    private FileConfig getFileConfig(Map<GeneratorData, Object> generatorData) {
        return (FileConfig) generatorData.get(CGeneratorData.FILE_CONFIG);
    }

    /**
     * Return the default compile command for the C docker container.
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
