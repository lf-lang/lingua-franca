package org.lflang.generator.ts

import org.lflang.generator.DockerGeneratorBase;
import org.lflang.FileConfig;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
class TSDockerGenerator : DockerGeneratorBase(false) {
    /**
     * The interface for data from the Typescript code generator.
     */
    public enum class TSGeneratorData : GeneratorData {
        /**
         * A `TSFileConfig` object that is the "tsFileConfig"
         * in TSGenerator.
         */
        TS_FILE_CONFIG,
        /**
         * A `String` object that is the name of the
         * typescript file generated.
         */
        TS_FILE_NAME,
    }

    /**
     * Translate data from the code generator to a map.
     *
     * @return data from the code generator put in a Map.
     */
    public fun fromData(
        tsFileName: String,
        tsFileConfig: TSFileConfig
    ): Map<GeneratorData, Any> {
        var generatorData = hashMapOf<GeneratorData, Any>()
        generatorData.put(TSGeneratorData.TS_FILE_NAME, tsFileName)
        generatorData.put(TSGeneratorData.TS_FILE_CONFIG, tsFileConfig)
        return generatorData
    }

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData Enum.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData Enum
     */
    override protected fun generateDockerData(generatorData: Map<GeneratorData, Any>) : Map<DockerData, Any> {
        var dockerData = hashMapOf<DockerData, Any>()
        var tsFileName = getTsFileName(generatorData)
        dockerData.put(DockerData.DOCKER_FILE_PATH, getTsFileConfig(generatorData).tsDockerFilePath(tsFileName))
        dockerData.put(DockerData.DOCKER_FILE_CONTENT, generateDockerFileContent(generatorData))
        dockerData.put(DockerData.DOCKER_COMPOSE_SERVICE_NAME, if (isFederated) "NOT IMPLEMENTED" else tsFileName.toLowerCase())
        dockerData.put(DockerData.DOCKER_BUILD_CONTEXT, if (isFederated) "NOT IMPLEMENTED" else ".")
        dockerData.put(DockerData.DOCKER_CONTAINER_NAME, if (isFederated) "NOT IMPLEMENTED" else tsFileName.toLowerCase())
        return dockerData;
    }

    /**
     * Return the value of "TS_FILE_NAME" in generatorData.
     */
    private fun getTsFileName(generatorData: Map<GeneratorData, Any>) : String {
        return generatorData.get(TSGeneratorData.TS_FILE_NAME) as String
    }

    /**
     * Return the value of "TS_FILE_CONFIG" in generatorData.
     */
    private fun getTsFileConfig(generatorData: Map<GeneratorData, Any>) : TSFileConfig {
        return generatorData.get(TSGeneratorData.TS_FILE_CONFIG) as TSFileConfig
    }

    /**
    * Returns the content of the docker file for [tsFileName].
    */
    private fun generateDockerFileContent(generatorData: Map<GeneratorData, Any>): String {
        var tsFileName = getTsFileName(generatorData)
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$tsFileName
        |COPY . .
        |ENTRYPOINT ["node", "dist/$tsFileName.js"]
        """
        return dockerFileContent.trimMargin()
    }
}
