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
    public class TSGeneratorData(
        private val tsFileName: String,
        private val tsFileConfig: TSFileConfig
    ) : GeneratorData {
        fun getTsFileName(): String = tsFileName
        fun getTsFileConfig(): TSFileConfig = tsFileConfig
    }

    /**
     * Translate data from the code generator to a map.
     *
     * @return data from the code generator put in a Map.
     */
    public fun fromData(
        tsFileName: String,
        tsFileConfig: TSFileConfig
    ): GeneratorData {
        return TSGeneratorData(tsFileName, tsFileConfig)
    }

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData class.
     *
     * @param generatorData Data from the code generator.
     * @return docker data as specified in the DockerData class
     */
    override protected fun generateDockerData(generatorData: GeneratorData) : DockerData {
        if (isFederated) {
            throw UnsupportedOperationException(
                "Federated Docker file generation is not supported in the TypeScript target"
            );
        }
        var tsGeneratorData = generatorData as TSGeneratorData
        var tsFileName = tsGeneratorData.getTsFileName()
        var dockerFilePath = tsGeneratorData.getTsFileConfig().tsDockerFilePath(tsFileName)
        var dockerFileContent = generateDockerFileContent(tsGeneratorData)
        var dockerComposeServiceName = if (isFederated) "NOT IMPLEMENTED" else tsFileName.toLowerCase()
        var dockerBuildContext = if (isFederated) "NOT IMPLEMENTED" else "."
        var dockerContainerName = if (isFederated) "NOT IMPLEMENTED" else tsFileName.toLowerCase()
        return DockerData(
            dockerFilePath, dockerFileContent, dockerComposeServiceName,
            dockerBuildContext, dockerContainerName
        );
    }

    /**
    * Returns the content of the docker file for [tsFileName].
    */
    private fun generateDockerFileContent(generatorData: TSGeneratorData): String {
        var tsFileName = generatorData.getTsFileName()
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$tsFileName
        |COPY . .
        |ENTRYPOINT ["node", "dist/$tsFileName.js"]
        """
        return dockerFileContent.trimMargin()
    }
}
