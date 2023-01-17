package org.lflang.generator.ts

import org.lflang.generator.DockerGeneratorBase
import org.lflang.FileConfig
import org.lflang.generator.DockerData
import org.lflang.generator.LFGeneratorContext

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 */
class TSDockerGenerator(context: LFGeneratorContext) : DockerGeneratorBase(context) {

    /**
     * Translate data from the code generator to docker data as
     * specified in the DockerData class.
     *
     * @return docker data as specified in the DockerData class
     */
    override fun generateDockerData() : DockerData {
        val tsFileName = "${context.fileConfig.name}.ts"
        val dockerFilePath = context.fileConfig.srcGenPath.resolve("$tsFileName.Dockerfile")
        val dockerFileContent = generateDockerFileContent(tsFileName)
        val dockerBuildContext = "."
        return DockerData(dockerFilePath, dockerFileContent, dockerBuildContext)
    }

    /**
    * Returns the content of the docker file for [tsFileName].
    */
    private fun generateDockerFileContent(tsFileName: String): String {
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$tsFileName
        |COPY . .
        |ENTRYPOINT ["node", "dist/$tsFileName.js"]
        """
        return dockerFileContent.trimMargin()
    }
}
