package org.lflang.generator.ts

import org.lflang.generator.DockerGeneratorBase
import org.lflang.FileConfig
import org.lflang.federated.generator.FedFileConfig
import org.lflang.generator.DockerData
import org.lflang.generator.LFGeneratorContext

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 */
class TSDockerGenerator(context: LFGeneratorContext) : DockerGeneratorBase(context) {

//    /**
//     * Translate data from the code generator to docker data as
//     * specified in the DockerData class.
//     *
//     * @return docker data as specified in the DockerData class
//     */
//    override fun generateDockerData(containerName: String) : DockerData {
//        val dockerFilePath = context.fileConfig.srcGenPath.resolve("Dockerfile")
//        val dockerFileContent = generateDockerFileContent("${context.fileConfig.name}")
//        val buildContext = if (context.fileConfig is FedFileConfig) context.fileConfig.name else "."
//        val serviceName = if (context.fileConfig is FedFileConfig) context.fileConfig.name else "main"
//
//        return DockerData(serviceName, containerName, dockerFilePath, dockerFileContent, buildContext)
//    }


    /**
    * Returns the content of the docker file for [tsFileName].
    */
    override fun generateDockerFileContent(): String {
        val name = context.fileConfig.name
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$name
        |COPY . .
        |ENTRYPOINT ["node", "dist/$name.js"]
        """
        return dockerFileContent.trimMargin()
    }
}
