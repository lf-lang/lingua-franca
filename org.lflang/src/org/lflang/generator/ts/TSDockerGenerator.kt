package org.lflang.generator.ts

import org.lflang.generator.DockerGenerator
import org.lflang.generator.LFGeneratorContext

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author Hou Seng Wong
 */
class TSDockerGenerator(context: LFGeneratorContext) : DockerGenerator(context) {

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
