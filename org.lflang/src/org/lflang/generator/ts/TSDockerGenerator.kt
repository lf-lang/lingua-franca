package org.lflang.generator.ts

import org.lflang.generator.DockerGeneratorBase;

/**
 * Generates the docker file related code for the Typescript target.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
class TSDockerGenerator(
    private val tsFileName: String
 ) : DockerGeneratorBase(false) {
    /**
    * Returns the content of the docker file for [tsFileName].
    */
    override fun generateDockerFileContent(lfModuleName: String): String {
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$tsFileName
        |COPY . .
        |ENTRYPOINT ["node", "dist/$tsFileName.js"]
        """
        return dockerFileContent.trimMargin()
    }
}
