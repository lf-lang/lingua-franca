package org.lflang.generator.ts

import java.util.StringJoiner

import org.lflang.generator.DockerGeneratorBase;

/**
 * Generate parameters for TypeScript target.
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