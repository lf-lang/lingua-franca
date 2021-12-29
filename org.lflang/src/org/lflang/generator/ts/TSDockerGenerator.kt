package org.lflang.generator.ts

import java.util.StringJoiner

/**
 * Generate parameters for TypeScript target.
 */
class TSDockerGenerator (
    private val tsFileName: String
 ) {
    fun generateDockerFileContent(): String {
        val dockerFileContent = """
        |FROM node:alpine
        |WORKDIR /linguafranca/$tsFileName
        |COPY . .
        |ENTRYPOINT ["node", "dist/$tsFileName.js"]
        """
        return dockerFileContent.trimMargin()
    }

    fun generateDockerComposeFileContent(): String {
        val dockerComposeFileContent = """
        |version: "3.9"
        |services:
        |    ${tsFileName.toLowerCase()}:
        |        build:
        |            context: .
        |            dockerfile: HelloWorldContainerized.Dockerfile
        |networks:
        |    default:
        |        name: lf
        """
        return dockerComposeFileContent.trimMargin()
    }
}