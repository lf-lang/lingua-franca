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
}