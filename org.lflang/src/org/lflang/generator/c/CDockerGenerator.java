package org.lflang.generator.c;

import java.nio.file.Path;

/**
 * Generates the docker file related code for the C and CCpp target.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CDockerGenerator {
    public static String generateDockerFileContent(
        String topLevelName, 
        String baseImage, 
        String compiler,
        String compileCommand, 
        Path srcGenPath
    ) {
        return String.join("\n", 
            "# Generated docker file for "+topLevelName+" in "+srcGenPath+".",
            "# For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution",
            "FROM "+baseImage+" AS builder",
            "WORKDIR /lingua-franca/"+topLevelName,
            "RUN set -ex && apk add --no-cache "+compiler+" musl-dev cmake make",
            "COPY . src-gen",
            compileCommand,
            "",
            "FROM "+baseImage,
            "WORKDIR /lingua-franca",
            "RUN mkdir bin",
            "COPY --from=builder /lingua-franca/"+topLevelName+"/bin/"+topLevelName+" ./bin/"+topLevelName,
            "",
            "# Use ENTRYPOINT not CMD so that command-line arguments go through",
            "ENTRYPOINT [\"./bin/"+topLevelName+"\"]"
        );
    }

    public static String generateDefaultCompileCommand() {
        return String.join("\n", 
            "RUN set -ex && \\",
            "mkdir bin && \\",
            "cmake -S src-gen -B bin && \\",
            "cd bin && \\",
            "make all"
        );
    }
}
