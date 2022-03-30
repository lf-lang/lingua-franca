package org.lflang.generator.python;

import java.nio.file.Path;

public class PythonDockerGenerator {
    public static String generateDockerFileContent(String topLevelName, Path srcGenPath) {
        return String.join("\n", 
            "# Generated docker file for "+topLevelName+".lf in "+srcGenPath+".",
            "# For instructions, see: https://github.com/icyphy/lingua-franca/wiki/Containerized-Execution",
            "FROM python:slim",
            "WORKDIR /lingua-franca/"+topLevelName,
            "RUN set -ex && apt-get update && apt-get install -y python3-pip",
            "COPY . src-gen",
            "RUN cd src-gen && python3 setup.py install && cd ..",
            "ENTRYPOINT [\"python3\", \"src-gen/"+topLevelName+".py\"]"
        );
    }
}
