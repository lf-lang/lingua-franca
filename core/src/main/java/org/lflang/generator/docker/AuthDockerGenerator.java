package org.lflang.generator.docker;

import java.io.IOException;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.util.FileUtil;



public class AuthDockerGenerator {
    private final LFGeneratorContext context;

    public AuthDockerGenerator(LFGeneratorContext context) {
        this.context = context;
    }
    
    public void generate() throws IOException {
        var srcGenAuthPath = context.getFileConfig().getSrcGenPath().resolve("auth");
        var dockerFileContent = generateDockerFileContent();
        FileUtil.writeToFile(dockerFileContent, srcGenAuthPath.resolve("Dockerfile"));

    }
    private String generateDockerFileContent() {
        var federationName = context.getFileConfig().name;
        return String.join(
            "\n",
            "FROM eclipse-temurin:17-jre",
            "WORKDIR /auth",
            "COPY credentials/ ./credentials/",
            "COPY databases/ ./databases/",
            "COPY properties/ ./properties/",
            "COPY auth-server-jar-with-dependencies.jar .",
            "EXPOSE 21900",
            "ENTRYPOINT [\"java\", \"-jar\", \"auth-server-jar-with-dependencies.jar\","
            + " \"-p\", \"properties/exampleAuth101.properties\","
            + " \"--password=" + federationName + "\"]"
        );
    }
}