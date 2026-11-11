package org.lflang.generator.docker;

import java.io.IOException;
import org.lflang.federated.generator.KubernetesCredentialPlan;
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
    if (KubernetesCredentialPlan.isKubernetes(context.getTargetConfig())) {
      // Only the server jar is part of the image. The keystores, the initial database, the
      // trusted CA certificate and the password input are mounted read-only from Secrets, the
      // properties from a ConfigMap, and the writable database lives in a Pod volume. The
      // password is read from a mounted file on standard input; it is neither in the image nor
      // on the command line.
      String props =
          KubernetesCredentialPlan.AUTH_CONFIG_MOUNT
              + "/"
              + KubernetesCredentialPlan.AUTH_PROPERTIES_KEY;
      String pw =
          KubernetesCredentialPlan.AUTH_SECRET_MOUNT
              + "/"
              + KubernetesCredentialPlan.AUTH_PASSWORD_KEY;
      return String.join(
          "\n",
          "FROM eclipse-temurin:17-jre",
          "WORKDIR /auth",
          "COPY auth-server-jar-with-dependencies.jar .",
          "EXPOSE 21900",
          "ENTRYPOINT [\"sh\", \"-c\", \"exec java -Djava.awt.headless=true -jar"
              + " /auth/auth-server-jar-with-dependencies.jar -p "
              + props
              + " < "
              + pw
              + "\"]");
    }
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
            + " \"--password="
            + federationName
            + "\"]");
  }
}
