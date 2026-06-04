package org.lflang.generator.docker;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/**
 * Generate a Dockerfile for building the rti provided by reactor-c.
 *
 * @author Marten Lohstroh
 */
public class RtiDockerGenerator extends CDockerGenerator {

  public RtiDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  @Override
  protected String generateDockerFileContent() {
    if (context.getTargetConfig().getOrDefault(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST) {
      return super.generateDockerFileContent();
    }

    InputStream stream =
        RtiDockerGenerator.class.getResourceAsStream(
            "/lib/c/reactor-c/core/federated/RTI/rti.Dockerfile");
    var content = new BufferedReader(new InputStreamReader(stream))
        .lines()
        .collect(Collectors.joining("\n"));

    var isTLS = context.getTargetConfig().getOrDefault(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.TLS;
    if (isTLS) {
      content = content.replace(
          "apk add --no-cache gcc musl-dev cmake make git",
          "apk add --no-cache gcc musl-dev cmake make git openssl-dev"
      );
      content = content.replace(
          "cmake ../",
          "cmake -DCOMM_TYPE=TLS ../"
      );
      content = content.replace(
          "FROM ${BASEIMAGE} AS app",
          "FROM ${BASEIMAGE} AS app\nRUN apk add --no-cache openssl"
      );
      content = content.replace(
          "COPY --from=builder /usr/local/bin/RTI /usr/local/bin/RTI",
          "COPY --from=builder /usr/local/bin/RTI /usr/local/bin/RTI\nCOPY credentials/ /lingua-franca/credentials/"
      );
      content = content.replace(
          "ENTRYPOINT [\"/usr/local/bin/RTI\"]",
          "ENTRYPOINT [\"/usr/local/bin/RTI\", \"-tls\", \"credentials/rti.crt\", \"credentials/rti.key\"]"
      );
    }
    return content;
  }

  @Override
  protected String generateCopyOfCredentials() {
    var config = context.getTargetConfig();
    var isSST = config.getOrDefault(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    var isTLS = config.getOrDefault(CommunicationModeProperty.INSTANCE) == CommunicationMode.TLS;

    if (isSST) {
      return "COPY sst/ ./sst/";
    }
    if (isTLS) {
      return "COPY credentials/ /lingua-franca/credentials/";
    }
    return "";
  }

  @Override
  public List<String> defaultEntryPoint() {
    if (context.getTargetConfig().getOrDefault(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST) {
      return List.of("/usr/local/bin/RTI", "-sst", "./sst/rti.config");
    }
    return List.of("/usr/local/bin/RTI");
  }

  @Override
  protected List<String> defaultBuildCommands() {
    return List.of(
        "mkdir -p bin", "cmake -DCOMM_TYPE=SST -S src-gen -B bin", "cd bin", "make all", "cd ..");
  }

  @Override
  protected String generateCopyOfExecutable() {
    var lfModuleName = context.getFileConfig().name;
    return "COPY --from=builder /lingua-franca/%s/bin/RTI /usr/local/bin/RTI"
        .formatted(lfModuleName);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    return "RUN set -ex && apk add --no-cache git";
  }
}
