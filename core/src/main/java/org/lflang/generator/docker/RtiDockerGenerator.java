package org.lflang.generator.docker;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.stream.Collectors;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;
import java.util.List;

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
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST) {
      return super.generateDockerFileContent();
    }

    InputStream stream =
        RtiDockerGenerator.class.getResourceAsStream(
            "/lib/c/reactor-c/core/federated/RTI/rti.Dockerfile");
    return new BufferedReader(new InputStreamReader(stream))
        .lines()
        .collect(Collectors.joining("\n"));
  }

  @Override
  protected String generateCopyOfCredentials() {
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST) {
      return "COPY sst/ ./sst/";
    }
    return "";
  }

  @Override
  public List<String> defaultEntryPoint() {
    if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST) {
      return List.of("/usr/local/bin/RTI", "-sst", "./sst/rti.config");
    }
    return List.of("/usr/local/bin/RTI");
  }

  @Override
  protected List<String> defaultBuildCommands() {
    return List.of(
        "mkdir -p bin",
        "cmake -DCOMM_TYPE=SST -S src-gen -B bin",
        "cd bin",
        "make all",
        "cd ..");
  }

  @Override
  protected String generateCopyOfExecutable() {
    var lfModuleName = context.getFileConfig().name;
    return "COPY --from=builder /lingua-franca/%s/bin/RTI /usr/local/bin/RTI".formatted(lfModuleName);
  }

  @Override
  protected String generateRunForInstallingDeps() {
    return "RUN set -ex && apk add --no-cache git";
  }
  
}
