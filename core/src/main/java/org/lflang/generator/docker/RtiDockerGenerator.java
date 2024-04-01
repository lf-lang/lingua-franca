package org.lflang.generator.docker;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.stream.Collectors;
import org.lflang.generator.LFGeneratorContext;

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
    InputStream stream =
        RtiDockerGenerator.class.getResourceAsStream(
            "/lib/c/reactor-c/core/federated/RTI/rti.Dockerfile");
    return new BufferedReader(new InputStreamReader(stream))
        .lines()
        .collect(Collectors.joining("\n"));
  }

  @Override
  public String baseImage() {
    return defaultImage();
  }
}
