package org.lflang.generator.docker;

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
    return """
        # Docker file for building the image of the rti
        FROM %s
        COPY core /reactor-c/core
        COPY include /reactor-c/include
        WORKDIR /reactor-c/core/federated/RTI
        %s
        RUN mkdir -p build && \\
            cd build && \\
            cmake ../ && \\
            make && \\
            make install

        # Use ENTRYPOINT not CMD so that command-line arguments go through
        ENTRYPOINT ["RTI"]
        """
        .formatted(baseImage(), generateRunForBuildDependencies());
  }

  @Override
  public String baseImage() {
    return defaultImage();
  }
}
