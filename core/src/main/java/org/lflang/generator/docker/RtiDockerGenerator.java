package org.lflang.generator.docker;

import org.lflang.generator.LFGeneratorContext;

/**
 * Generate the docker file related code for the C and CCpp target.
 *
 * @author Hou Seng Wong
 */
public class RtiDockerGenerator extends DockerGenerator {

  /**
   * The constructor for the base docker file generation class.
   *
   * @param context The context of the code generator.
   */
  public RtiDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  /** Generate the contents of the docker file. */
  @Override
  protected String generateDockerFileContent() {
    return """
        # Docker file for building the image of the rti
        FROM alpine:latest
        COPY core /reactor-c/core
        COPY include /reactor-c/include
        WORKDIR /reactor-c/core/federated/RTI
        RUN set -ex && apk add --no-cache gcc musl-dev cmake make && \\
            mkdir build && \\
            cd build && \\
            cmake ../ && \\
            make && \\
            make install

        # Use ENTRYPOINT not CMD so that command-line arguments go through
        ENTRYPOINT ["RTI"]
        """;
  }
}
