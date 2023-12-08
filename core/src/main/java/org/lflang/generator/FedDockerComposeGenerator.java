package org.lflang.generator;

import org.lflang.target.property.TracingProperty;

import java.util.List;

/**
 * A docker-compose configuration generator for a federated program.
 *
 * @author Marten Lohstroh
 */
public class FedDockerComposeGenerator extends DockerComposeGenerator {

  /** The host on which to run the rti. */
  private String rtiHost;

  /** The name of this federation. */
  private String containerName;

  public FedDockerComposeGenerator(LFGeneratorContext context, String rtiHost) {
    super(context);
    this.rtiHost = rtiHost;
    this.containerName = context.getFileConfig().name;
  }

  @Override
  protected String generateDockerServices(List<DockerData> services) {
    var tracing = "";
    if (context.getTargetConfig().getOrDefault(TracingProperty.INSTANCE).isEnabled()) {
      tracing = " -t ";
    }
    return """
            %s\
                rti:
                    image: "lflang/rti:rti"
                    hostname: "%s"
                    command: "-i 1 %s -n %s"
                    container_name: "%s-rti"
            """
        .formatted(
            super.generateDockerServices(services), this.rtiHost, tracing, services.size(), containerName);
  }

  @Override
  protected String getServiceDescription(DockerData data) {
    return """
            %s\
                    command: "-i 1"
                    depends_on:
                        - rti
            """
        .formatted(super.getServiceDescription(data));
  }

  @Override
  protected String getServiceName(DockerData data) {
    return data.serviceName;
  }

  @Override
  protected String getBuildContext(DockerData data) {
    return data.serviceName;
  }

  @Override
  protected String getContainerName(DockerData data) {
    return this.containerName + "-" + data.serviceName;
  }
}
