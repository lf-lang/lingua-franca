package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.DockerProperty.DockerOptions;
import org.lflang.target.property.TracingProperty;

/**
 * A docker-compose configuration generator for a federated program.
 *
 * @author Marten Lohstroh
 */
public class FedDockerComposeGenerator extends DockerComposeGenerator {

  /** The host on which to run the rti. */
  private final String rtiHost;

  /** The name of this federation. */
  private final String containerName;

  private final String rtiImage;

  public FedDockerComposeGenerator(LFGeneratorContext context, String rtiHost) {
    super(context);
    this.rtiHost = rtiHost;
    this.containerName = context.getFileConfig().name;
    this.rtiImage = context.getTargetConfig().get(DockerProperty.INSTANCE).rti();
  }

  @Override
  protected String generateDockerServices(List<DockerData> services) {
    var tracing = "";
    if (context.getTargetConfig().getOrDefault(TracingProperty.INSTANCE).isEnabled()) {
      tracing = " -t ";
    }
    var attributes =
        """
                image: "%s"
                hostname: "%s"
                command: "-i 1 %s -n %s"
                container_name: "%s-rti"
        """
            .formatted(this.rtiImage, this.rtiHost, tracing, services.size(), containerName);
    if (this.rtiImage.equals(DockerOptions.LOCAL_RTI_IMAGE)) {
      return """
             %s\
                 rti:
                     build:
                         context: "rti"
             %s
             """
          .formatted(super.generateDockerServices(services), attributes);
    } else {
      return """
             %s\
                 rti:
             %s
             """
          .formatted(super.generateDockerServices(services), attributes);
    }
  }

  @Override
  protected String getServiceDescription(DockerData data) {
    return """
           %s\
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
