package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.DockerProperty.DockerOptions;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/**
 * A docker-compose configuration generator for a federated program.
 *
 * @author Marten Lohstroh
 * @ingroup Docker
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
    var isSST = context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    var authService = isSST ? """
                 auth:
                     networks:
                         default: 
                             ipv4_address: "172.21.0.2"
                     build:
                         context: "auth"
                     container_name: "%s-auth"
                     ports:
                         - "21900:21900"
                     healthcheck:
                         test: ["CMD", "nc", "-z", "localhost", "21900"]
                         interval: 2s
                         retries: 15
             """.formatted(containerName) : "";
    var rtiDependsOn = isSST ? """
                     depends_on:
                         - auth
             """ : "";
    if (this.rtiImage.equals(DockerOptions.LOCAL_RTI_IMAGE)) {
      var rtiBuildContext = "context: \"rti\"";
      return """
             %s\
                 rti:
                     build:
                         %s
             %s\
             %s\
             %s
             """
          .formatted(super.generateDockerServices(services), rtiBuildContext, attributes, rtiDependsOn, authService);
    } else {
      return """
             %s\
                 rti:
             %s\
             %s\
             %s
             """
          .formatted(super.generateDockerServices(services), attributes, rtiDependsOn, authService);
    }
  }

  @Override
  protected String getServiceDescription(DockerData data) {
    var isSST = context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    var dependsOn = isSST ? """
                    depends_on:
                        - rti
                        - auth
            """ : """
                    depends_on:
                        - rti
            """;
    return """
           %s\
           %s
           """.formatted(super.getServiceDescription(data), dependsOn);
  }

  @Override
  protected String generateDockerNetwork(String networkName) {
    var isSST = context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    if (isSST) {
      return """
             networks:
                 default: 
                     name: "%s"
                     ipam: 
                         config:
                             - subnet: "%s"
             """.formatted(networkName, "172.21.0.0/16");
    }

    return super.generateDockerNetwork(networkName);
  }

  @Override
  protected String getServiceName(DockerData data) {
    return data.serviceName;
  }

  @Override
  protected String getContainerName(DockerData data) {
    return this.containerName + "-" + data.serviceName;
  }

  @Override
  protected String getDockerFilePath(DockerData data) {
    return "";
  }
}
