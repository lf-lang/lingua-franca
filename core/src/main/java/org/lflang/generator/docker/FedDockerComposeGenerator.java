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
    var deploymentType = context.getTargetConfig().get(DockerProperty.INSTANCE).deployment();
    var registryAddress = context.getTargetConfig().get(DockerProperty.INSTANCE).registryAddress();
    var isSST =
        context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;

    var rtiImg =
        deploymentType.equals("kubernetes")
            ? String.format("%s/%s-rti:latest", registryAddress, this.containerName.toLowerCase())
            : this.rtiImage;
    var authImg =
        (deploymentType.equals("kubernetes") && isSST)
            ? String.format("%s/%s-auth:latest", registryAddress, this.containerName.toLowerCase())
            : "";

    var attributes =
        """
                image: "%s"
                hostname: "rti"
                command: "-i 1 %s -n %s"
                container_name: "%s-rti"
        """
            .formatted(rtiImg, tracing, services.size(), containerName.toLowerCase());

    var authIP = context.getTargetConfig().get(DockerProperty.INSTANCE).authIP();
    var authService =
        isSST
            ? deploymentType.equals("compose")
                ? """
                      auth:
                          networks:
                              default:
                                  ipv4_address: "%s"
                          build:
                              context: "auth"
                          container_name: "%s-auth"
                          ports:
                              - "21900:21900"
                          healthcheck:
                              test: ["CMD", "bash", "-c", "echo > /dev/tcp/localhost/21900"]
                              interval: 2s
                              retries: 15
                  """
                    .formatted(authIP, containerName)
                : """
                      auth:
                          build:
                              context: "auth"
                          image: "%s"
                          container_name: "%s-auth"
                  """
                    .formatted(authImg, containerName.toLowerCase())
            : "";

    var rtiDependsOn =
        (isSST && deploymentType.equals("compose"))
            ? """
                      depends_on:
                          - auth
              """
            : "";
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
          .formatted(
              super.generateDockerServices(services),
              rtiBuildContext,
              attributes,
              rtiDependsOn,
              authService);
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
    var isSST =
        context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    var deploymentType = context.getTargetConfig().get(DockerProperty.INSTANCE).deployment();

    if (deploymentType.equals("kubernetes")) {
      var registryAddr = context.getTargetConfig().get(DockerProperty.INSTANCE).registryAddress();

      return """
                 %s:
                     build:
                         context: "%s"
                         %s
                     image: "%s/%s:latest"
                     container_name: "%s"
                     tty: true
                     extra_hosts:
                       - "host.docker.internal:host-gateway"
                     environment:
                       - "LF_TELEGRAF_HOST_NAME=${LF_TELEGRAF_HOST_NAME:-host.docker.internal}"
             """
          .formatted(
              getServiceName(data),
              getBuildContext(data),
              getDockerFilePath(data),
              registryAddr,
              getContainerName(data).toLowerCase(),
              getContainerName(data).toLowerCase());
    }
    var dependsOn =
        (isSST && deploymentType.equals("compose"))
            ? """
                      depends_on:
                          - rti
                          - auth
              """
            : """
                      depends_on:
                          - rti
              """;
    return """
           %s\
           %s
           """
        .formatted(super.getServiceDescription(data), dependsOn);
  }

  @Override
  protected String generateDockerNetwork(String networkName) {
    var isSST =
        context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
    var subnet = context.getTargetConfig().get(DockerProperty.INSTANCE).subnet();
    var deploymentType = context.getTargetConfig().get(DockerProperty.INSTANCE).deployment();
    if (isSST && deploymentType.equals("compose")) {
      return """
             networks:
                 default:
                     name: "%s"
                     ipam:
                         config:
                             - subnet: "%s"
             """
          .formatted(networkName, subnet);
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
