package org.lflang.generator;


import java.util.List;

public class FedDockerComposeGenerator extends DockerComposeGenerator {

    /**
     * The host on which to run the rti.
     */
    private String rtiHost;

    /**
     * The name of this federation.
     */
    private String containerName;

    public FedDockerComposeGenerator(LFGeneratorContext context, String rtiHost) {
        super(context);
        this.rtiHost = rtiHost;
        this.containerName = context.getFileConfig().name;
    }

    /**
     * Override in FedGenerator
     * @param services
     *
     * @return
     */
    @Override
    protected String generateDockerServices(List<DockerData> services) {
        return String.join("\n",
            super.generateDockerServices(services),
            "    rti:",
            "        image: lflang/rti:rti",
            "        hostname: " + this.rtiHost,
            "        command: -i 1 -n " + services.size(),
            "        container_name: " + containerName + "-" + "rti");
    }

    /**
     * Turn given docker data into a string usable in a federated docker compose file.
     */
    @Override
    protected String getServiceDescription(DockerData data) {
        var svc = new StringBuilder(super.getServiceDescription(data));
        svc.append("\n        command: -i 1");
        svc.append("\n        depends_on:");
        svc.append("\n            - rti");
        return svc.toString();
    }
}
