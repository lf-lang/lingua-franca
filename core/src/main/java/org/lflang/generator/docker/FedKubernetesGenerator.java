package org.lflang.generator.docker;

import org.lflang.generator.LFGeneratorContext;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;
import java.util.ArrayList;
import org.lflang.target.property.DockerProperty;
import org.lflang.util.FileUtil;
import java.io.IOException;
import java.util.List;


public class FedKubernetesGenerator {
    private final LFGeneratorContext context;
    private final List<FederateInstance> federates;
    private final RtiConfig config;
    private final String rtiNodeHost;

    public FedKubernetesGenerator(LFGeneratorContext context, List<FederateInstance> federates, RtiConfig config, String rtiNodeHost) {
        this.context = context;
        this.federates = federates;
        this.config = config;
        this.rtiNodeHost = rtiNodeHost;
    }
    

    public void generate() throws IOException {
        var federationName = context.getFileConfig().name;
        var srcGenPath = context.getFileConfig().getSrcGenPath().resolve(federationName + "-pods.yaml");
        var podsFileContent = generatePodFileContent();
        FileUtil.writeToFile(podsFileContent, srcGenPath);

    }

    private String generatePodFileContent() {
        var federationName = context.getFileConfig().name;
        var registryAddress = context.getTargetConfig().get(DockerProperty.INSTANCE).registryAddress();
        var rtiHost = this.config.getHost();
        var federateCount = this.federates.size();
        
        List<String> sections = new ArrayList<>();

        if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST) {
            sections.add(generateAuthPod(federationName.toLowerCase(), registryAddress));
            sections.add(generateAuthService(federationName.toLowerCase()));
        }

        sections.add(generateRtiPod(federationName.toLowerCase(), rtiHost, registryAddress, federateCount));
        sections.add(generateRtiService(federationName.toLowerCase()));
        sections.add(generateFederatesPod(federationName.toLowerCase(), registryAddress));

        return String.join("\n---\n", sections);
    }

    private String generateAuthPod(String federation, String registryAddress) {
        return """
                 apiVersion: v1
                 kind: Pod
                 metadata:
                    name: %s-auth
                    namespace: %s
                    labels:
                        app: %s-auth
                 spec:
                    restartPolicy: Never
                    hostNetwork: true
                    nodeSelector: 
                        lf-host: "%s"
                    containers:
                        - name: auth
                          image: "%s/%s-auth:latest"
                          imagePullPolicy: Always
                          readinessProbe:
                              tcpSocket:
                                  port: 21900
                              initialDelaySeconds: 3
                              periodSeconds: 2
            """.formatted(
                    federation,
                    federation,
                    federation,
                    rtiNodeHost,
                    registryAddress,
                    federation
            );
    }

    private String generateAuthService(String federation) {
        return """
                 apiVersion: v1
                 kind: Service
                 metadata:
                    name: auth
                    namespace: %s
                 spec:
                    selector: 
                        app: %s-auth
                    ports:
                        - port: 21900
                          targetPort: 21900
            """.formatted(federation, federation);
    }

    private String generateRtiPod(String federation, String host, String registryAddress, int federateCount) {
        var isSST = context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
        var rtiHostAliases = isSST ? """
                hostAliases:
                            - ip: %s
                              hostnames:
                                - "auth"
                """.formatted(rtiNodeHost) : "";
        var rtiInitContainers = isSST ? """
                initContainers:
                            - name: wait-for-auth
                              image: alpine
                              command: ['sh', '-c', 'until nc -z auth 21900 2>/dev/null; do sleep 2; done']
                """ : "";

        return """ 
                 apiVersion: v1
                 kind: Pod
                 metadata:
                    name: %s-rti
                    namespace: %s
                    labels:
                        app: %s-rti
                 spec:
                    restartPolicy: Never
                    hostNetwork: true
                    nodeSelector:
                        lf-host: "%s"
                    %s
                    %s
                    containers:
                        - name: rti
                          image: "%s/%s-rti:latest"
                          imagePullPolicy: Always
                          args: ["-i", "1", "-n", "%d"]

            """.formatted(
                    federation,
                    federation,
                    federation,
                    rtiNodeHost,
                    rtiHostAliases,
                    rtiInitContainers,
                    registryAddress,
                    federation,
                    federateCount
                );
    }

    private String generateRtiService(String federation) {
        return """
                 apiVersion: v1
                 kind: Service
                 metadata:
                    name: rti
                    namespace: %s
                 spec:
                    selector: 
                        app: %s-rti
                    ports:
                        - port: 15045
                          targetPort: 15045
            """.formatted(federation, federation);
    }

    private String generateFederatesPod(String federation, String registryAddress) {
        List<String> pods = new ArrayList<>();
        for (FederateInstance federate : this.federates) {
            pods.add(generatePerEntityContent(federation, federate.host, registryAddress, federate.name));
        }

        return String.join(
            "\n---\n",
            pods
        );
    }

    private String generatePerEntityContent(String federation, String host, String registryAddress, String entityName) {
        var isSST = context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST;
        var entityInitContainers = isSST ? """
                initContainers:
                            - name: wait-for-auth
                              image: alpine
                              command: ['sh', '-c', 'until nc -z auth 21900 2>/dev/null; do sleep 2; done']
                """ : "";

        return """ 
                 apiVersion: v1
                 kind: Pod
                 metadata:
                    name: %s-%s
                    namespace: %s
                 spec:
                    restartPolicy: Never
                    hostNetwork: true
                    nodeSelector:
                        lf-host: "%s"
                    hostAliases: 
                        - ip: %s
                          hostnames:
                            %s
                    %s
                    containers:
                        - name: %s
                          image: "%s/%s-%s:latest"
                          imagePullPolicy: Always
                          args: ["-r", "rti"]
        
            """.formatted(
                    federation,
                    entityName.replace("__", "-"),
                    federation,
                    host,
                    rtiNodeHost,
                    getHostNames(),
                    entityInitContainers,
                    entityName.replace("__", "-"),
                    registryAddress,
                    federation,
                    entityName
                );
    }

    private String getHostNames() {
        if (context.getTargetConfig().get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST){
            return "- \"rti\"\n                - \"auth\"";
        }

        return "- \"rti\"";
    }


}