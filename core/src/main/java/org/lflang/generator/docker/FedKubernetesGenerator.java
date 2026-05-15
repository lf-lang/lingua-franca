package org.lflang.generator.docker;

import org.lflang.generator.LFGeneratorContext;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
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
        return String.join(
            "\n---\n",
            generateRtiPod(federationName.toLowerCase(), rtiHost, registryAddress, federateCount),
            generateRtiService(federationName.toLowerCase()),
            generateFederatesPod(federationName.toLowerCase(), registryAddress)
            
        );
    }


    private String generateRtiPod(String federation, String host, String registryAddress, int federateCount) {
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
                            - "rti"
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
                    entityName.replace("__", "-"),
                    registryAddress,
                    federation,
                    entityName
                );
    }


}