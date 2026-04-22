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

    public FedKubernetesGenerator(LFGeneratorContext context, List<FederateInstance> federates, RtiConfig config) {
        this.context = context;
        this.federates = federates;
        this.config = config;
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
        return String.join(
            "\n---\n",
            generateRtiPod(federationName, rtiHost, registryAddress),
            generateFederatesPod(federationName, registryAddress)
            
        );
    }


    private String generateRtiPod(String federation, String host, String registryAddress) {
        return """ 
                 apiVersion: v1
                 kind: Pod
                 metadata:
                    name: %s-rti
                    namespace: %s
                 spec:
                    hostNetwork:  true
                    restartPolicy: Never
                    nodeSelector:
                        lf-host: "%s"
                    containers:
                        - name: rti
                          image: "%s/%s-rti:latest"
                          imagePullPolicy: Always
        
            """.formatted(
                    federation,
                    federation,
                    host,
                    registryAddress,
                    federation
                );
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
                    hostNetwork:  true
                    restartPolicy: Never
                    nodeSelector:
                        lf-host: "%s"
                    containers:
                        - name: %s
                          image: "%s/%s-%s:latest"
                          imagePullPolicy: Always
        
            """.formatted(
                    federation,
                    entityName,
                    federation,
                    host,
                    entityName,
                    registryAddress,
                    federation,
                    entityName
                );
    }


}