package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.attribute.PosixFilePermission;
import java.util.Set;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.util.FileUtil;

public class FedDeploymentScriptGenerator {

  private final LFGeneratorContext context;

  public FedDeploymentScriptGenerator(LFGeneratorContext context) {
    this.context = context;
  }

  public void generate() throws IOException {
    var federationName = context.getFileConfig().name;
    var srcGenPath = context.getFileConfig().getSrcGenPath().toAbsolutePath();
    var deploymentType = context.getTargetConfig().get(DockerProperty.INSTANCE).deployment();

    var scriptContent =
        generateDeploymentLaunchScript(srcGenPath.toString(), deploymentType, federationName);
    var scriptPath = context.getFileConfig().binPath.resolve(federationName + "_deploy");

    FileUtil.writeToFile(scriptContent, scriptPath);
    Files.setPosixFilePermissions(
        scriptPath,
        Set.of(
            PosixFilePermission.OWNER_READ,
            PosixFilePermission.OWNER_WRITE,
            PosixFilePermission.OWNER_EXECUTE,
            PosixFilePermission.GROUP_READ,
            PosixFilePermission.GROUP_EXECUTE,
            PosixFilePermission.OTHERS_READ,
            PosixFilePermission.OTHERS_EXECUTE));
  }

  private String generateDeploymentLaunchScript(
      String srcGenPath, String deploymentType, String federationName) {
    var dockerImageCleanup =
        """
docker rmi $(docker images --format "{{.ID}} {{.Repository}}" | grep %s | awk '{print $1}') 2>/dev/null || true
"""
            .formatted(federationName.toLowerCase());

    var cleanup =
        deploymentType.equals("kubernetes")
            ? """
              cleanup() {
                  kubectl delete -f %s-pods.yaml --ignore-not-found
                  kubectl delete namespace %s --ignore-not-found
                  %s
              }
              trap cleanup EXIT
              """
                .formatted(federationName, federationName.toLowerCase(), dockerImageCleanup)
            : """
              cleanup() {
                  docker compose down
                  %s
              }
              trap cleanup EXIT
              """
                .formatted(dockerImageCleanup);

    var header =
        """
        #!/bin/bash
        cd %s
        """
                .formatted(srcGenPath)
            + cleanup;

    var dockerCheck =
        """
        if ! command -v docker &> /dev/null; then
            echo "Docker is not installed."
            exit 1
        fi
        """;

    var composeFileCheck =
        """
        if [ ! -f docker-compose.yml ]; then
            echo "docker-compose.yml not found."
            exit 1
        fi
        """;

    var podFileCheck =
        """
        if [ ! -f %s-pods.yaml ]; then
            echo "%s-pods.yaml not found."
            exit 1
        fi
        """
            .formatted(federationName, federationName);

    var kubectlCheck =
        """
        if ! command -v kubectl &> /dev/null; then
            echo "kubectl is not installed."
            exit 1
        fi
        """;

    var logStreaming =
        """
        echo "Waiting for pods to start..."
        kubectl wait --for=condition=Ready pods --all -n %s --timeout=60s
        for pod in $(kubectl get pods -n %s -o name | grep -v auth); do
            kubectl logs -f -n %s $pod &
        done
        wait
        """
            .formatted(
                federationName.toLowerCase(),
                federationName.toLowerCase(),
                federationName.toLowerCase());

    var deploySteps =
        deploymentType.equals("kubernetes")
            ? kubectlCheck
                + podFileCheck
                + """
                  docker compose build
                  docker compose push
                  kubectl wait --for=delete namespace/%s --timeout=120s 2>/dev/null || true
                  kubectl create namespace %s
                  kubectl delete -f %s-pods.yaml --ignore-not-found
                  kubectl apply -f %s-pods.yaml
                  """
                    .formatted(
                        federationName.toLowerCase(),
                        federationName.toLowerCase(),
                        federationName,
                        federationName)
                + logStreaming
            : """
              docker compose build
              docker compose up
              """;

    return header + dockerCheck + composeFileCheck + deploySteps;
  }
}
