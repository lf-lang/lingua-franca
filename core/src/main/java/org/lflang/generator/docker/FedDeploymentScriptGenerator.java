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
    var scriptPath = context.getFileConfig().binPath.resolve(federationName);

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
    var hasOverride =
        Files.exists(
            context.getFileConfig().getSrcGenPath().resolve("docker-compose-override.yml"));
    var overrideArg =
        hasOverride
            ? "-f docker-compose.yml -f docker-compose-override.yml"
            : "-f docker-compose.yml";

    var dockerImageCleanup =
        """
docker rmi $(docker images --format "{{.ID}} {{.Repository}}" | grep %s | awk '{print $1}') 2>/dev/null || true
"""
            .formatted(federationName.toLowerCase());

    var cleanup =
        deploymentType.equals("kubernetes")
            ? """
              cleanup() {
                  # Remove the temporary pod configuration file.
                  rm -f %s-pods-temp.yaml
                  # Delete the Kubernetes namespace to remove all deployed pods and services.
                  kubectl delete namespace %s --ignore-not-found
                  # Clean up local docker images created for this federation.
                  %s
              }
              trap cleanup EXIT
              """
                .formatted(federationName, federationName.toLowerCase(), dockerImageCleanup)
            : """
              cleanup() {
                  # Tear down docker compose services.
                  docker compose %s down
                  # Clean up local docker images.
                  %s
              }
              trap cleanup EXIT
              """
                .formatted(overrideArg, dockerImageCleanup);

    var header =
        """
        #!/bin/bash
        BUILD=false
        for arg in "$@"; do
            case "$arg" in
                -b|--build)
                    BUILD=true
                    ;;
            esac
        done
        export FEDERATION_ID=$(openssl rand -hex 24)
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
        # Wait for all pods in the namespace to become ready.
        echo "Waiting for pods to start..."
        kubectl wait --for=condition=Ready pods --all -n %s --timeout=60s
        # Stream logs from all running pods (except the auth pod) in the background.
        for pod in $(kubectl get pods -n %s -o name | grep -v auth); do
            kubectl logs -f -n %s $pod &
        done
        # Wait for all background log streaming processes to finish (exits when pods terminate).
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
# Build docker images using docker-compose.yml if requested.
if [ "$BUILD" = true ]; then
    docker compose %s build
fi
# Push the built docker images to the local registry.
docker compose %s push
# Wait for the old namespace to be fully deleted if it is still terminating.
kubectl wait --for=delete namespace/%s --timeout=120s 2>/dev/null || true
# Create a fresh namespace for this deployment.
kubectl create namespace %s
echo "Federation ID: ${FEDERATION_ID}"
# Replace the placeholder in the pods YAML with the generated ID.
# A temporary file is used to avoid overwriting the original template.
sed -e "s/FEDERATION_ID_PLACEHOLDER/${FEDERATION_ID}/g" %s-pods.yaml > %s-pods-temp.yaml
# Apply the temporary YAML file with the concrete federation ID.
kubectl apply -f %s-pods-temp.yaml
"""
                    .formatted(
                        overrideArg,
                        overrideArg,
                        federationName.toLowerCase(),
                        federationName.toLowerCase(),
                        federationName,
                        federationName,
                        federationName)
                + logStreaming
            : """
              # Build docker compose services if requested.
              if [ "$BUILD" = true ]; then
                  docker compose %s build
              fi
              # Start docker compose services.
              docker compose %s up --abort-on-container-failure
              """
                .formatted(overrideArg, overrideArg);

    return header + dockerCheck + composeFileCheck + deploySteps;
  }
}
