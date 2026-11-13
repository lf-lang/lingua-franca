package org.lflang.generator.docker;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.attribute.PosixFilePermission;
import java.util.Set;
import org.lflang.federated.generator.KubernetesCredentialPlan;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.util.FileUtil;

public class FedDeploymentScriptGenerator {

  private final LFGeneratorContext context;

  /** Credential plan for Kubernetes deployments; null for Docker Compose. */
  private final KubernetesCredentialPlan plan;

  public FedDeploymentScriptGenerator(LFGeneratorContext context) {
    this(context, null);
  }

  public FedDeploymentScriptGenerator(LFGeneratorContext context, KubernetesCredentialPlan plan) {
    this.context = context;
    this.plan = plan;
  }

  public void generate() throws IOException {
    var federationName = context.getFileConfig().name;
    var srcGenPath = context.getFileConfig().getSrcGenPath().toAbsolutePath();
    var deploymentType = context.getTargetConfig().get(DockerProperty.INSTANCE).deployment();

    var scriptContent =
        deploymentType.equals("kubernetes") && plan != null
            ? generateKubernetesLaunchScript(srcGenPath.toString(), federationName)
            : generateDeploymentLaunchScript(srcGenPath.toString(), deploymentType, federationName);
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

  /** Single-quote a string for the POSIX shell. */
  private static String q(String s) {
    return "'" + s.replace("'", "'\"'\"'") + "'";
  }

  /**
   * Launcher for Kubernetes deployments with run-time credential injection.
   *
   * <p>Order: pre-flight check of every credential file -> push images -> create the namespace
   * (refusing to reuse an existing one) -> create one Secret per component (and the Auth ConfigMap)
   * from files -> apply Pods/Services -> readiness and log streaming. Secret values never appear on
   * a command line, in a log, or in a stored manifest: kubectl reads them from the staged files.
   * The EXIT trap deletes only what this run created.
   */
  private String generateKubernetesLaunchScript(String srcGenPath, String federationName) {
    var ns = federationName.toLowerCase();
    var sb = new StringBuilder();
    sb.append(
        """
        #!/bin/bash
        # Do not enable shell tracing (set -x): commands below handle credential file paths.
        set -uo pipefail
        BUILD=false
        for arg in "$@"; do
            case "$arg" in
                -b|--build)
                    BUILD=true
                    ;;
            esac
        done
        export FEDERATION_ID=$(openssl rand -hex 24)
        cd %s || exit 1
        NAMESPACE=%s
        NS_CREATED=false
        umask 077
        LF_TMP_DIR=$(mktemp -d) || exit 1
        PODS_FILE="$LF_TMP_DIR/pods.yaml"
        cleanup() {
            rm -rf "$LF_TMP_DIR"
            # Delete the namespace only if this run created it (never a pre-existing one).
            if [ "$NS_CREATED" = true ]; then
                kubectl delete namespace "$NAMESPACE" --ignore-not-found
            fi
            # Remove only this federation's local images (exact names from docker-compose.yml).
            for image in $(docker compose -f docker-compose.yml config --images 2>/dev/null); do
                docker rmi "$image" >/dev/null 2>&1 || true
            done
        }
        trap cleanup EXIT
        fail() {
            echo "ERROR: $*" >&2
            exit 1
        }
        command -v docker &> /dev/null || fail "Docker is not installed."
        command -v kubectl &> /dev/null || fail "kubectl is not installed."
        [ -f docker-compose.yml ] || fail "docker-compose.yml not found."
        [ -f %s-pods.yaml ] || fail "%s-pods.yaml not found."
        """
            .formatted(q(srcGenPath), q(ns), federationName, federationName));

    // 1) Pre-flight: every planned file must exist and be non-empty.
    sb.append("# Pre-flight check of the credential files (paths only; values are never read).\n");
    for (var s : plan.secrets()) {
      for (var item : s.items()) {
        var path = plan.stagedPath(s, item).toAbsolutePath().toString();
        sb.append("[ -s ")
            .append(q(path))
            .append(" ] || fail ")
            .append(q("missing credential file for component " + s.component() + ": " + path))
            .append("\n");
      }
    }

    // 2) Images.
    sb.append(
        """
        if [ "$BUILD" = true ]; then
            docker compose -f docker-compose.yml build || fail "image build failed."
        fi
        docker compose -f docker-compose.yml push || fail "image push failed."
        # Wait for an old namespace of this federation to finish terminating, then create it.
        kubectl wait --for=delete namespace/"$NAMESPACE" --timeout=120s 2>/dev/null || true
        if kubectl get namespace "$NAMESPACE" > /dev/null 2>&1; then
            fail "namespace $NAMESPACE already exists; not reusing or deleting it."
        fi
        kubectl create namespace "$NAMESPACE" || fail "could not create namespace $NAMESPACE."
        NS_CREATED=true
        echo "Federation ID: ${FEDERATION_ID}"
        """);

    // 3) Secrets / ConfigMap, one per component, created from files.
    for (var s : plan.secrets()) {
      sb.append("kubectl create ")
          .append(s.configMap() ? "configmap " : "secret generic ")
          .append(s.name())
          .append(" -n \"$NAMESPACE\"");
      for (var item : s.items()) {
        sb.append(" --from-file=")
            .append(q(item.key() + "=" + plan.stagedPath(s, item).toAbsolutePath()));
      }
      sb.append(" > /dev/null || fail ")
          .append(q("could not create " + s.name() + " for component " + s.component() + "."))
          .append("\n");
    }

    // 4) Pods and Services; 5) readiness and logs (unchanged behavior).
    sb.append(
        """
        sed -e "s/FEDERATION_ID_PLACEHOLDER/${FEDERATION_ID}/g" %s-pods.yaml > "$PODS_FILE" \\
            || fail "could not prepare the Pod manifest."
        kubectl apply -f "$PODS_FILE" || fail "kubectl apply failed."
        # Wait for all pods in the namespace to become ready.
        echo "Waiting for pods to start..."
        kubectl wait --for=condition=Ready pods --all -n "$NAMESPACE" --timeout=60s
        # Stream logs from all running pods (except the auth pod) in the background.
        for pod in $(kubectl get pods -n "$NAMESPACE" -o name | grep -v auth); do
            kubectl logs -f -n "$NAMESPACE" $pod &
        done
        # Wait for all background log streaming processes to finish (exits when pods terminate).
        wait
        """
            .formatted(federationName));
    return sb.toString();
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
