package org.lflang.generator.docker;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.KubernetesCredentialPlan;
import org.lflang.federated.generator.KubernetesCredentialPlan.SecretSpec;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;
import org.lflang.util.FileUtil;

/**
 * Generate the Pod and Service manifest for a Kubernetes deployment of a federation.
 *
 * <p>Credentials are not part of the images. Each Pod mounts, read-only, only the Secret(s) of its
 * own component as listed by {@link KubernetesCredentialPlan}, with explicit key-to-path items. The
 * manifest references Secrets by name only; it contains no secret values.
 */
public class FedKubernetesGenerator {
  private final LFGeneratorContext context;
  private final List<FederateInstance> federates;
  private final RtiConfig config;
  private final String rtiNodeHost;
  private final KubernetesCredentialPlan plan;

  public FedKubernetesGenerator(
      LFGeneratorContext context,
      List<FederateInstance> federates,
      RtiConfig config,
      String rtiNodeHost,
      KubernetesCredentialPlan plan) {
    this.context = context;
    this.federates = federates;
    this.config = config;
    this.rtiNodeHost = rtiNodeHost;
    this.plan = plan;
  }

  public void generate() throws IOException {
    var federationName = context.getFileConfig().name;
    var srcGenPath = context.getFileConfig().getSrcGenPath().resolve(federationName + "-pods.yaml");
    FileUtil.writeToFile(generatePodFileContent(), srcGenPath);
  }

  private boolean isSST() {
    return context.getTargetConfig().getOrDefault(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.SST;
  }

  String generatePodFileContent() {
    var federation = context.getFileConfig().name.toLowerCase();
    var registry = context.getTargetConfig().get(DockerProperty.INSTANCE).registryAddress();
    List<String> sections = new ArrayList<>();
    if (isSST()) {
      sections.add(authPod(federation, registry));
      sections.add(service("auth", federation + "-auth", federation, 21900));
    }
    sections.add(rtiPod(federation, registry));
    sections.add(service("rti", federation + "-rti", federation, 15045));
    for (FederateInstance f : federates) {
      sections.add(federatePod(federation, registry, f));
    }
    return String.join("\n---\n", sections) + "\n";
  }

  private String service(String name, String app, String namespace, int port) {
    return String.join(
        "\n",
        "apiVersion: v1",
        "kind: Service",
        "metadata:",
        "  name: " + name,
        "  namespace: " + namespace,
        "spec:",
        "  selector:",
        "    app: " + app,
        "  ports:",
        "    - port: " + port,
        "      targetPort: " + port);
  }

  /** Lines common to all Pods: metadata, restart policy, host network, and node placement. */
  private List<String> podHeader(String name, String namespace, String app, String nodeHost) {
    List<String> l = new ArrayList<>();
    l.add("apiVersion: v1");
    l.add("kind: Pod");
    l.add("metadata:");
    l.add("  name: " + name);
    l.add("  namespace: " + namespace);
    if (app != null) {
      l.add("  labels:");
      l.add("    app: " + app);
    }
    l.add("spec:");
    l.add("  restartPolicy: Never");
    l.add("  hostNetwork: true");
    l.add("  nodeSelector:");
    l.add("    lf-host: \"" + nodeHost + "\"");
    return l;
  }

  private void hostAliases(List<String> l, String... names) {
    l.add("  hostAliases:");
    l.add("    - ip: \"" + rtiNodeHost + "\"");
    l.add("      hostnames:");
    for (String n : names) l.add("        - \"" + n + "\"");
  }

  /** Init container that waits for Auth's port. It mounts no credentials. */
  private void waitForAuth(List<String> l) {
    l.add("  initContainers:");
    l.add("    - name: wait-for-auth");
    l.add("      image: alpine");
    l.add("      command: ['sh', '-c', 'until nc -z auth 21900 2>/dev/null; do sleep 2; done']");
  }

  private void volumeMounts(List<String> l, List<SecretSpec> specs, String indent) {
    if (specs.isEmpty()) return;
    l.add(indent + "volumeMounts:");
    for (SecretSpec s : specs) {
      l.add(indent + "  - name: " + s.name());
      l.add(indent + "    mountPath: " + s.mountPath());
      l.add(indent + "    readOnly: true");
    }
  }

  private void volumes(List<String> l, List<SecretSpec> specs, boolean withAuthState) {
    if (specs.isEmpty() && !withAuthState) return;
    l.add("  volumes:");
    for (SecretSpec s : specs) {
      l.add("    - name: " + s.name());
      if (s.configMap()) {
        l.add("      configMap:");
        l.add("        name: " + s.name());
        l.add("        defaultMode: 0444");
      } else {
        l.add("      secret:");
        l.add("        secretName: " + s.name());
        l.add("        defaultMode: 0400");
      }
      l.add("        optional: false");
      l.add("        items:");
      for (var item : s.items()) {
        l.add("          - key: " + item.key());
        l.add("            path: " + item.mountSubPath());
      }
    }
    if (withAuthState) {
      l.add("    - name: auth-state");
      l.add("      emptyDir: {}");
    }
  }

  private String authPod(String federation, String registry) {
    var specs = plan.secretsFor("auth");
    var main = specs.stream().filter(s -> !s.initOnly()).toList();
    var init = specs.stream().filter(SecretSpec::initOnly).toList();
    String image = "\"" + registry + "/" + federation + "-auth:latest\"";
    String state = KubernetesCredentialPlan.AUTH_STATE_MOUNT;
    String db = "auth" + KubernetesCredentialPlan.AUTH_ID;
    List<String> l = podHeader(federation + "-auth", federation, federation + "-auth", rtiNodeHost);
    // The Auth database is modified at run time (distribution and session keys). Secret volumes
    // are read-only, so the initial database is copied into a Pod-local emptyDir. This state is
    // not preserved across Pod restarts. If the copy fails, the Auth container does not start.
    l.add("  initContainers:");
    l.add("    - name: init-auth-db");
    l.add("      image: " + image);
    l.add("      imagePullPolicy: Always");
    l.add("      command:");
    l.add("        - sh");
    l.add("        - -c");
    l.add(
        "        - set -eu; mkdir -p "
            + state
            + "/"
            + db
            + "; cp "
            + KubernetesCredentialPlan.AUTH_INITDB_MOUNT
            + "/"
            + KubernetesCredentialPlan.AUTH_DB_KEY
            + " "
            + state
            + "/"
            + db
            + "/"
            + KubernetesCredentialPlan.AUTH_DB_KEY
            + "; chmod -R go-rwx "
            + state);
    volumeMounts(l, init, "      ");
    l.add("        - name: auth-state");
    l.add("          mountPath: " + state);
    l.add("  containers:");
    l.add("    - name: auth");
    l.add("      image: " + image);
    l.add("      imagePullPolicy: Always");
    volumeMounts(l, main, "      ");
    l.add("        - name: auth-state");
    l.add("          mountPath: " + state);
    l.add("      readinessProbe:");
    l.add("        tcpSocket:");
    l.add("          port: 21900");
    l.add("        initialDelaySeconds: 3");
    l.add("        periodSeconds: 2");
    volumes(l, specs, true);
    return String.join("\n", l);
  }

  private String rtiPod(String federation, String registry) {
    var specs = plan.secretsFor("rti");
    List<String> l = podHeader(federation + "-rti", federation, federation + "-rti", rtiNodeHost);
    if (isSST()) {
      hostAliases(l, "auth");
      waitForAuth(l);
    }
    l.add("  containers:");
    l.add("    - name: rti");
    l.add("      image: \"" + registry + "/" + federation + "-rti:latest\"");
    l.add("      imagePullPolicy: Always");
    l.add(
        "      args: [\"-i\", \"FEDERATION_ID_PLACEHOLDER\", \"-n\", \""
            + federates.size()
            + "\"]");
    volumeMounts(l, specs, "      ");
    volumes(l, specs, false);
    return String.join("\n", l);
  }

  private String federatePod(String federation, String registry, FederateInstance f) {
    var specs = plan.secretsFor(f.name);
    String component = f.name.replace("__", "-");
    List<String> l = podHeader(federation + "-" + component, federation, null, f.host);
    if (isSST()) {
      hostAliases(l, "rti", "auth");
      waitForAuth(l);
    } else {
      hostAliases(l, "rti");
    }
    l.add("  containers:");
    l.add("    - name: " + component);
    l.add("      image: \"" + registry + "/" + federation + "-" + f.name + ":latest\"");
    l.add("      imagePullPolicy: Always");
    l.add("      args: [\"-r\", \"rti\", \"-i\", \"FEDERATION_ID_PLACEHOLDER\"]");
    volumeMounts(l, specs, "      ");
    volumes(l, specs, false);
    return String.join("\n", l);
  }
}
