package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermission;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.SSTProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/**
 * Per-component credential plan for Kubernetes deployments.
 *
 * <p>For each component (RTI, every federate, and Auth for SST) this class decides which generated
 * files are provided to the Pod, under which Kubernetes Secret key, and at which path inside the
 * container. The same plan drives (1) staging of the files into a provisioning directory that is
 * outside every Docker build context, (2) the Secret volumes and mounts in the Pod manifest, and
 * (3) the Secret creation commands and pre-flight checks in the launcher. It contains file paths
 * only, never secret values.
 *
 * <p>Container paths are chosen so that the existing runtime arguments stay unchanged: TLS uses
 * {@code credentials/<entity>.crt|.key} (and {@code ca.crt} next to them for peer verification),
 * and SST uses {@code ./sst/<entity>.config}, whose credential paths are relative to the working
 * directory {@code /lingua-franca}.
 */
public final class KubernetesCredentialPlan {

  /** One file provided to a container: Secret key, staged source file, path inside the mount. */
  public record Item(String key, Path source, String mountSubPath) {}

  /**
   * One Secret (or ConfigMap, for non-secret configuration).
   *
   * @param name Kubernetes object name.
   * @param component The component (entity) the object belongs to.
   * @param mountPath Directory inside the container where the items are mounted read-only.
   * @param configMap True for a ConfigMap (non-secret configuration), false for a Secret.
   * @param initOnly True if only the init container mounts it.
   */
  public record SecretSpec(
      String name,
      String component,
      String mountPath,
      boolean configMap,
      boolean initOnly,
      List<Item> items) {}

  public static final String TLS_MOUNT = "/lingua-franca/credentials";
  public static final String SST_MOUNT = "/lingua-franca/sst";
  public static final String AUTH_SECRET_MOUNT = "/auth/secret";
  public static final String AUTH_INITDB_MOUNT = "/auth/initdb";
  public static final String AUTH_CONFIG_MOUNT = "/auth/config";
  public static final String AUTH_STATE_MOUNT = "/auth/state";
  public static final String AUTH_PROPERTIES_KEY = "auth.properties";
  public static final String AUTH_PASSWORD_KEY = "password-input";
  public static final String AUTH_DB_KEY = "auth.db";

  /** The Auth instance the generated graph assigns every entity to. */
  public static final int AUTH_ID = 101;

  private static final Pattern SECRET_KEY = Pattern.compile("[-._a-zA-Z0-9]+");

  /** Program name and fed-gen/<name> directory; paths follow FederationFileConfig. */
  private final String name;

  private final Path genPath;
  private final CommunicationMode mode;
  private final boolean permanentDistKey;
  private final List<String> entities;
  private final List<SecretSpec> secrets = new ArrayList<>();

  private KubernetesCredentialPlan(
      String name,
      Path genPath,
      CommunicationMode mode,
      boolean permanentDistKey,
      List<String> entities) {
    this.name = name;
    this.genPath = genPath;
    this.mode = mode;
    this.permanentDistKey = permanentDistKey;
    this.entities = entities;
    build();
  }

  /** Return true if the target configuration selects Docker with Kubernetes deployment. */
  public static boolean isKubernetes(TargetConfig config) {
    var docker = config.get(DockerProperty.INSTANCE);
    return docker.enabled() && "kubernetes".equals(docker.deployment());
  }

  /** Create the plan for a federation. The RTI is always the entity named "rti". */
  public static KubernetesCredentialPlan create(
      FederationFileConfig fileConfig, TargetConfig config, List<FederateInstance> federates) {
    List<String> entities = new ArrayList<>();
    entities.add("rti");
    for (FederateInstance f : federates) entities.add(f.name);
    return new KubernetesCredentialPlan(
        fileConfig.name,
        fileConfig.getGenPath(),
        config.getOrDefault(CommunicationModeProperty.INSTANCE),
        config.get(SSTProperty.INSTANCE).usePermanentDistKey(),
        entities);
  }

  /**
   * Create a plan from explicit values (used by tests). {@code genPath} is the federation's
   * {@code fed-gen/<name>} directory.
   */
  public static KubernetesCredentialPlan create(
      String name,
      Path genPath,
      CommunicationMode mode,
      boolean permanentDistKey,
      List<String> entities) {
    return new KubernetesCredentialPlan(name, genPath, mode, permanentDistKey, entities);
  }

  /** Directory (outside every Docker build context) that holds the staged files. */
  public static Path provisioningRoot(FederationFileConfig fileConfig) {
    return provisioningRoot(fileConfig.getGenPath());
  }

  private static Path provisioningRoot(Path genPath) {
    return genPath.resolve("k8s-credentials");
  }

  /** DNS-1123 compliant name for a component, matching the Pod names of FedKubernetesGenerator. */
  public static String componentName(String federation, String entity) {
    return (federation + "-" + entity.replace("__", "-"))
        .toLowerCase()
        .replaceAll("[^a-z0-9-]", "-");
  }

  public List<SecretSpec> secrets() {
    return secrets;
  }

  /** The Secret/ConfigMap objects mounted by the given component (empty for TCP). */
  public List<SecretSpec> secretsFor(String component) {
    return secrets.stream().filter(s -> s.component().equals(component)).toList();
  }

  public boolean isEmpty() {
    return secrets.isEmpty();
  }

  /** Path of the staged copy of an item (what kubectl reads). */
  public Path stagedPath(SecretSpec secret, Item item) {
    return provisioningRoot(genPath).resolve(secret.name()).resolve(item.key());
  }

  private void build() {
    String fed = name;
    // Same layout as FederationFileConfig/TLSGenerator: fed-gen/<name>/{credentials,sst}.
    Path sst = genPath.resolve("sst");
    if (mode == CommunicationMode.TLS) {
      for (String e : entities) {
        Path dir = genPath.resolve("credentials").resolve(e);
        add(
            new SecretSpec(
                componentName(fed, e) + "-tls",
                e,
                TLS_MOUNT,
                false,
                false,
                List.of(
                    new Item(e + ".crt", dir.resolve(e + ".crt"), e + ".crt"),
                    new Item(e + ".key", dir.resolve(e + ".key"), e + ".key"),
                    new Item("ca.crt", dir.resolve("ca.crt"), "ca.crt"))));
      }
    } else if (mode == CommunicationMode.SST) {
      Path creds = sst.resolve("credentials");
      Path keys = creds.resolve("keys").resolve("net1");
      String authCert = "Auth" + AUTH_ID + "EntityCert.pem";
      for (String e : entities) {
        List<Item> items = new ArrayList<>();
        items.add(
            new Item(e + ".config", sst.resolve("configs").resolve(e + ".config"), e + ".config"));
        if (permanentDistKey) {
          for (String suffix : List.of("CipherKey.key", "MacKey.key")) {
            String file = "Net1." + e + suffix;
            items.add(new Item(file, keys.resolve(file), "credentials/keys/net1/" + file));
          }
        } else {
          items.add(
              new Item(
                  authCert,
                  creds.resolve("auth_certs").resolve(authCert),
                  "credentials/auth_certs/" + authCert));
          String file = "Net1." + e + "Key.pem";
          items.add(new Item(file, keys.resolve(file), "credentials/keys/net1/" + file));
        }
        add(new SecretSpec(componentName(fed, e) + "-sst", e, SST_MOUNT, false, false, items));
      }
      // Auth: only the files of the Auth instance the entities are assigned to. The CA private
      // key, the other Auth's material and the original properties are not provided.
      Path auth = sst.resolve("auth");
      Path db = auth.resolve("databases").resolve("auth" + AUTH_ID);
      Path ks = db.resolve("my_keystores");
      String a = "Auth" + AUTH_ID;
      String authName = componentName(fed, "auth");
      add(
          new SecretSpec(
              authName + "-keys",
              "auth",
              AUTH_SECRET_MOUNT,
              false,
              false,
              List.of(
                  new Item(a + "Entity.pfx", ks.resolve(a + "Entity.pfx"), a + "Entity.pfx"),
                  new Item(a + "Internet.pfx", ks.resolve(a + "Internet.pfx"), a + "Internet.pfx"),
                  new Item(a + "Database.pfx", ks.resolve(a + "Database.pfx"), a + "Database.pfx"),
                  new Item(a + "Database.bin", ks.resolve(a + "Database.bin"), a + "Database.bin"),
                  new Item(
                      "CACert.pem",
                      auth.resolve("credentials").resolve("ca").resolve("CACert.pem"),
                      "CACert.pem"),
                  new Item(
                      AUTH_PASSWORD_KEY,
                      provisioningRoot(genPath)
                          .resolve(authName + "-keys")
                          .resolve(AUTH_PASSWORD_KEY),
                      AUTH_PASSWORD_KEY))));
      add(
          new SecretSpec(
              authName + "-initdb",
              "auth",
              AUTH_INITDB_MOUNT,
              false,
              true,
              List.of(new Item(AUTH_DB_KEY, db.resolve(AUTH_DB_KEY), AUTH_DB_KEY))));
      add(
          new SecretSpec(
              authName + "-config",
              "auth",
              AUTH_CONFIG_MOUNT,
              true,
              false,
              List.of(
                  new Item(
                      AUTH_PROPERTIES_KEY,
                      provisioningRoot(genPath)
                          .resolve(authName + "-config")
                          .resolve(AUTH_PROPERTIES_KEY),
                      AUTH_PROPERTIES_KEY))));
    }
  }

  private void add(SecretSpec spec) {
    for (Item item : spec.items()) {
      if (!SECRET_KEY.matcher(item.key()).matches()) {
        throw new IllegalArgumentException("Invalid Kubernetes Secret key: " + item.key());
      }
    }
    secrets.add(spec);
  }

  /**
   * Stage every planned file into the provisioning directory with owner-only permissions. Generated
   * inputs (Auth properties and password input) are written by the caller before staging. Throws
   * if any source file is missing; the message names the component and the path only.
   */
  public void stage() throws IOException {
    Path root = provisioningRoot(genPath);
    createPrivateDirectory(root);
    for (SecretSpec s : secrets) {
      Path dir = root.resolve(s.name());
      createPrivateDirectory(dir);
      for (Item item : s.items()) {
        Path dst = dir.resolve(item.key());
        if (item.source().equals(dst)) {
          if (!Files.isRegularFile(dst)) {
            throw new IOException(
                "Missing credential file for component '" + s.component() + "': " + dst);
          }
        } else {
          if (!Files.isRegularFile(item.source())) {
            throw new IOException(
                "Missing credential file for component '" + s.component() + "': " + item.source());
          }
          Files.copy(item.source(), dst, StandardCopyOption.REPLACE_EXISTING);
        }
        restrictFile(dst);
      }
    }
  }

  /** Write a generated file (e.g. Auth properties) to its staged location with mode 0600. */
  public static void writePrivateFile(Path file, String content) throws IOException {
    createPrivateDirectory(file.getParent());
    Files.deleteIfExists(file);
    if (posix()) {
      Files.createFile(
          file, PosixFilePermissions.asFileAttribute(PosixFilePermissions.fromString("rw-------")));
    }
    Files.writeString(file, content);
    restrictFile(file);
  }

  private static boolean posix() {
    return FileSystems.getDefault().supportedFileAttributeViews().contains("posix");
  }

  /** Create a directory (and parents) and make it owner-only (0700). */
  public static void createPrivateDirectory(Path dir) throws IOException {
    Files.createDirectories(dir);
    if (posix()) Files.setPosixFilePermissions(dir, PosixFilePermissions.fromString("rwx------"));
  }

  private static void restrictFile(Path file) throws IOException {
    if (posix()) Files.setPosixFilePermissions(file, PosixFilePermissions.fromString("rw-------"));
  }

  /** Recursively restrict a credential tree: directories 0700, regular files 0600. */
  public static void restrictTree(Path root) throws IOException {
    if (!posix() || !Files.exists(root)) return;
    try (Stream<Path> paths = Files.walk(root)) {
      for (Path p : paths.toList()) {
        if (Files.isSymbolicLink(p)) continue;
        Set<PosixFilePermission> perms =
            PosixFilePermissions.fromString(Files.isDirectory(p) ? "rwx------" : "rw-------");
        Files.setPosixFilePermissions(p, perms);
      }
    }
  }
}
