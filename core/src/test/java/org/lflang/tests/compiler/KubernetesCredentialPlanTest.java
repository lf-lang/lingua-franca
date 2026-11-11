package org.lflang.tests.compiler;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.lflang.federated.generator.KubernetesCredentialPlan;
import org.lflang.federated.generator.KubernetesCredentialPlan.SecretSpec;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/** Tests of the per-component credential selection for Kubernetes deployments. */
class KubernetesCredentialPlanTest {

  private static final List<String> ENTITIES = List.of("rti", "federate__a", "federate__p1");

  private static Map<String, String> keyToPath(SecretSpec s) {
    return s.items().stream()
        .collect(
            Collectors.toMap(
                KubernetesCredentialPlan.Item::key, KubernetesCredentialPlan.Item::mountSubPath));
  }

  private static KubernetesCredentialPlan plan(Path gen, CommunicationMode mode, boolean perm) {
    return KubernetesCredentialPlan.create("Prog", gen, mode, perm, ENTITIES);
  }

  @Test
  void tcpNeedsNoSecrets(@TempDir Path gen) {
    assertTrue(plan(gen, CommunicationMode.TCP, false).isEmpty());
  }

  @Test
  void tlsGivesEachEntityOnlyItsOwnKeyAndTheCaCertificate(@TempDir Path gen) {
    var p = plan(gen, CommunicationMode.TLS, false);
    for (String e : ENTITIES) {
      var specs = p.secretsFor(e);
      assertEquals(1, specs.size());
      var s = specs.get(0);
      assertEquals(KubernetesCredentialPlan.TLS_MOUNT, s.mountPath());
      assertEquals(
          Map.of(e + ".crt", e + ".crt", e + ".key", e + ".key", "ca.crt", "ca.crt"), keyToPath(s));
      // Sources are the entity's own directory; never the CA directory (ca.key).
      for (var item : s.items()) {
        assertEquals(gen.resolve("credentials").resolve(e), item.source().getParent());
      }
      assertTrue(s.name().matches("[a-z0-9]([-a-z0-9]*[a-z0-9])?"), s.name());
    }
    assertTrue(p.secretsFor("auth").isEmpty());
  }

  @Test
  void sstDefaultModeRestoresNestedPaths(@TempDir Path gen) {
    var p = plan(gen, CommunicationMode.SST, false);
    var s = p.secretsFor("federate__a").get(0);
    assertEquals(KubernetesCredentialPlan.SST_MOUNT, s.mountPath());
    assertEquals(
        Map.of(
            "federate__a.config", "federate__a.config",
            "Auth101EntityCert.pem", "credentials/auth_certs/Auth101EntityCert.pem",
            "Net1.federate__aKey.pem", "credentials/keys/net1/Net1.federate__aKey.pem"),
        keyToPath(s));
  }

  @Test
  void sstPermanentModeUsesOnlyDistributionKeys(@TempDir Path gen) {
    var p = plan(gen, CommunicationMode.SST, true);
    var s = p.secretsFor("rti").get(0);
    assertEquals(
        Map.of(
            "rti.config", "rti.config",
            "Net1.rtiCipherKey.key", "credentials/keys/net1/Net1.rtiCipherKey.key",
            "Net1.rtiMacKey.key", "credentials/keys/net1/Net1.rtiMacKey.key"),
        keyToPath(s));
  }

  @Test
  void authGetsOnlyItsOwnMaterialAndNoCaPrivateKey(@TempDir Path gen) {
    var p = plan(gen, CommunicationMode.SST, false);
    var auth = p.secretsFor("auth");
    assertEquals(3, auth.size());
    var keys = auth.stream().filter(s -> s.name().endsWith("-keys")).findFirst().orElseThrow();
    var initdb = auth.stream().filter(SecretSpec::initOnly).findFirst().orElseThrow();
    var config = auth.stream().filter(SecretSpec::configMap).findFirst().orElseThrow();
    assertEquals(
        java.util.Set.of(
            "Auth101Entity.pfx",
            "Auth101Internet.pfx",
            "Auth101Database.pfx",
            "Auth101Database.bin",
            "CACert.pem",
            KubernetesCredentialPlan.AUTH_PASSWORD_KEY),
        keyToPath(keys).keySet());
    assertEquals(
        java.util.Set.of(KubernetesCredentialPlan.AUTH_DB_KEY), keyToPath(initdb).keySet());
    assertEquals(
        java.util.Set.of(KubernetesCredentialPlan.AUTH_PROPERTIES_KEY), keyToPath(config).keySet());
    for (var s : p.secrets()) {
      for (var item : s.items()) {
        String src = item.source().toString();
        assertFalse(src.endsWith("CAKey.pem"), src);
        assertFalse(src.contains("auth102") || src.contains("Auth102"), src);
      }
    }
    // Entity secrets never contain Auth material.
    for (String e : ENTITIES) {
      for (var item : p.secretsFor(e).get(0).items()) {
        assertFalse(item.source().toString().contains("/sst/auth/"), item.source().toString());
      }
    }
  }

  @Test
  void stagingFailsWithComponentNameWhenAFileIsMissing(@TempDir Path gen) {
    var p = plan(gen, CommunicationMode.TLS, false);
    var ex = assertThrows(IOException.class, p::stage);
    assertTrue(ex.getMessage().contains("component 'rti'"), ex.getMessage());
  }

  @Test
  void stagedFilesAreOwnerOnly(@TempDir Path gen) throws IOException {
    var p = plan(gen, CommunicationMode.TLS, false);
    for (String e : ENTITIES) {
      Path dir = gen.resolve("credentials").resolve(e);
      Files.createDirectories(dir);
      for (String f : List.of(e + ".crt", e + ".key", "ca.crt"))
        Files.writeString(dir.resolve(f), "x");
    }
    p.stage();
    if (!FileSystems.getDefault().supportedFileAttributeViews().contains("posix")) return;
    for (var s : p.secrets()) {
      for (var item : s.items()) {
        Path staged = p.stagedPath(s, item);
        assertEquals(
            "rw-------", PosixFilePermissions.toString(Files.getPosixFilePermissions(staged)));
        assertEquals(
            "rwx------",
            PosixFilePermissions.toString(Files.getPosixFilePermissions(staged.getParent())));
      }
    }
  }
}
