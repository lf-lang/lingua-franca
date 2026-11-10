package org.lflang.federated.generator;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.util.FileUtil;

/**
 * TLS related methods (cert/key generation + copy to src-gen).
 *
 * - Generate cert+key for RTI and each federate into:
 * fed-gen/[program]/credentials/[name]/
 * - Copy only the needed pair into: src-gen/[entity]/credentials/
 */
public class TLSGenerator {
  private final FederationFileConfig fileConfig;
  private final MessageReporter messageReporter;
  private final LFGeneratorContext context;

  public TLSGenerator(
      FederationFileConfig fileConfig,
      MessageReporter messageReporter,
      LFGeneratorContext context) {
    this.fileConfig = fileConfig;
    this.messageReporter = messageReporter;
    this.context = context;
  }

  /** Entry point called from generator when comm-type is TLS. */
  public void setupTLS(List<FederateInstance> federates) throws IOException {

    // 1) Generate cert/key for RTI + each federate into
    // fed-gen/[program]/credentials
    Path credentialsRoot = getLocalCredentialsRoot(fileConfig); // fed-gen/[program]/credentials
    Files.createDirectories(credentialsRoot);

    // Per-federation CA: every entity certificate is signed by it, and every
    // entity receives ca.crt as its trust anchor for peer verification.
    generateFederationCA(credentialsRoot);

    // RTI
    generateEntityCertAndKey(credentialsRoot, "rti");

    // Federates
    for (FederateInstance fed : federates) {
      generateEntityCertAndKey(credentialsRoot, fed.name);
    }

    // 2) Copy into src-gen for tar deployments (only what each needs)
    copyTLSCredentialsToSrcGen(federates, credentialsRoot);
  }

  /** fed-gen/[program]/credentials */
  public static Path getLocalCredentialsRoot(FederationFileConfig fileConfig) {
    return fileConfig.getGenPath().resolve("credentials");
  }

  public static Path getLocalEntityCredentialsDir(
      FederationFileConfig fileConfig, String entityName) {
    return getLocalCredentialsRoot(fileConfig).resolve(entityName);
  }

  /** fed-gen/[program]/credentials/[entity]/[entity].crt */
  public static Path getLocalCertPath(FederationFileConfig fileConfig, String entityName) {
    return getLocalEntityCredentialsDir(fileConfig, entityName).resolve(entityName + ".crt");
  }

  /** fed-gen/[program]/credentials/[entity]/[entity].key */
  public static Path getLocalKeyPath(FederationFileConfig fileConfig, String entityName) {
    return getLocalEntityCredentialsDir(fileConfig, entityName).resolve(entityName + ".key");
  }

  /**
   * Remote base (NO "~"). You should prefix with "$HOME/" in generated scripts.
   * e.g. "$HOME/" + getRelativeRemoteCredentialsDir(...)
   */
  public static String getRelativeRemoteCredentialsDir(
      FederationFileConfig fileConfig, String entityName) {
    return "LinguaFrancaRemote/" + fileConfig.name + "/" + entityName + "/credentials";
  }

  public static String getRelativeRemoteCertPath(
      FederationFileConfig fileConfig, String entityName) {
    return getRelativeRemoteCredentialsDir(fileConfig, entityName) + "/" + entityName + ".crt";
  }

  public static String getRelativeRemoteKeyPath(
      FederationFileConfig fileConfig, String entityName) {
    return getRelativeRemoteCredentialsDir(fileConfig, entityName) + "/" + entityName + ".key";
  }

  // ------------------------------------------------------------
  // internals
  // ------------------------------------------------------------

  /** credentialsRoot/ca/ca.{key,crt}: self-signed CA for this federation. */
  private void generateFederationCA(Path credentialsRoot) throws IOException {
    Path dir = credentialsRoot.resolve("ca");
    Files.createDirectories(dir);
    String keyStr = FileUtil.toUnixString(dir.resolve("ca.key"));
    String crtStr = FileUtil.toUnixString(dir.resolve("ca.crt"));
    String cmd =
        "openssl req -x509 -newkey rsa:2048 -nodes -keyout "
            + shellEscape(keyStr)
            + " -out "
            + shellEscape(crtStr)
            + " -days 365 -subj "
            + shellEscape("/CN=LF Federation CA " + fileConfig.name)
            + " -addext basicConstraints=critical,CA:TRUE"
            + " -addext keyUsage=critical,keyCertSign,cRLSign";
    messageReporter.nowhere().info("Generating TLS federation CA...");
    runLocalCommand(cmd, "[TLS Gen CA]");
  }

  /**
   * Generate a key and a certificate signed by the federation CA for one entity under
   * credentialsRoot/<entityName>/, and place a copy of ca.crt next to them.
   */
  private void generateEntityCertAndKey(Path credentialsRoot, String entityName)
      throws IOException {
    Path dir = credentialsRoot.resolve(entityName);
    Files.createDirectories(dir);

    Path key = dir.resolve(entityName + ".key");
    Path crt = dir.resolve(entityName + ".crt");
    Path csr = dir.resolve(entityName + ".csr");
    Path ext = dir.resolve(entityName + ".ext");
    Path caDir = credentialsRoot.resolve("ca");

    // Using openssl to generate:
    // - RSA 2048 key (unencrypted) + CSR with CN=<entityName>
    // - certificate signed by the federation CA, usable for TLS server and client auth
    // NOTE: Use unix paths in the shell command.
    String keyStr = FileUtil.toUnixString(key);
    String crtStr = FileUtil.toUnixString(crt);
    String csrStr = FileUtil.toUnixString(csr);
    String extStr = FileUtil.toUnixString(ext);
    String caKey = FileUtil.toUnixString(caDir.resolve("ca.key"));
    String caCrt = FileUtil.toUnixString(caDir.resolve("ca.crt"));

    Files.writeString(
        ext,
        "basicConstraints=critical,CA:FALSE\n"
            + "keyUsage=critical,digitalSignature,keyEncipherment\n"
            + "extendedKeyUsage=serverAuth,clientAuth\n");

    String cmd =
        "openssl req -newkey rsa:2048 -nodes -keyout "
            + shellEscape(keyStr)
            + " -out "
            + shellEscape(csrStr)
            + " -subj "
            + shellEscape("/CN=" + entityName)
            + " && openssl x509 -req -in "
            + shellEscape(csrStr)
            + " -CA "
            + shellEscape(caCrt)
            + " -CAkey "
            + shellEscape(caKey)
            + " -set_serial "
            + Math.abs(new java.security.SecureRandom().nextLong())
            + " -days 365 -extfile "
            + shellEscape(extStr)
            + " -out "
            + shellEscape(crtStr)
            + " && rm -f "
            + shellEscape(csrStr)
            + " "
            + shellEscape(extStr);

    messageReporter.nowhere().info("Generating TLS certificate and key for " + entityName + "...");
    runLocalCommand(cmd, "[TLS Gen " + entityName + "]");
    Files.copy(
        caDir.resolve("ca.crt"),
        dir.resolve("ca.crt"),
        java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    messageReporter
        .nowhere()
        .info("Successfully generated TLS credentials for " + entityName + ".");
  }

  private void copyTLSCredentialsToSrcGen(List<FederateInstance> federates, Path credentialsRoot)
      throws IOException {

    // Federates: src-gen/<fed>/credentials/ (copy only that fed's key+cert)
    for (FederateInstance fed : federates) {
      Path dst = fileConfig.getSrcGenPath().resolve(fed.name).resolve("credentials");
      Files.createDirectories(dst);

      Path srcDir = credentialsRoot.resolve(fed.name);
      if (!Files.isDirectory(srcDir)) {
        throw new IOException("Missing TLS credentials dir for federate: " + srcDir);
      }

      // Copy only files in that entity dir
      FileUtil.copyDirectoryContents(srcDir, dst, false);
    }

    // RTI: src-gen/RTI/credentials/
    Path rtiDst = fileConfig.getRtiSrcGenPath().resolve("credentials");
    Files.createDirectories(rtiDst);

    Path rtiSrcDir = credentialsRoot.resolve("rti");
    if (!Files.isDirectory(rtiSrcDir)) {
      throw new IOException("Missing TLS credentials dir for RTI: " + rtiSrcDir);
    }
    FileUtil.copyDirectoryContents(rtiSrcDir, rtiDst, false);
  }

  private void runLocalCommand(String bashCommand, String tag) throws IOException {
    ProcessBuilder pb = new ProcessBuilder("bash", "-c", bashCommand);
    pb.redirectErrorStream(true);
    Process p = pb.start();

    try (BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()))) {
      String line;
      while ((line = r.readLine()) != null) {
        // Suppress openssl's progress indicator lines like ".+...+...+*..." or "-----"
        if (line.matches("^[\\.\\+\\*\\- ]+$")) {
          continue;
        }
        messageReporter.nowhere().info(tag + " " + line);
      }
    }

    try {
      int code = p.waitFor();
      if (code != 0) {
        throw new IOException(tag + " command failed (exit=" + code + "): " + bashCommand);
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException(tag + " interrupted", e);
    }
  }

  private static String shellEscape(String s) {
    // simplest safe-ish quoting for bash -c
    return "'" + s.replace("'", "'\"'\"'") + "'";
  }
}
