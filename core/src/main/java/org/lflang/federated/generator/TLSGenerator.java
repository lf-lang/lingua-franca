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
 * - Generate cert+key for RTI and each federate into: fed-gen/<program>/credentials/<name>/
 * - Copy only the needed pair into: src-gen/<entity>/credentials/
 */
public class TLSGenerator {

  /** Entry point called from generator when comm-type is TLS. */
  public static void setupTLS(
      FederationFileConfig fileConfig,
      List<FederateInstance> federates,
      MessageReporter messageReporter,
      LFGeneratorContext context
  ) throws IOException {

    // 1) Generate cert/key for RTI + each federate into fed-gen/<program>/credentials
    Path credentialsRoot = getLocalCredentialsRoot(fileConfig); // fed-gen/<program>/credentials
    Files.createDirectories(credentialsRoot);

    // RTI
    generateEntityCertAndKey(credentialsRoot, "rti", messageReporter);

    // Federates
    for (FederateInstance fed : federates) {
      generateEntityCertAndKey(credentialsRoot, fed.name, messageReporter);
    }

    // 2) Copy into src-gen for tar deployments (only what each needs)
    copyTLSCredentialsToSrcGen(fileConfig, federates, credentialsRoot);
  }

  /** fed-gen/<program>/credentials */
  public static Path getLocalCredentialsRoot(FederationFileConfig fileConfig) {
    return fileConfig.getGenPath().resolve("credentials");
  }

  public static Path getLocalEntityCredentialsDir(FederationFileConfig fileConfig, String entityName) {
    return getLocalCredentialsRoot(fileConfig).resolve(entityName);
  }

  /** fed-gen/<program>/credentials/<entity>/<entity>.crt */
  public static Path getLocalCertPath(FederationFileConfig fileConfig, String entityName) {
    return getLocalEntityCredentialsDir(fileConfig, entityName).resolve(entityName + ".crt");
  }

  /** fed-gen/<program>/credentials/<entity>/<entity>.key */
  public static Path getLocalKeyPath(FederationFileConfig fileConfig, String entityName) {
    return getLocalEntityCredentialsDir(fileConfig, entityName).resolve(entityName + ".key");
  }

  /**
   * Remote base (NO "~"). You should prefix with "$HOME/" in generated scripts.
   * e.g. "$HOME/" + getRelativeRemoteCredentialsDir(...)
   */
  public static String getRelativeRemoteCredentialsDir(FederationFileConfig fileConfig, String entityName) {
    return "LinguaFrancaRemote/" + fileConfig.name + "/" + entityName + "/credentials";
  }

  public static String getRelativeRemoteCertPath(FederationFileConfig fileConfig, String entityName) {
    return getRelativeRemoteCredentialsDir(fileConfig, entityName) + "/" + entityName + ".crt";
  }

  public static String getRelativeRemoteKeyPath(FederationFileConfig fileConfig, String entityName) {
    return getRelativeRemoteCredentialsDir(fileConfig, entityName) + "/" + entityName + ".key";
  }

  // ------------------------------------------------------------
  // internals
  // ------------------------------------------------------------

  /** Generate a self-signed cert+key for one entity under credentialsRoot/<entityName>/ */
  private static void generateEntityCertAndKey(
      Path credentialsRoot, String entityName, MessageReporter reporter
  ) throws IOException {
    Path dir = credentialsRoot.resolve(entityName);
    Files.createDirectories(dir);

    Path key = dir.resolve(entityName + ".key");
    Path crt = dir.resolve(entityName + ".crt");


    // Using openssl to generate:
    //   - RSA 2048 key (unencrypted) + self-signed cert, CN=<entityName>
    // NOTE: Use unix paths in the shell command.
    String keyStr = FileUtil.toUnixString(key);
    String crtStr = FileUtil.toUnixString(crt);

    String cmd =
        "openssl req -x509 -newkey rsa:2048 -nodes "
            + "-keyout " + shellEscape(keyStr) + " "
            + "-out " + shellEscape(crtStr) + " "
            + "-days 365 "
            + "-subj " + shellEscape("/CN=" + entityName);

    runLocalCommand(cmd, reporter, "[TLS Gen " + entityName + "]");
  }

  private static void copyTLSCredentialsToSrcGen(
      FederationFileConfig fileConfig,
      List<FederateInstance> federates,
      Path credentialsRoot
  ) throws IOException {

    // Federates: src-gen/<fed>/credentials/  (copy only that fed's key+cert)
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

  private static void runLocalCommand(String bashCommand, MessageReporter reporter, String tag)
      throws IOException {
    ProcessBuilder pb = new ProcessBuilder("bash", "-c", bashCommand);
    pb.redirectErrorStream(true);
    Process p = pb.start();

    try (BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()))) {
      String line;
      while ((line = r.readLine()) != null) {
        reporter.nowhere().info(tag + " " + line);
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