package org.lflang.federated.generator;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.SSTProperty;
import org.lflang.util.EvalTrace;
import org.lflang.util.FileUtil;

/**
 * SST related methods.
 *
 * @author Dongha Kim
 */
public class SSTGenerator {
  private final FederationFileConfig fileConfig;
  private final MessageReporter messageReporter;
  private final LFGeneratorContext context;

  public SSTGenerator(
      FederationFileConfig fileConfig,
      MessageReporter messageReporter,
      LFGeneratorContext context) {
    this.fileConfig = fileConfig;
    this.messageReporter = messageReporter;
    this.context = context;
  }

  public void setupSST(List<FederateInstance> federates, RtiConfig rtiConfig, String authHost)
      throws IOException {
    String sstRootPath = context.getTargetConfig().get(SSTProperty.INSTANCE).rootPath();
    if (sstRootPath == null || sstRootPath.isEmpty()) {
      sstRootPath = System.getenv("SST_ROOT");
    }

    if (sstRootPath == null || sstRootPath.isEmpty()) {
      context
          .getErrorReporter()
          .nowhere()
          .error(
              "`comm-type: SST` requires a path to the SST repository. Please either set the"
                  + " `sst-root-path` target property or set the `SST_ROOT` environment variable.");
      return;
    }

    var dockerEnabled = context.getTargetConfig().get(DockerProperty.INSTANCE).enabled();
    if (dockerEnabled) {
      for (FederateInstance federate : federates) {
        copySSTCSource(federate.name);
      }
      copySSTCSource("rti");
    }

    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTConfigPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTCredentialsPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTGraphsPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTPolicyPath().toFile());

    // Create graph used when creating credentials.
    // Set graph path.
    Path graphPath = fileConfig.getSSTGraphsPath().resolve(fileConfig.name + ".graph");
    boolean usePermanentDistKey =
        context.getTargetConfig().get(SSTProperty.INSTANCE).usePermanentDistKey();
    // Generate the graph file content
    JsonObject graphObject = generateGraphFile(federates, rtiConfig, usePermanentDistKey);
    // Write the graph object to a JSON file
    try (FileWriter fileWriter = new FileWriter(graphPath.toString())) {
      Gson gson = new GsonBuilder().setPrettyPrinting().create();
      gson.toJson(graphObject, fileWriter);
      messageReporter
          .nowhere()
          .info("Graph file generated successfully into: " + graphPath.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Write the policy file (JSON array).
    Path policyPath = fileConfig.getSSTPolicyPath().resolve(fileConfig.name + ".json");
    JsonArray policyArray = generateCommunicationPolicy();

    try (FileWriter fileWriter = new FileWriter(policyPath.toString())) {
      Gson gson = new GsonBuilder().setPrettyPrinting().create();
      gson.toJson(policyArray, fileWriter);
      messageReporter.nowhere().info("Policy file generated successfully into: " + policyPath);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Set root path to execute commands.
    Path sstRepoRootPath = Paths.get(sstRootPath);
    ProcessBuilder processBuilder = new ProcessBuilder();

    // Set the working directory to the specified path
    processBuilder.directory(sstRepoRootPath.resolve("examples").toFile());

    // Clean the old credentials & generate new credentials.

    processBuilder.command(
        "bash",
        "-c",
        "echo \"Executing: ./cleanAll.sh ; ./generateAll.sh -g "
            + graphPath
            + " -p "
            + fileConfig.name
            + " --policy "
            + policyPath
            + "\" && "
            + "./cleanAll.sh ; ./generateAll.sh -g "
            + graphPath
            + " -p "
            + fileConfig.name
            + " --policy "
            + policyPath
            + " && "
            + "echo \"generateAll.sh finished successfully.\"");

    // Start the process
    try {
      EvalTrace.begin("gen.sst.generate_credentials");
      Process process = processBuilder.start();

      // Create threads to capture output and error streams
      Thread outputThread =
          new Thread(
              () -> {
                try (BufferedReader reader =
                    new BufferedReader(new InputStreamReader(process.getInputStream()))) {
                  String line;
                  while ((line = reader.readLine()) != null) {
                    messageReporter.nowhere().info("[SST Script] " + line);
                  }
                } catch (IOException e) {
                  e.printStackTrace();
                }
              });

      outputThread.start();

      int exitCode = process.waitFor(); // Wait for process to finish
      EvalTrace.end("gen.sst.generate_credentials");
      outputThread.join();

      if (exitCode == 0) {
        messageReporter.nowhere().info("Credential generation script execution succeeded.");
      } else {
        messageReporter.nowhere().error("Script execution failed with exit code: " + exitCode);
      }
    } catch (IOException | InterruptedException e) {
      throw new RuntimeException(e);
    }

    // Build the auth-server
    ProcessBuilder mvnProcessBuilder = new ProcessBuilder();
    mvnProcessBuilder.directory(sstRepoRootPath.resolve("auth").resolve("auth-server").toFile());
    mvnProcessBuilder.command("mvn", "clean", "install");

    try {
      EvalTrace.begin("build.sst.auth_maven");
      Process mvnProcess = mvnProcessBuilder.start();

      // Create threads to capture output and error streams
      Thread mvnOutputThread =
          new Thread(
              () -> {
                try (BufferedReader reader =
                    new BufferedReader(new InputStreamReader(mvnProcess.getInputStream()))) {
                  String line;
                  while ((line = reader.readLine()) != null) {
                    messageReporter.nowhere().info("[SST Auth Server] " + line);
                  }
                } catch (IOException e) {
                  e.printStackTrace();
                }
              });

      mvnOutputThread.start();

      int mvnExitCode = mvnProcess.waitFor();
      EvalTrace.end("build.sst.auth_maven");
      mvnOutputThread.join();

      if (mvnExitCode == 0) {
        messageReporter.nowhere().info("Auth server built successfully.");
      } else {
        messageReporter.nowhere().error("Auth server build failed with exit code: " + mvnExitCode);
      }
    } catch (IOException | InterruptedException e) {
      throw new RuntimeException(e);
    }

    // Copy credentials.
    try {
      copyCredentials(sstRepoRootPath);
      messageReporter
          .nowhere()
          .info("Credentials copied into: " + fileConfig.getSSTCredentialsPath().toString());
      copyAuthNecessary(sstRepoRootPath);
      messageReporter
          .nowhere()
          .info("Auth necessary files copied into: " + fileConfig.getSSTAuthPath().toString());
      updatePropertiesFile(
          fileConfig.getSSTAuthPath().resolve("properties"),
          fileConfig.getSSTAuthPath().toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    // The local credential store (entity keys, Auth CA key, Auth databases) is owner-only.
    KubernetesCredentialPlan.restrictTree(fileConfig.getSSTCredentialsPath());
    KubernetesCredentialPlan.restrictTree(fileConfig.getSSTAuthPath());

    var useDocker = context.getTargetConfig().get(DockerProperty.INSTANCE).enabled();
    var isKubernetes = KubernetesCredentialPlan.isKubernetes(context.getTargetConfig());
    Path srcGenAuthPath = context.getFileConfig().getSrcGenPath().resolve("auth");
    try {
      if (isKubernetes) {
        // Kubernetes: the Auth image build context contains only the server jar. Keystores,
        // the initial database, the trusted CA certificate and the password input are provided
        // at run time (see KubernetesCredentialPlan).
        Files.createDirectories(srcGenAuthPath);
        FileUtil.copyFile(
            fileConfig.getSSTAuthPath().resolve("auth-server-jar-with-dependencies.jar"),
            srcGenAuthPath.resolve("auth-server-jar-with-dependencies.jar"));
      } else if (useDocker) {
        FileUtil.copyDirectoryContents(fileConfig.getSSTAuthPath(), srcGenAuthPath, false);
        updatePropertiesFile(srcGenAuthPath.resolve("properties"), "/auth");
      }
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Generate SST config for the RTI.
    generateSSTConfig("rti", rtiConfig.getHost(), authHost, useDocker, usePermanentDistKey);
    messageReporter
        .nowhere()
        .info("Generated RTI's SST config into: " + getSSTConfigPath(fileConfig, "rti").toString());

    // Generate SST config for the federates.
    for (FederateInstance federate : federates) {
      generateSSTConfig(
          federate.name, rtiConfig.getHost(), authHost, useDocker, usePermanentDistKey);
      messageReporter
          .nowhere()
          .info(
              "Federate generated SST config into: "
                  + getSSTConfigPath(fileConfig, federate.name).toString());
    }

    // Copy the configs and credentials of rti and federates, to the src-gen for tar
    // deployments.
    copyAuthAndConfigsAndKeys(federates, usePermanentDistKey, useDocker);
  }

  public static Path getSSTConfigPath(FederationFileConfig fileConfig, String name) {
    return fileConfig.getSSTConfigPath().resolve(name + ".config");
  }

  private void copySSTCSource(String name) {
    var destDirBase = context.getFileConfig().getSrcGenPath().resolve(name).resolve("sst-src");
    String sstRootPath = context.getTargetConfig().get(SSTProperty.INSTANCE).rootPath();
    if (sstRootPath == null || sstRootPath.isEmpty()) {
      sstRootPath = System.getenv("SST_ROOT");
    }
    var srcDirBase = Path.of(sstRootPath).resolve("entity/c");

    try {
      // Copy the files in the src folder
      var destDir = destDirBase.resolve("src");
      FileUtil.createDirectoryIfDoesNotExist(destDir.toFile());
      var srcDir = srcDirBase.resolve("src");
      FileUtil.copyDirectoryContents(srcDir, destDir, false);

      // Copy the test files
      destDir = destDirBase.resolve("tests");
      FileUtil.createDirectoryIfDoesNotExist(destDir.toFile());
      srcDir = srcDirBase.resolve("tests");
      FileUtil.copyDirectoryContents(srcDir, destDir, false);

      // copy cmake
      destDir = destDirBase.resolve("cmake");
      FileUtil.createDirectoryIfDoesNotExist(destDir.toFile());
      srcDir = srcDirBase.resolve("cmake");
      FileUtil.copyDirectoryContents(srcDir, destDir, false);

      // copy CMakeLists.txt
      FileUtil.copyFile(
          srcDirBase.resolve("CMakeLists.txt"), destDirBase.resolve("CMakeLists.txt"));
    } catch (IOException e) {
      context
          .getErrorReporter()
          .nowhere()
          .error("Error while copying sst files: " + e.getMessage());
    }
  }

  private void generateSSTConfig(
      String name, String rtiIP, String authIP, boolean useDocker, boolean usePermanentDistKey) {
    // Values to fill in
    String entityName = "net1." + name;
    int authID = 101;
    String sessionkey_encryptionMode = "AES_128_CBC";
    int hmacMode = 1;
    String pubkeyRoot =
        useDocker
            ? "sst/credentials/auth_certs/Auth101EntityCert.pem"
            : fileConfig.getSSTCredentialsPath().resolve("auth_certs").toString()
                + File.separator
                + "Auth"
                + authID
                + "EntityCert.pem";
    String privkeyRoot =
        useDocker
            ? "sst/credentials/keys/net1/Net1." + name + "Key.pem"
            : fileConfig.getSSTCredentialsPath().resolve("keys").resolve("net1").toString()
                + File.separator
                + "Net1."
                + name
                + "Key.pem";
    // In containers the credentials are under ./sst/credentials (relative to the working
    // directory), as for the private key above.
    String cipherkeyRoot =
        useDocker
            ? "sst/credentials/keys/net1/Net1." + name + "CipherKey.key"
            : fileConfig.getSSTCredentialsPath().resolve("keys").resolve("net1").toString()
                + File.separator
                + "Net1."
                + name
                + "CipherKey.key";
    String mackeyRoot =
        useDocker
            ? "sst/credentials/keys/net1/Net1." + name + "MacKey.key"
            : fileConfig.getSSTCredentialsPath().resolve("keys").resolve("net1").toString()
                + File.separator
                + "Net1."
                + name
                + "MacKey.key";
    if ("localhost".equals(rtiIP)) {
      rtiIP = "127.0.0.1";
      authIP = "127.0.0.1";
    }
    String authIpAddress = authIP;
    int authPortNumber = 21900;
    String entityServerIpAddress = rtiIP;
    int entityServerPortNumber = 15045;
    String networkProtocol = "TCP";

    // Create the configuration content
    StringBuilder configContent = new StringBuilder();
    configContent
        .append("entityInfo.name=")
        .append(entityName)
        .append("\n")
        .append("entityInfo.purpose={\"group\":\"RTI\"}\n")
        .append("entityInfo.number_key=1\n")
        .append("authInfo.id=")
        .append(authID)
        .append("\n")
        .append("sessionKey.encryptionMode=")
        .append(sessionkey_encryptionMode)
        .append("\n")
        .append("HmacMode=")
        .append(hmacMode)
        .append("\n");

    if (usePermanentDistKey) {
      configContent
          .append("PermanentDistKeyMode=on\n")
          .append("distKey.encryptionMode=")
          .append(sessionkey_encryptionMode)
          .append("\n")
          .append("distKey.cipherkey.path=")
          .append(cipherkeyRoot)
          .append("\n")
          .append("distkey.mackey.path=")
          .append(mackeyRoot)
          .append("\n");
    } else {
      configContent
          .append("authInfo.pubkey.path=")
          .append(pubkeyRoot)
          .append("\n")
          .append("entityInfo.privkey.path=")
          .append(privkeyRoot)
          .append("\n");
    }

    configContent
        .append("auth.ip.address=")
        .append(authIpAddress)
        .append("\n")
        .append("auth.port.number=")
        .append(authPortNumber)
        .append("\n")
        .append("entity.server.ip.address=")
        .append(entityServerIpAddress)
        .append("\n")
        .append("entity.server.port.number=")
        .append(entityServerPortNumber)
        .append("\n")
        .append("network.protocol=")
        .append(networkProtocol)
        .append("\n");

    try {
      // Create the new file and write the modified content
      Path newFilePath;
      newFilePath = fileConfig.getSSTConfigPath().resolve(name + ".config");
      // Create /SST directories if necessary
      Files.createDirectories(newFilePath.getParent().getParent());
      // Create /SST/configs directories if necessary
      Files.createDirectories(newFilePath.getParent()); // Create parent directories if necessary
      BufferedWriter writer = new BufferedWriter(new FileWriter(newFilePath.toFile(), false));
      writer.write(configContent.toString());
      writer.close();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private JsonObject generateGraphFile(
      List<FederateInstance> federateInstances, RtiConfig rtiConfig, boolean usePermanentDistKey) {
    JsonObject graphObject = new JsonObject();

    // Auth list
    JsonArray authList = new JsonArray();
    authList.add(
        createAuthEntry(101, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
    authList.add(
        createAuthEntry(102, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
    graphObject.add("authList", authList);

    // Auth trusts
    JsonArray authTrusts = new JsonArray();
    JsonObject trustRelation = new JsonObject();
    trustRelation.addProperty("id1", 101);
    trustRelation.addProperty("id2", 102);
    authTrusts.add(trustRelation);
    graphObject.add("authTrusts", authTrusts);

    // Assignments section
    JsonObject assignments = new JsonObject();
    assignments.addProperty("net1.rti", 101);
    for (FederateInstance federate : federateInstances) {
      assignments.addProperty("net1." + federate.name, 101); // Assuming "101" is a placeholder
    }
    graphObject.add("assignments", assignments);

    // Entity list section
    JsonArray entityList = createEntityList(federateInstances, rtiConfig, usePermanentDistKey);
    graphObject.add("entityList", entityList);

    // File sharing lists (empty for this example)
    graphObject.add("filesharingLists", new JsonArray());

    // Privilege list (empty for this example)
    graphObject.add("privilegeList", new JsonArray());

    return graphObject;
  }

  private JsonObject createGroupPolicy(
      String requestingGroup,
      String targetType,
      String target,
      int maxNumSessionKeyOwners,
      String sessionCryptoSpec,
      String absoluteValidity,
      String relativeValidity) {

    JsonObject o = new JsonObject();
    o.addProperty("RequestingGroup", requestingGroup);
    o.addProperty("TargetType", targetType);
    o.addProperty("Target", target);
    o.addProperty("MaxNumSessionKeyOwners", maxNumSessionKeyOwners);
    o.addProperty("SessionCryptoSpec", sessionCryptoSpec);
    o.addProperty("AbsoluteValidity", absoluteValidity);
    o.addProperty("RelativeValidity", relativeValidity);
    o.addProperty("Expiration", "Infinity");
    o.addProperty("IsDelegated", 0);
    return o;
  }

  // Creates the policy JSON array to be passed to authConfigGenerator.js via
  // --policy <file>.
  private JsonArray generateCommunicationPolicy() {
    JsonArray policies = new JsonArray();

    policies.add(
        createGroupPolicy("Federates", "Group", "RTI", 2, "AES-128-CBC:SHA256", "1*day", "2*hour"));

    policies.add(
        createGroupPolicy(
            "Federates", "Group", "Federates", 2, "AES-128-CBC:SHA256", "1*day", "2*hour"));

    return policies;
  }

  private JsonObject createAuthEntry(
      int id,
      String entityHost,
      String authHost,
      int tcpPort,
      int udpPort,
      int authPort,
      int callbackPort,
      int dbProtectionMethod,
      boolean backupEnabled,
      boolean contextualCallbackEnabled) {
    JsonObject authEntry = new JsonObject();
    authEntry.addProperty("id", id);
    authEntry.addProperty("entityHost", entityHost);
    authEntry.addProperty("authHost", authHost);
    authEntry.addProperty("tcpPort", tcpPort);
    authEntry.addProperty("udpPort", udpPort);
    authEntry.addProperty("authPort", authPort);
    authEntry.addProperty("callbackPort", callbackPort);
    authEntry.addProperty("dbProtectionMethod", dbProtectionMethod);
    authEntry.addProperty("backupEnabled", backupEnabled);
    authEntry.addProperty("contextualCallbackEnabled", contextualCallbackEnabled);
    return authEntry;
  }

  private JsonArray createEntityList(
      List<FederateInstance> federateInstances, RtiConfig rtiConfig, boolean usePermanentDistKey) {
    JsonArray entityList = new JsonArray();

    // RTI entity
    JsonObject rti = createEntity("RTI", "net1.rti", "Net1.rti", usePermanentDistKey);
    rti.addProperty("port", rtiConfig.getPort());
    rti.addProperty("host", rtiConfig.getHost());
    entityList.add(rti);

    // Federate entities
    for (FederateInstance federate : federateInstances) {
      String federateName = federate.name;
      JsonObject entity =
          createEntity(
              "Federates", "net1." + federateName, "Net1." + federateName, usePermanentDistKey);
      entity.addProperty("host", federate.host);
      entityList.add(entity);
    }
    return entityList;
  }

  private JsonObject createEntity(
      String group, String name, String credentialPrefix, boolean usePermanentDistKey) {
    JsonObject entity = new JsonObject();
    entity.addProperty("group", group);
    entity.addProperty("name", name);
    entity.addProperty("distProtocol", "TCP");
    entity.addProperty("usePermanentDistKey", usePermanentDistKey);
    entity.addProperty("distKeyValidityPeriod", "1*hour");
    entity.addProperty("maxSessionKeysPerRequest", 1);
    entity.addProperty("netName", "net1");
    entity.addProperty("credentialPrefix", credentialPrefix);
    // Add distributionCryptoSpec
    JsonObject distributionCryptoSpec = new JsonObject();
    distributionCryptoSpec.addProperty("cipher", "AES-128-CBC");
    distributionCryptoSpec.addProperty("mac", "SHA256");
    entity.add("distributionCryptoSpec", distributionCryptoSpec);

    // Add sessionCryptoSpec
    JsonObject sessionCryptoSpec = new JsonObject();
    sessionCryptoSpec.addProperty("cipher", "AES-128-CBC");
    sessionCryptoSpec.addProperty("mac", "SHA256");
    entity.add("sessionCryptoSpec", sessionCryptoSpec);

    entity.addProperty("host", "localhost");
    entity.add("backupToAuthIds", new JsonArray()); // Empty array for backupToAuthIds
    return entity;
  }

  private void copyCredentials(Path sstRepoRootPath) throws IOException {
    // Copy auth_certs.
    Path source1 = sstRepoRootPath.resolve("entity").resolve("auth_certs");
    Path destination1 = fileConfig.getSSTCredentialsPath().resolve("auth_certs");

    // Copy keys.
    Path source2 = sstRepoRootPath.resolve("entity").resolve("credentials").resolve("keys");
    Path destination2 = fileConfig.getSSTCredentialsPath().resolve("keys");
    FileUtil.copyDirectoryContents(source1, destination1, false);
    FileUtil.copyDirectoryContents(source2, destination2, false);
  }

  private void copyAuthNecessary(Path sstRepoRootPath) throws IOException {
    // Copy Auth credentials.
    Path source1 = sstRepoRootPath.resolve("auth").resolve("credentials").resolve("ca");
    Path destination1 = fileConfig.getSSTAuthPath().resolve("credentials").resolve("ca");

    // Copy Auth databases.
    Path source2 = sstRepoRootPath.resolve("auth").resolve("databases");
    Path destination2 = fileConfig.getSSTAuthPath().resolve("databases");

    // Copy Auth properties.
    Path source3 = sstRepoRootPath.resolve("auth").resolve("properties");
    Path destination3 = fileConfig.getSSTAuthPath().resolve("properties");

    // Copy jar file.
    Path source4 =
        sstRepoRootPath
            .resolve("auth")
            .resolve("auth-server")
            .resolve("target")
            .resolve("auth-server-jar-with-dependencies.jar");
    Path destination4 =
        fileConfig.getSSTAuthPath().resolve("auth-server-jar-with-dependencies.jar");

    FileUtil.copyDirectoryContents(source1, destination1, false);
    FileUtil.copyDirectoryContents(source2, destination2, false);
    FileUtil.copyDirectoryContents(source3, destination3, false);
    FileUtil.copyFile(source4, destination4);
  }

  /**
   * Update all .properties files under the given propertiesDir.
   *
   * @param propertiesDir  Path to the ".../sst/auth/properties" directory (parent
   *                       path only).
   * @param updateBasePath new base path to use for replacement (must point to
   *                       ".../sst/auth/" or ".../RTI/auth/")
   */
  private void updatePropertiesFile(Path propertiesDir, String updateBasePath) throws IOException {
    if (propertiesDir == null) {
      throw new IllegalArgumentException("propertiesDir must not be null");
    }
    if (!Files.isDirectory(propertiesDir)) {
      throw new IOException("Properties directory not found: " + propertiesDir);
    }

    if (!updateBasePath.endsWith("/")) {
      updateBasePath += "/";
    }
    // Find all .properties files under the directory (recursive)
    List<Path> propFiles = FileUtil.globFilesEndsWith(propertiesDir, ".properties");
    if (propFiles.isEmpty()) {
      // nothing to do
      return;
    }

    for (Path propFile : propFiles) {
      List<String> updatedLines = new ArrayList<>();
      try (BufferedReader reader = new BufferedReader(new FileReader(propFile.toFile()))) {
        String line;
        while ((line = reader.readLine()) != null) {
          if (line.startsWith("entity_key_store_path=")
              || line.startsWith("internet_key_store_path=")
              || line.startsWith("database_key_store_path=")
              || line.startsWith("database_encryption_key_path=")
              || line.startsWith("trusted_ca_cert_paths=")
              || line.startsWith("auth_database_dir=")) {
            line = updatePath(line, updateBasePath);
          }
          updatedLines.add(line);
        }
      }

      // Write changes back only if something changed
      String joined = String.join("\n", updatedLines) + "\n";
      FileUtil.writeToFile(joined, propFile);
    }
  }

  private String updatePath(String line, String base) {
    int idx = line.indexOf('=');
    if (idx < 0) return line;

    String key = line.substring(0, idx + 1);
    String value = line.substring(idx + 1);

    // Case 1: relative "../..." -> just strip "../"
    if (value.startsWith("../")) {
      return key + base + value.substring("../".length());
    }

    // Case 2: absolute path already -> rewrite using tail after "/sst/auth/"
    String marker = "/sst/auth/";
    int m = value.indexOf(marker);
    if (m >= 0) {
      String tail = value.substring(m + marker.length()); // e.g. "databases/auth101/..."
      return key + base + tail;
    }

    // Otherwise: unknown format, leave unchanged
    return line;
  }

  private void copyAuthAndConfigsAndKeys(
      List<FederateInstance> federates, boolean usePermanentDistKey, boolean useDocker)
      throws IOException {
    if (KubernetesCredentialPlan.isKubernetes(context.getTargetConfig())) {
      // Kubernetes: nothing secret goes into src-gen (the Docker build contexts). Stage each
      // component's files into the provisioning directory instead.
      stageForKubernetes(federates);
      return;
    }

    if (useDocker) {
      // Docker Compose: unchanged behavior (credentials are copied into the images).
      Path auth_src = fileConfig.getSSTAuthPath();
      Path rti_src = fileConfig.getRtiSrcGenPath().resolve("auth");
      FileUtil.copyDirectoryContents(auth_src, rti_src, false);
      updatePropertiesFile(
          rti_src.resolve("properties"), getRelativeRemoteBasePath("RTI") + "/auth/");
    } else {
      // SCP/SSH: Auth gets its own source tree (and archive), deployed to the RTI host. It holds
      // only what Auth AUTH_ID needs at run time: not the CA private key and not the other Auth's
      // keystores and database.
      stageAuthForRemote(fileConfig.getSrcGenPath().resolve("auth"));
    }

    // 2. Copy Configs and Keys to src-gen of federates and RTIs.
    Path credentialsRoot = fileConfig.getSSTCredentialsPath(); // .../sst/credentials
    Path keysRoot = credentialsRoot.resolve("keys");
    Path configsRoot = fileConfig.getSSTConfigPath();
    Path authCertsRoot = credentialsRoot.resolve("auth_certs");

    // =========================
    // Federates
    // =========================
    for (FederateInstance federate : federates) {
      Path dst = fileConfig.getSrcGenPath().resolve(federate.name).resolve("sst");
      Files.createDirectories(dst);

      // ---- Make sure we keep the credentials directory structure
      Path dstCredentialsRoot = dst.resolve("credentials");
      Files.createDirectories(dstCredentialsRoot);

      // 1) Copy private key
      if (usePermanentDistKey) {
        String cipherKeySuffix = federate.name + "CipherKey.key";
        String macKeySuffix = federate.name + "MacKey.key";
        List<Path> cipherKeyMatches = FileUtil.globFilesEndsWith(keysRoot, cipherKeySuffix);
        List<Path> macKeyMatches = FileUtil.globFilesEndsWith(keysRoot, macKeySuffix);
        if (cipherKeyMatches.isEmpty() || macKeyMatches.isEmpty()) {
          throw new IOException(
              "No key file found for federate: " + federate.name + " under " + keysRoot);
        }
        Path cipherKeyFile = cipherKeyMatches.get(0);
        Path cipherKeyRel = credentialsRoot.relativize(cipherKeyFile);
        Path cipherKeyDst = dstCredentialsRoot.resolve(cipherKeyRel);
        Files.createDirectories(cipherKeyDst.getParent());
        FileUtil.copyFile(cipherKeyFile, cipherKeyDst);

        Path macKeyFile = macKeyMatches.get(0);
        Path macKeyRel = credentialsRoot.relativize(macKeyFile);
        Path macKeyDst = dstCredentialsRoot.resolve(macKeyRel);
        FileUtil.copyFile(macKeyFile, macKeyDst);
        // The entity's RSA private key is not copied: in permanent distribution-key mode the SST
        // C API loads only the distribution cipher and MAC keys (init_SST()).
      } else {
        String keySuffix = federate.name + "Key.pem";
        List<Path> keyMatches = FileUtil.globFilesEndsWith(keysRoot, keySuffix);
        if (keyMatches.isEmpty()) {
          throw new IOException(
              "No key file found for federate: "
                  + federate.name
                  + " (expected suffix: "
                  + keySuffix
                  + ") under "
                  + keysRoot);
        }
        Path keyFile = keyMatches.get(0);

        // Preserve structure: credentials/keys/net1/<file>
        Path keyRel = credentialsRoot.relativize(keyFile); // keys/net1/Net1.xxxKey.pem
        Path keyDst = dstCredentialsRoot.resolve(keyRel); // dst/credentials/keys/net1/...
        Files.createDirectories(keyDst.getParent());
        FileUtil.copyFile(keyFile, keyDst);
      }

      // 2) Copy config
      Path configSrc = configsRoot.resolve(federate.name + ".config");
      if (!Files.isRegularFile(configSrc)) {
        throw new IOException(
            "No config file found for federate: " + federate.name + " at " + configSrc);
      }
      FileUtil.copyFile(configSrc, dst.resolve(federate.name + ".config"));

      // 3) Copy auth certificates
      if (!usePermanentDistKey) {
        if (!Files.isDirectory(authCertsRoot)) {
          throw new IOException("Missing auth_certs directory at " + authCertsRoot);
        }
        FileUtil.copyDirectory(authCertsRoot, dstCredentialsRoot, false);
      }

      // 4) Update the copied configs to the remote base.
      if (!useDocker) {
        updateConfigFile(
            dst.resolve(federate.name + ".config"),
            fileConfig.name + "/" + federate.name + "/sst/");
      }
    }

    // =========================
    // RTI
    // =========================
    Path rtiDst = fileConfig.getRtiSrcGenPath().resolve("sst");
    Files.createDirectories(rtiDst);

    // Keep credentials directory structure
    Path rtiCredentialsDst = rtiDst.resolve("credentials");
    Files.createDirectories(rtiCredentialsDst);

    // 1) Copy RTI private key (keep original relative path under credentials/)
    if (usePermanentDistKey) {
      String cipherKeySuffix = "rtiCipherKey.key";
      String macKeySuffix = "rtiMacKey.key";
      List<Path> cipherKeyMatches = FileUtil.globFilesEndsWith(keysRoot, cipherKeySuffix);
      List<Path> macKeyMatches = FileUtil.globFilesEndsWith(keysRoot, macKeySuffix);
      if (cipherKeyMatches.isEmpty() || macKeyMatches.isEmpty()) {
        throw new IOException("No key file found for RTI under " + keysRoot);
      }
      Path cipherKeyFile = cipherKeyMatches.get(0);
      Path cipherKeyRel = credentialsRoot.relativize(cipherKeyFile);
      Path cipherKeyDst = rtiCredentialsDst.resolve(cipherKeyRel);
      Files.createDirectories(cipherKeyDst.getParent());
      FileUtil.copyFile(cipherKeyFile, cipherKeyDst);

      Path macKeyFile = macKeyMatches.get(0);
      Path macKeyRel = credentialsRoot.relativize(macKeyFile);
      Path macKeyDst = rtiCredentialsDst.resolve(macKeyRel);
      FileUtil.copyFile(macKeyFile, macKeyDst);
      // The RSA private key is not needed in permanent distribution-key mode (see above).
    } else {
      String rtiKeySuffix = "rtiKey.pem";
      List<Path> rtiKeyMatches = FileUtil.globFilesEndsWith(keysRoot, rtiKeySuffix);
      if (rtiKeyMatches.isEmpty()) {
        throw new IOException(
            "No key file found for RTI (expected suffix: " + rtiKeySuffix + ") under " + keysRoot);
      }
      Path rtiKeyFile = rtiKeyMatches.get(0);

      // Preserve structure: credentials/keys/net1/<file>
      Path rtiKeyRel = credentialsRoot.relativize(rtiKeyFile); // keys/net1/Net1.rtiKey.pem
      Path rtiKeyDst = rtiCredentialsDst.resolve(rtiKeyRel); // rtiDst/credentials/keys/net1/...
      Files.createDirectories(rtiKeyDst.getParent());
      FileUtil.copyFile(rtiKeyFile, rtiKeyDst);
    }

    // 2) Copy RTI config
    Path rtiConfigSrc = configsRoot.resolve("rti.config");
    if (!Files.isRegularFile(rtiConfigSrc)) {
      throw new IOException("No rti.config found at " + rtiConfigSrc);
    }
    FileUtil.copyFile(rtiConfigSrc, rtiDst.resolve("rti.config"));

    // 3) Copy auth certificates
    if (!usePermanentDistKey) {
      if (!Files.isDirectory(authCertsRoot)) {
        throw new IOException("Missing auth_certs directory at " + authCertsRoot);
      }
      FileUtil.copyDirectory(authCertsRoot, rtiCredentialsDst, false);
    }

    // 4) Update the copied configs to the remote base.
    if (!useDocker) {
      updateConfigFile(rtiDst.resolve("rti.config"), getRelativeSSTRemoteBasePath("RTI"));
    }

    // Restrict the copied credential trees (they are archived with these permissions).
    KubernetesCredentialPlan.restrictTree(rtiDst);
    for (FederateInstance federate : federates) {
      KubernetesCredentialPlan.restrictTree(
          fileConfig.getSrcGenPath().resolve(federate.name).resolve("sst"));
    }
  }

  /**
   * Stage a minimal Auth tree for SCP/SSH deployment: the server jar, the properties of Auth
   * AUTH_ID (paths rewritten to the remote location), its database directory, and the trusted CA
   * certificate. The CA private key and the other Auth's files are not included.
   */
  private void stageAuthForRemote(Path dst) throws IOException {
    Path auth = fileConfig.getSSTAuthPath();
    int id = KubernetesCredentialPlan.AUTH_ID;
    String props = "exampleAuth" + id + ".properties";
    Files.createDirectories(dst.resolve("properties"));
    Files.createDirectories(dst.resolve("credentials").resolve("ca"));
    FileUtil.copyFile(
        auth.resolve("auth-server-jar-with-dependencies.jar"),
        dst.resolve("auth-server-jar-with-dependencies.jar"));
    FileUtil.copyFile(
        auth.resolve("properties").resolve(props), dst.resolve("properties").resolve(props));
    FileUtil.copyFile(
        auth.resolve("credentials").resolve("ca").resolve("CACert.pem"),
        dst.resolve("credentials").resolve("ca").resolve("CACert.pem"));
    FileUtil.copyDirectoryContents(
        auth.resolve("databases").resolve("auth" + id),
        dst.resolve("databases").resolve("auth" + id),
        false);
    updatePropertiesFile(dst.resolve("properties"), getRelativeRemoteBasePath("auth") + "/");
    KubernetesCredentialPlan.restrictTree(dst);
  }

  /**
   * Kubernetes: generate the Auth properties (paths pointing to the read-only Secret mount and to
   * the writable state volume) and the password input, then stage every component's files.
   */
  private void stageForKubernetes(List<FederateInstance> federates) throws IOException {
    var plan = KubernetesCredentialPlan.create(fileConfig, context.getTargetConfig(), federates);
    int id = KubernetesCredentialPlan.AUTH_ID;
    Path srcProps =
        fileConfig
            .getSSTAuthPath()
            .resolve("properties")
            .resolve("exampleAuth" + id + ".properties");
    String a = "Auth" + id;
    String secretDir = KubernetesCredentialPlan.AUTH_SECRET_MOUNT + "/";
    List<String> lines = new ArrayList<>();
    for (String line : Files.readAllLines(srcProps)) {
      int eq = line.indexOf('=');
      String key = eq < 0 ? "" : line.substring(0, eq);
      String value =
          switch (key) {
            case "entity_key_store_path" -> secretDir + a + "Entity.pfx";
            case "internet_key_store_path" -> secretDir + a + "Internet.pfx";
            case "database_key_store_path" -> secretDir + a + "Database.pfx";
            case "database_encryption_key_path" -> secretDir + a + "Database.bin";
            case "trusted_ca_cert_paths" -> secretDir + "CACert.pem";
            case "auth_database_dir" -> KubernetesCredentialPlan.AUTH_STATE_MOUNT + "/auth" + id;
            default -> null;
          };
      lines.add(value == null ? line : key + "=" + value);
    }
    String fed = fileConfig.name;
    for (var spec : plan.secrets()) {
      for (var item : spec.items()) {
        if (item.key().equals(KubernetesCredentialPlan.AUTH_PROPERTIES_KEY)) {
          KubernetesCredentialPlan.writePrivateFile(item.source(), String.join("\n", lines) + "\n");
        } else if (item.key().equals(KubernetesCredentialPlan.AUTH_PASSWORD_KEY)) {
          // The keystore password chosen by generateAll.sh (-p <program name>). Auth reads it from
          // standard input when no console is attached; the leading "y" answers its prompt.
          KubernetesCredentialPlan.writePrivateFile(item.source(), "y\n" + fed + "\n");
        }
      }
    }
    plan.stage();
  }

  private void updateConfigFile(Path fileToUpdate, String newBasePath) throws IOException {
    if (fileToUpdate == null) throw new IllegalArgumentException("fileToUpdate is null");
    if (newBasePath == null || newBasePath.isBlank())
      throw new IllegalArgumentException("newBasePath is null/blank");
    if (!Files.isRegularFile(fileToUpdate))
      throw new IOException("Config file not found: " + fileToUpdate);

    String base = newBasePath.replace("\\", "/");
    if (!base.endsWith("/")) base += "/";

    List<String> out = new ArrayList<>();
    boolean changed = false;

    for (String line : Files.readAllLines(fileToUpdate)) {
      String updated = line;

      if (line.startsWith("authInfo.pubkey.path=")) {
        updated = replaceConfigPathBaseKeepTail(line, "authInfo.pubkey.path=", base);
      } else if (line.startsWith("entityInfo.privkey.path=")) {
        updated = replaceConfigPathBaseKeepTail(line, "entityInfo.privkey.path=", base);
      } else if (line.startsWith("distKey.cipherkey.path=")) {
        updated = replaceConfigPathBaseKeepTail(line, "distKey.cipherkey.path=", base);
      } else if (line.startsWith("distkey.mackey.path=")) {
        updated = replaceConfigPathBaseKeepTail(line, "distkey.mackey.path=", base);
      }

      if (!updated.equals(line)) changed = true;
      out.add(updated);
    }

    if (changed) {
      FileUtil.writeToFile(String.join("\n", out) + "\n", fileToUpdate);
    }
  }

  private String replaceConfigPathBaseKeepTail(String fullLine, String keyPrefix, String base) {
    String value = fullLine.substring(keyPrefix.length()).trim().replace("\\", "/");

    int idx = value.indexOf("credentials/");
    if (idx < 0) {
      throw new IllegalArgumentException("Unexpected path format for line: " + fullLine);
    }

    String tail = value.substring(idx); // e.g. credentials/auth_certs/...
    return keyPrefix + base + tail;
  }

  /** Return the path to the RTI binary on the remote host. */
  public static String getRemoteBasePath(FederationFileConfig fileConfig, String entityName) {
    return "~/LinguaFrancaRemote/" + fileConfig.name + "/" + entityName;
  }

  /** Return the path to the RTI binary on the remote host. */
  public static String getSSTRemoteBasePath(FederationFileConfig fileConfig, String entityName) {
    return getRemoteBasePath(fileConfig, entityName) + "/sst/";
  }

  private String getRelativeRemoteBasePath(String entityName) {
    return "LinguaFrancaRemote/" + fileConfig.name + "/" + entityName;
  }

  private String getRelativeSSTRemoteBasePath(String entityName) {
    return "LinguaFrancaRemote/" + fileConfig.name + "/" + entityName + "/sst/";
  }
}
