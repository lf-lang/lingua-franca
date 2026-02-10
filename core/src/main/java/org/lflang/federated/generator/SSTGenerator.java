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
import org.lflang.target.property.SSTPathProperty;
import org.lflang.util.FileUtil;

/**
 * SST related methods.
 *
 * @author Dongha Kim
 */
public class SSTGenerator {
  public static void setupSST(
      FederationFileConfig fileConfig,
      List<FederateInstance> federates,
      MessageReporter messageReporter,
      LFGeneratorContext context,
      RtiConfig rtiConfig) throws IOException {
    if (context.getTargetConfig().get(SSTPathProperty.INSTANCE).isEmpty()) {
      context
          .getErrorReporter()
          .nowhere()
          .error(
              "Target property `sst-root-path:` has not been defined. `comm-type: SST` requires"
                  + " `sst-root-path`");
      return;
    }

    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTConfigPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTCredentialsPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTGraphsPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTPolicyPath().toFile());

    // Create graph used when creating credentials.
    // Set graph path.
    Path graphPath = fileConfig.getSSTGraphsPath().resolve(fileConfig.name + ".graph");
    // Generate the graph file content
    JsonObject graphObject = SSTGenerator.generateGraphFile(federates, rtiConfig);
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
    Path sstRepoRootPath = Paths.get(context.getTargetConfig().get(SSTPathProperty.INSTANCE));
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
      Process mvnProcess = mvnProcessBuilder.start();

      // Create threads to capture output and error streams
      Thread mvnOutputThread = new Thread(
          () -> {
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(mvnProcess.getInputStream()))) {
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
      SSTGenerator.copyCredentials(fileConfig, sstRepoRootPath);
      messageReporter
          .nowhere()
          .info("Credentials copied into: " + fileConfig.getSSTCredentialsPath().toString());
      SSTGenerator.copyAuthNecessary(fileConfig, sstRepoRootPath);
      messageReporter
          .nowhere()
          .info("Auth necessary files copied into: " + fileConfig.getSSTAuthPath().toString());
      SSTGenerator.updatePropertiesFile(fileConfig);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Generate SST config for the rti.
    SSTGenerator.generateSSTConfig(fileConfig, "rti");
    messageReporter
        .nowhere()
        .info(
            "Generated RTI's SST config into: "
                + SSTGenerator.getSSTConfig(fileConfig, "rti").toString());

    // Generate SST config for the federates.
    for (FederateInstance federate : federates) {
      SSTGenerator.generateSSTConfig(fileConfig, federate.name);
      messageReporter
          .nowhere()
          .info(
              "Federate generated SST config into: "
                  + SSTGenerator.getSSTConfig(fileConfig, federate.name).toString());
    }

    // Copy the configs and credentials of rti and federates, to the src-gen for tar deployments.
    SSTGenerator.copyAuthAndConfigsAndKeys(fileConfig, federates);
  }

  public static Path getSSTConfig(FederationFileConfig fileConfig, String name) {
    return fileConfig.getSSTConfigPath().resolve(name + ".config");
  }

  private static void generateSSTConfig(FederationFileConfig fileConfig, String name) {
    // Values to fill in
    String entityName = "net1." + name;
    int authID = 101;
    String encryptionMode = "AES_128_CBC";
    int hmacMode = 1;
    String pubkeyRoot =
        fileConfig.getSSTCredentialsPath().resolve("auth_certs").toString()
            + File.separator
            + "Auth"
            + authID
            + "EntityCert.pem";
    String privkeyRoot =
        fileConfig.getSSTCredentialsPath().resolve("keys").resolve("net1").toString()
            + File.separator
            + "Net1."
            + name
            + "Key.pem";
    String authIpAddress = "127.0.0.1";
    int authPortNumber = 21900;
    String entityServerIpAddress = "127.0.0.1";
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
        .append("encryptionMode=")
        .append(encryptionMode)
        .append("\n")
        .append("HmacMode=")
        .append(hmacMode)
        .append("\n")
        .append("authInfo.pubkey.path=")
        .append(pubkeyRoot)
        .append("\n")
        .append("entityInfo.privkey.path=")
        .append(privkeyRoot)
        .append("\n")
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

  private static JsonObject generateGraphFile(
      List<FederateInstance> federateInstances, RtiConfig rtiConfig) {
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
    JsonArray entityList = createEntityList(federateInstances, rtiConfig);
    graphObject.add("entityList", entityList);

    // File sharing lists (empty for this example)
    graphObject.add("filesharingLists", new JsonArray());

    return graphObject;
  }

  private static JsonObject createGroupPolicy(
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
    return o;
  }

    // Creates the policy JSON array to be passed to authConfigGenerator.js via --policy <file>.
  private static JsonArray generateCommunicationPolicy() {
    JsonArray policies = new JsonArray();

    policies.add(createGroupPolicy(
        "Federates",
        "Group",
        "RTI",
        2,
        "AES-128-CBC:SHA256",
        "1*day",
        "2*hour"));

    policies.add(createGroupPolicy(
        "Federates",
        "Group",
        "Federates",
        2,
        "AES-128-CBC:SHA256",
        "1*day",
        "2*hour"));

    return policies;
  }

  private static JsonObject createAuthEntry(
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

  private static JsonArray createEntityList(
      List<FederateInstance> federateInstances, RtiConfig rtiConfig) {
    JsonArray entityList = new JsonArray();

    // RTI entity
    JsonObject rti = createEntity("RTI", "net1.rti", "Net1.rti");
    rti.addProperty("port", rtiConfig.getPort());
    rti.addProperty("host", rtiConfig.getHost());
    entityList.add(rti);

    // Federate entities
    for (FederateInstance federate : federateInstances) {
      String federateName = federate.name;
      JsonObject entity = createEntity("Federates", "net1." + federateName, "Net1." + federateName);
      entityList.add(entity);
    }
    return entityList;
  }

  private static JsonObject createEntity(String group, String name, String credentialPrefix) {
    JsonObject entity = new JsonObject();
    entity.addProperty("group", group);
    entity.addProperty("name", name);
    entity.addProperty("distProtocol", "TCP");
    entity.addProperty("usePermanentDistKey", false);
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

  private static void copyCredentials(FederationFileConfig fileConfig, Path sstRepoRootPath)
      throws IOException {
    // Copy auth_certs.
    Path source1 = sstRepoRootPath.resolve("entity").resolve("auth_certs");
    Path destination1 = fileConfig.getSSTCredentialsPath().resolve("auth_certs");

    // Copy keys.
    Path source2 = sstRepoRootPath.resolve("entity").resolve("credentials").resolve("keys");
    Path destination2 = fileConfig.getSSTCredentialsPath().resolve("keys");
    FileUtil.copyDirectoryContents(source1, destination1, false);
    FileUtil.copyDirectoryContents(source2, destination2, false);
  }

  private static void copyAuthNecessary(FederationFileConfig fileConfig, Path sstRepoRootPath)
      throws IOException {
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
    Path source4 = sstRepoRootPath.resolve("auth").resolve("auth-server").resolve("target").resolve("auth-server-jar-with-dependencies.jar");
    Path destination4 = fileConfig.getSSTAuthPath().resolve("auth-server-jar-with-dependencies.jar");

    FileUtil.copyDirectoryContents(source1, destination1, false);
    FileUtil.copyDirectoryContents(source2, destination2, false);
    FileUtil.copyDirectoryContents(source3, destination3, false);
    FileUtil.copyFile(source4, destination4);
  }

  private static void updatePropertiesFile(FederationFileConfig fileConfig) throws IOException {
    File file =
        Paths.get(
                fileConfig.getSSTAuthPath().resolve("properties").toString(),
                "exampleAuth101.properties")
            .toFile();
    List<String> updatedLines = new ArrayList<>();
    String sstAuthPathStr = fileConfig.getSSTAuthPath().toString();

    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = reader.readLine()) != null) {
        if (line.startsWith("entity_key_store_path=")) {
          line = updatePath(line, sstAuthPathStr);
        } else if (line.startsWith("internet_key_store_path=")) {
          line = updatePath(line, sstAuthPathStr);
        } else if (line.startsWith("database_key_store_path=")) {
          line = updatePath(line, sstAuthPathStr);
        } else if (line.startsWith("database_encryption_key_path=")) {
          line = updatePath(line, sstAuthPathStr);
        } else if (line.startsWith("trusted_ca_cert_paths=")) {
          line = updatePath(line, sstAuthPathStr);
        } else if (line.startsWith("auth_database_dir=")) {
          line = updatePath(line, sstAuthPathStr);
        }
        updatedLines.add(line);
      }
    }

    // Write the updated lines back to the file
    try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
      for (String updatedLine : updatedLines) {
        writer.write(updatedLine);
        writer.newLine();
      }
    }
  }

  private static void copyAuthAndConfigsAndKeys(FederationFileConfig fileConfig, List<FederateInstance> federates)
      throws IOException {
    // 1. Copy Auth to RTI directory.
    Path auth_src = fileConfig.getSSTAuthPath();
    Path rti_src = fileConfig.getRtiSrcGenPath().resolve("auth");
    FileUtil.copyDirectoryContents(auth_src, rti_src, false);

    Path keysRoot = fileConfig.getSSTCredentialsPath().resolve("keys");
    Path configsRoot = fileConfig.getSSTConfigPath();
    Path authCertsRoot = fileConfig.getSSTCredentialsPath().resolve("auth_certs");

    // 2. Copy Configs and Keys to src-gen of federates and RTIs.
    // =========================
    // Federates
    // =========================
    for (FederateInstance federate : federates) {
      Path dst = fileConfig.getSrcGenPath()
          .resolve(federate.name)
          .resolve("sst");
      Files.createDirectories(dst);

      // 1) Copy private key
      String keySuffix = federate.name + "Key.pem";
      List<Path> keyMatches = FileUtil.globFilesEndsWith(keysRoot, keySuffix);
      if (keyMatches.isEmpty()) {
        throw new IOException(
            "No key file found for federate: " + federate.name
                + " (expected suffix: " + keySuffix + ") under " + keysRoot);
      }
      Path keyFile = keyMatches.get(0);
      FileUtil.copyFile(keyFile, dst.resolve(keyFile.getFileName()));

      // 2) Copy config
      Path configSrc = configsRoot.resolve(federate.name + ".config");
      if (!Files.isRegularFile(configSrc)) {
        throw new IOException(
            "No config file found for federate: " + federate.name
                + " at " + configSrc);
      }
      FileUtil.copyFile(configSrc, dst.resolve(federate.name + ".config"));

      // 3) Copy auth certificates
      if (!Files.isDirectory(authCertsRoot)) {
        throw new IOException("Missing auth_certs directory at " + authCertsRoot);
      }
      FileUtil.copyDirectoryContents(authCertsRoot, dst, false);
    }

    // =========================
    // RTI
    // =========================
    Path rtiDst = fileConfig.getRtiSrcGenPath().resolve("sst");
    Files.createDirectories(rtiDst);

    // 1) Copy RTI private key
    String rtiKeySuffix = "rtiKey.pem";
    List<Path> rtiKeyMatches = FileUtil.globFilesEndsWith(keysRoot, rtiKeySuffix);
    if (rtiKeyMatches.isEmpty()) {
      throw new IOException(
          "No key file found for RTI (expected suffix: " + rtiKeySuffix + ") under " + keysRoot);
    }
    Path rtiKeyFile = rtiKeyMatches.get(0);
    FileUtil.copyFile(rtiKeyFile, rtiDst.resolve(rtiKeyFile.getFileName()));

    // 2) Copy RTI config
    Path rtiConfigSrc = configsRoot.resolve("rti.config");
    if (!Files.isRegularFile(rtiConfigSrc)) {
      throw new IOException("No rti.config found at " + rtiConfigSrc);
    }
    FileUtil.copyFile(rtiConfigSrc, rtiDst.resolve("rti.config"));

    // 3) Copy auth certificates to RTI
    if (!Files.isDirectory(authCertsRoot)) {
      throw new IOException("Missing auth_certs directory at " + authCertsRoot);
    }
    FileUtil.copyDirectoryContents(authCertsRoot, rtiDst, false);
  }

  private static String updatePath(String line, String sstAuthPathStr) {
    return line.replace("../", sstAuthPathStr + "/");
  }


  /** Return the path to the RTI binary on the remote host. */
  public static String getSSTRemoteBasePath(FederationFileConfig fileConfig, String entityName) {
    return "~/LinguaFrancaRemote/" + fileConfig.name + "/" + entityName + "/sst/";
  }
}
