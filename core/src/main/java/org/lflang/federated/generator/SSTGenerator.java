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
      LFGeneratorContext context) {
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

    // Create graph used when creating credentials.
    // Set graph path.
    Path graphPath = fileConfig.getSSTGraphsPath().resolve(fileConfig.name + ".graph");
    // Generate the graph file content
    JsonObject graphObject = SSTGenerator.generateGraphFile(federates);
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

    // Set root path to execute commands.
    Path sstRepoRootPath = Paths.get(context.getTargetConfig().get(SSTPathProperty.INSTANCE));
    ProcessBuilder processBuilder = new ProcessBuilder();

    // Set the working directory to the specified path
    processBuilder.directory(sstRepoRootPath.resolve("examples").toFile());

    // Clean the old credentials & generate new credentials.
    // processBuilder.command("bash", "-c", "echo" + graphPath);

    processBuilder.command("bash", "-c", "./cleanAll.sh ; ./generateAll.sh -g " + graphPath);

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

      Thread errorThread =
          new Thread(
              () -> {
                try (BufferedReader reader =
                    new BufferedReader(new InputStreamReader(process.getErrorStream()))) {
                  String line;
                  while ((line = reader.readLine()) != null) {
                    context.getErrorReporter().nowhere().error("[SST Script Error] " + line);
                  }
                } catch (IOException e) {
                  e.printStackTrace();
                }
              });

      outputThread.start();
      errorThread.start();

      int exitCode = process.waitFor(); // Wait for process to finish
      outputThread.join();
      errorThread.join();

      if (exitCode == 0) {
        messageReporter.nowhere().info("Credential generation script execution succeeded.");
      } else {
        messageReporter.nowhere().error("Script execution failed with exit code: " + exitCode);
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
            "Federate generated SST config into: "
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
  }

  public static Path getSSTConfig(FederationFileConfig fileConfig, String name) {
    return fileConfig.getSSTConfigPath().resolve(name + ".config");
  }

  // TODO: FIX HERE!!!!!!!!!!!!!!!
  private static void generateSSTConfig(FederationFileConfig fileConfig, String name) {
    // Values to fill in
    String entityName = "net1." + name;
    String pubkeyRoot =
        fileConfig.getSSTCredentialsPath().resolve("auth_certs").toString()
            + File.separator
            + "Auth101EntityCert.pem";
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
        .append("entityInfo.purpose={\"group\":\"Servers\"}\n")
        .append("entityInfo.number_key=1\n")
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

  private static JsonObject generateGraphFile(List<FederateInstance> federateInstances) {
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
    JsonArray entityList = createEntityList(federateInstances);
    graphObject.add("entityList", entityList);

    // File sharing lists (empty for this example)
    graphObject.add("filesharingLists", new JsonArray());

    return graphObject;
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

  private static JsonArray createEntityList(List<FederateInstance> federateInstances) {
    JsonArray entityList = new JsonArray();

    // RTI entity
    JsonObject rti = createEntity("Servers", "net1.rti", "Net1.rti");
    // TODO: Make the two below work on future.
    rti.addProperty("port", 15045);
    rti.addProperty("host", "localhost");
    entityList.add(rti);

    // Federate entities
    for (FederateInstance federate : federateInstances) {
      String federateName = federate.name;
      JsonObject entity = createEntity("Clients", "net1." + federateName, "Net1." + federateName);
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

    FileUtil.copyDirectoryContents(source1, destination1, false);
    FileUtil.copyDirectoryContents(source2, destination2, false);
    FileUtil.copyDirectoryContents(source3, destination3, false);
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

  private static String updatePath(String line, String sstAuthPathStr) {
    return line.replace("../", sstAuthPathStr + "/");
  }
}
