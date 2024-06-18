package org.lflang.federated.generator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.SSTPathProperty;
import org.lflang.util.FileUtil;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

/**
 * SST related methods.
 *
 * @author Dongha Kim
 */

public class SSTGenerator {
  public static void setupSST(FederationFileConfig fileConfig, List<FederateInstance> federates, MessageReporter messageReporter, LFGeneratorContext context) {
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTConfigPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTCredentialsPath().toFile());
    FileUtil.createDirectoryIfDoesNotExist(fileConfig.getSSTDatabasesPath().toFile());
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
      e.printStackTrace();
      System.err.println("Failed to write graph file.");
    }

    
    // Set root path to execute commands.
    String sstRootPath = context.getTargetConfig().get(SSTPathProperty.INSTANCE);
    ProcessBuilder processBuilder = new ProcessBuilder();
    
    // Set the working directory to the specified path
    processBuilder.directory(new File(sstRootPath + File.separator + "examples"));
    
    // Clean the old credentials & generate new credentials.
    processBuilder.command("bash", "-c", "./cleanAll.sh ; ./generateAll.sh -g " + graphPath);
    
    // processBuilder.directory(new File(sstRootPath + File.separator + "auth" +
    // File.separator + "auth-server"));
    
    // String javaCommand = "java";
    // String jarFile = "target/auth-server-jar-with-dependencies.jar";
    // String propertiesFile = "../properties/exampleAuth101.properties";
    
    // // Construct the command and its arguments as separate elements
    // processBuilder.command("bash", "-c", javaCommand + " -jar " + jarFile + " -p
    // " + propertiesFile);
    
    // Start the process
    try {
      Process process = processBuilder.start();
      
      // Wait for the process to complete
      int exitCode = process.waitFor();
      
      // Output the result
      if (exitCode == 0) {
        System.out.println("Script executed successfully.");
      } else {
        System.out.println("Script execution failed with exit code: " + exitCode);
        // Optionally, you can read the error stream to see the script output
        String errorOutput = new String(process.getErrorStream().readAllBytes());
        System.out.println("Error Output: " + errorOutput);
      }
    } catch (IOException | InterruptedException e) {
      e.printStackTrace();
      System.err.println("An error occurred while executing the script.");
    }
    
    // Generate SST config for the rti.
    SSTGenerator.generateSSTConfig(fileConfig, "rti");
    messageReporter
        .nowhere()
        .info("Federate generated SST config into: " + SSTGenerator.getSSTConfig(fileConfig, "rti").toString());

    // Generate SST config for the federates.
    for (FederateInstance federate : federates) {
      SSTGenerator.generateSSTConfig(fileConfig, federate.name);
      messageReporter
          .nowhere()
          .info(
              "Federate generated SST config into: " + SSTGenerator.getSSTConfig(fileConfig, federate.name).toString());
    }
  }

  public static Path getSSTConfig(FederationFileConfig fileConfig, String name) {
    return fileConfig.getSSTConfigPath().resolve(name + ".config");
  }

  private static void generateSSTConfig(FederationFileConfig fileConfig, String name) {
    String rootPath = fileConfig.getGenPath() + "/../../../../";
    // Values to fill in
    String entityName = "net1." + name;
    String pubkeyRoot = rootPath + "../iotauth/entity/auth_certs/Auth101EntityCert.pem";
    String privkeyRoot = rootPath + "../iotauth/entity/credentials/keys/net1/Net1." + name + "Key.pem";
    String authIpAddress = "127.0.0.1";
    int authPortNumber = 21900;
    String entityServerIpAddress = "127.0.0.1";
    int entityServerPortNumber = 15045;
    String networkProtocol = "TCP";

    // Create the configuration content
    StringBuilder configContent = new StringBuilder();
    configContent.append("entityInfo.name=").append(entityName).append("\n")
        .append("entityInfo.purpose={\"group\":\"Servers\"}\n")
        .append("entityInfo.number_key=1\n")
        .append("authInfo.pubkey.path=").append(pubkeyRoot).append("\n")
        .append("entityInfo.privkey.path=").append(privkeyRoot).append("\n")
        .append("auth.ip.address=").append(authIpAddress).append("\n")
        .append("auth.port.number=").append(authPortNumber).append("\n")
        .append("entity.server.ip.address=").append(entityServerIpAddress).append("\n")
        .append("entity.server.port.number=").append(entityServerPortNumber).append("\n")
        .append("network.protocol=").append(networkProtocol).append("\n");

    try {
      // Create the new file and write the modified content
      Path newFilePath;
      newFilePath = fileConfig.getSSTConfigPath().resolve(name + ".config");
      System.out.println("11111111111111111111111111111111111111" + newFilePath);
      // Create /SST directories if necessary
      Files.createDirectories(newFilePath.getParent().getParent());
      // Create /SST/configs directories if necessary
      Files.createDirectories(newFilePath.getParent()); // Create parent directories if necessary
      BufferedWriter writer = new BufferedWriter(new FileWriter(newFilePath.toFile(), false));
      writer.write(configContent.toString());
      writer.close();
    } catch (IOException e) {
      System.out.println("Config generation failed.");
      e.printStackTrace();
    }
  }


  private static JsonObject generateGraphFile(List<FederateInstance> federateInstances) {
    JsonObject graphObject = new JsonObject();

    // Auth list
    JsonArray authList = new JsonArray();
    authList.add(createAuthEntry(101, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
    authList.add(createAuthEntry(102, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
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

  private static JsonObject createAuthEntry(int id, String entityHost, String authHost, int tcpPort, int udpPort,
      int authPort, int callbackPort, int dbProtectionMethod,
      boolean backupEnabled, boolean contextualCallbackEnabled) {
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
    entity.add("backupToAuthIds", new JsonArray()); // Empty array for backupToAuthIds
    return entity;
  }
}
