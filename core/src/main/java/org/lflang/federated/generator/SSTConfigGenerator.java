package org.lflang.federated.generator;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class SSTConfigGenerator {
  public static void generateSSTConfig(FederationFileConfig fileConfig, String name) {
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
      System.out.println("SST config file generated successfully at: " + name);
      Path newFilePath;
      newFilePath = fileConfig.getSSTConfigPath().resolve(name + ".config");
      // Create /SST directories if necessary
      Files.createDirectories(newFilePath.getParent().getParent()); 
      // Create /SST/configs directories if necessary
      Files.createDirectories(newFilePath.getParent()); // Create parent directories if necessary
      BufferedWriter writer = new BufferedWriter(new FileWriter(newFilePath.toFile(), false));
      writer.write(configContent.toString());
      writer.close();

      System.out.println("SST config file generated successfully at: " + newFilePath);
    } catch (IOException e) {
      System.out.println("Config generation failed.");
      e.printStackTrace();
    }
  }

  public static Path getSSTConfig(FederationFileConfig fileConfig, String name) {
    return fileConfig.getSSTConfigPath().resolve(name + ".config"); 
  }
}
