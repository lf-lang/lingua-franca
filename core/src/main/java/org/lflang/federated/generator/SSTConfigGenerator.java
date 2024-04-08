package org.lflang.federated.generator;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class SSTConfigGenerator {
  private static void generateConfig(FederationFileConfig fileConfig, String name, boolean isRTI) {
    String rootPath = fileConfig.getGenPath() + "/../../../../";
    // Path to the skeleton file
    String skeletonFilePath = rootPath
        + "core/src/main/resources/lib/c/reactor-c/core/federated/network/SSTskeleton.config";
    // Values to fill in
    String pubkeyRoot = rootPath + "../iotauth/entity/auth_certs/Auth101EntityCert.pem";
    String privkeyRoot = rootPath + "../iotauth/entity/credentials/keys/net1/Net1." + name + "Key.pem";

    try {
      // Read the skeleton file
      Path skeletonPath = Paths.get(skeletonFilePath);
      String modifiedContent = Files.readString(skeletonPath);

      // Modify the content with the provided values
      modifiedContent = modifiedContent.replace("entityInfo.name=net1.", "entityInfo.name=net1." + name)
          .replace("authInfo.pubkey.path=", "authInfo.pubkey.path=" + pubkeyRoot)
          .replace("entityInfo.privkey.path=", "entityInfo.privkey.path=" + privkeyRoot);

      // Create the new file and write the modified content
      Path newFilePath;
      if (!isRTI) {
        newFilePath = Paths.get(fileConfig.getSrcGenPath().resolve(name) + "/core/federated/network/" + name + ".config");
      } else {
        newFilePath = Paths.get(fileConfig.getSrcGenPath() + "/" + name + ".config");
      }
      Files.createDirectories(newFilePath.getParent()); // Create parent directories if necessary
      // Create a config file if it does not exists. If the config already exists, overwrite the file.
      BufferedWriter writer = new BufferedWriter(new FileWriter(newFilePath.toFile(), false));
      writer.write(modifiedContent);
      writer.close();

      System.out.println("SST config file generated successfully at: " + newFilePath);
    } catch (IOException e) {
      System.out.println("Config generation failed.");
      e.printStackTrace();
    }
  }
  public static void generateFederateConfig(FederationFileConfig fileConfig, FederateInstance federate) {
    generateConfig(fileConfig, federate.name, false);
  }

  public static void generateRTIConfig(FederationFileConfig fileConfig) {
    generateConfig(fileConfig, "rti", true);
  }

  public static String getFederateConfigPath(FederationFileConfig fileConfig, FederateInstance federate) {
    return Paths.get(fileConfig.getSrcGenPath().resolve(federate.name) + "/core/federated/network/" + federate.name + ".config").toString();
  }

  public static String getRTIConfigPath(FederationFileConfig fileConfig) {
    return Paths.get(fileConfig.getSrcGenPath() + "/" + "rti.config").toString();
  }

}
