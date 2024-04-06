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
  public static void generateConfig(FederationFileConfig fileConfig, FederateInstance federate){
    // Path to the skeleton file
    String skeletonFilePath = fileConfig.getSrcGenPath().resolve(federate.name) + "/core/federated/network/SSTfederateskeleton.config";
    // Values to fill in
    String name = federate.name; // Change this to the desired name
    String rootPath = ;
    String pubkeyRoot = rootPath + "../iotauth/entity/auth_certs/Auth101EntityCert.pem";
    String privkeyRoot = rootPath + "../iotauth/entity/credentials/keys/net1/Net1." + name + "Key.pem";

    try {
        // Read the skeleton file
        BufferedReader reader = new BufferedReader(new FileReader(skeletonFilePath));
        StringBuilder stringBuilder = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            stringBuilder.append(line).append(System.lineSeparator());
        }
        reader.close();
      
        // Modify the content with the provided values
        String modifiedContent = stringBuilder.toString()
                .replace("entityInfo.name=net1.", "entityInfo.name=net1." + name)
                .replace("authInfo.pubkey.path=", "authInfo.pubkey.path=" + pubkeyRoot)
                .replace("entityInfo.privkey.path=", "entityInfo.privkey.path=" + privkeyRoot);
      
        // Write the modified content to a new file
        String newFilePath = fileConfig.getSrcGenPath().resolve(federate.name) + "/core/federated/network/" + federate.name + ".config";
        // String newFilePath = targetDirectoryPath + "generated_config_" + name + ".config";
        BufferedWriter writer = new BufferedWriter(new FileWriter(newFilePath));
        writer.write(modifiedContent);
        writer.close();
      
        System.out.println("File generated successfully at: " + newFilePath);
    } catch (IOException e) {
        e.printStackTrace();
    }
  }
  
}
