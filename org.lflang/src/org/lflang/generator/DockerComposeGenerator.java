package org.lflang.generator;

import java.io.File;
import java.io.IOException;
import org.eclipse.xtext.xbase.lib.Exceptions;

/**
 * Generates the content of the docker-compose.yml file 
 * used to deploy containerized LF programs.
 * 
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class DockerComposeGenerator {
    /**
     * Write the docker-compose.yml for orchestrating the federates.
     * @param the directory to write the docker-compose.yml
     * @param content of the "services" section of the docker-compose.yml
     * @param the name of the network hosting the federation
     */
    public static void writeFederatesDockerComposeFile(
        File dir, 
        StringBuilder dockerComposeServices, 
        String networkName
    ) {
        var dockerComposeFileName = "docker-compose.yml";
        var dockerComposeFile = dir + File.separator + dockerComposeFileName;
        var contents = new CodeBuilder();
        contents.pr(String.join("\n", 
            "version: \"3.9\"",
            "services:",
            dockerComposeServices.toString(),
            "networks:",
            "    lingua-franca:",
            "        name: "+networkName
        ));
        try {
            contents.writeToFile(dockerComposeFile);
        } catch (IOException e) {
            throw Exceptions.sneakyThrow(e);
        }
    }

    /**
     * Append a service to the "services" section of the docker-compose.yml file.
     * @param the content of the "services" section of the docker-compose.yml file.
     * @param the name of the federate to be added to "services".
     * @param the name of the federate's Dockerfile.
     */
    public static void appendFederateToDockerComposeServices(StringBuilder dockerComposeServices, String federateName, String context, String dockerFileName) {
        var tab = " ".repeat(4);
        dockerComposeServices.append(tab+federateName+":\n");
        dockerComposeServices.append(tab+tab+"build:\n");
        dockerComposeServices.append(tab+tab+tab+"context: "+context+"\n");
        dockerComposeServices.append(tab+tab+tab+"dockerfile: "+dockerFileName+"\n");
        dockerComposeServices.append(tab+tab+"command: -i 1\n");
    }

    /**
     * Append the RTI to the "services" section of the docker-compose.yml file.
     * @param the content of the "services" section of the docker-compose.yml file.
     * @param the name given to the RTI in the "services" section.
     * @param the tag of the RTI's image.
     * @param the number of federates.
     */
    public static void appendRtiToDockerComposeServices(StringBuilder dockerComposeServices, String dockerImageName, String hostName, int n) {
        var tab = " ".repeat(4);
        dockerComposeServices.append(tab+"rti:\n");
        dockerComposeServices.append(tab+tab+"image: "+dockerImageName+"\n");
        dockerComposeServices.append(tab+tab+"hostname: "+hostName+"\n");
        dockerComposeServices.append(tab+tab+"command: -i 1 -n "+n+"\n");
    }
}
