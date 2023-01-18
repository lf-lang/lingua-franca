package org.lflang.federated.generator;

import java.io.IOException;
import java.util.List;

import org.lflang.FileConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DockerData;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext;

public class FedDockerGenerator extends DockerGeneratorBase {

    /**
     * A docker file will be generated for each lingua franca module.
     * This maps the name of the LF module to the data related to the docker
     * file for that module.
     */
    protected List<DockerData> dockerDataList;


    /**
     * The host on which to run the rti.
     */
    private String rtiHost;


    /**
     * The constructor for the base docker file generation class.
     *
     * @param context
     * @param rtiHost
     */
    public FedDockerGenerator(LFGeneratorContext context, String rtiHost) {
        super(context);
        this.rtiHost = rtiHost;
    }

    /**
     * Add a file to the list of docker files to generate.
     */
    public void add(DockerData data) {
        dockerDataList.add(data);
    }


    @Override
    public DockerData generateDockerData() {
        return null;
    }

//    /**
//     * Write the docker files generated for the federates added using `addFederate` so far.
//     */
//    @Override
//    public void writeDockerFiles() throws IOException {
//        var svcs = new StringBuilder();
//        for (DockerData dockerData : dockerDataList) {
//            writeDockerFile(dockerData);
//            svcs.append(dockerData.getServiceDescription(true));
//            //System.out.println(getDockerBuildCommandMsg(dockerData));
//        }
//        // FIXME: figure out what to print
//        //System.out.println(getDockerComposeUpMsg());
//        writeDockerComposeFile(svcs.toString(), "lf");
//    }

    /**
     * Override in FedGenerator
     * @param services
     *
     * @return
     */
    @Override
    protected String generateDockerServices(List<DockerData> services) {
        return String.join("\n",
                    super.generateDockerServices(services),
                                "    rti:",
                    "        image: lflang/rti:rti",
                    "        hostname: " + this.rtiHost,
                    "        command: -i 1 -n " + services.size());
    }

    /**
     * Append the RTI to the "services" section of the docker-compose.yml file
     * and write the docker-compose.yml file.
     * @param services Section that lists all the services.
     * @param networkName The name of the network to which docker will connect the containers.
     * @throws IOException
     */
//    @Override
//    protected void writeDockerComposeFile(
//        String services,
//        String networkName
//    ) throws IOException {
//        var sb = new StringBuilder(services);
//        var tab = " ".repeat(4);
//        sb.append(tab).append("rti:\n");
//        sb.append(tab.repeat(2)).append("image: lflang/rti:rti\n");
//        sb.append(tab.repeat(2)).append("hostname: ").append(this.rtiHost).append("\n");
//        sb.append(tab.repeat(2)).append("command: -i 1 -n ").append(dockerDataList.size()).append("\n");
//        super.writeDockerComposeFile(sb.toString(), networkName);
//    }
}
