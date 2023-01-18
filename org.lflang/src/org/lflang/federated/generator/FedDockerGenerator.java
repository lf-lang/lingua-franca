package org.lflang.federated.generator;

import java.io.IOException;
import java.util.List;

import org.lflang.generator.DockerData;
import org.lflang.generator.DockerGeneratorBase;
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
    protected String generateDockerFileContent() {
        return null;
    }

    @Override
    public DockerData generateDockerData() {
        return null;
    }

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
                    "        command: -i 1 -n " + services.size(),
                    "        container_name: " + context.getFileConfig().name + "-" + "rti");
    }
}
