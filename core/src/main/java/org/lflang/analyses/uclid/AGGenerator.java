package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.lflang.generator.GeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.docker.DockerGenerator;
import org.lflang.target.Target;

/**
 * A generator that produces C files for CBMC 
 */
public class AGGenerator extends GeneratorBase {

    private Path outputDir;
    protected CbmcGenerator cbmcGenerator;

    public AGGenerator(LFGeneratorContext context) {
        // Find all reaction bodies.
        super(context);
        cbmcGenerator = new CbmcGenerator(context);
    }

    public void doGenerate() {
        System.out.println("*** In CBMC doGenerate!");
        setupDirectories();

        // Generate C files.
        cbmcGenerator.doGenerate(this.targetConfig, this.mainDef);
    }

    @Override
    public TargetTypes getTargetTypes() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetTypes'");
    }

    @Override
    protected DockerGenerator getDockerGenerator(LFGeneratorContext context) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDockerGenerator'");
    }

    @Override
    public Target getTarget() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTarget'");
    }

    ////////////////////////////////////////////////////////////
    //// Private methods

    private void setupDirectories() {
        // Make sure the target directory exists.
        Path modelGenDir = context.getFileConfig().getModelGenPath();
        this.outputDir = Paths.get(modelGenDir.toString());
        try {
            Files.createDirectories(outputDir);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        System.out.println("The models will be located in: " + outputDir);
    }
}
