package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.lflang.ast.ASTUtils;
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
    protected UclidFSMGenerator uclidFSMGenerator;

    public AGGenerator(LFGeneratorContext context) {
        
        // Find all reaction bodies.
        super(context);

        // Reuse parts of doGenerate() from GeneratorBase.
        super.printInfo(context);
        ASTUtils.setMainName(context.getFileConfig().resource, context.getFileConfig().name);
        super.createMainInstantiation();
        super.setReactorsAndInstantiationGraph(context.getMode());
        
        // Create the main reactor instance if there is a main reactor.
        this.main =
            ASTUtils.createMainReactorInstance(mainDef, reactors, messageReporter, targetConfig);
        
        cbmcGenerator = new CbmcGenerator(context);
        uclidFSMGenerator = new UclidFSMGenerator(context);
    }

    public void doGenerate() {
        System.out.println("*** In CBMC doGenerate!");
        setupDirectories();

        // Generate C and UCLID5 files.
        cbmcGenerator.doGenerate(this.targetConfig);
        uclidFSMGenerator.doGenerate(this.targetConfig, this.mainDef, this.main);
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
