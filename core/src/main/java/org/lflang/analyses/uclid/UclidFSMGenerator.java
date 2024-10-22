package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.Tag;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.target.TargetConfig;

public class UclidFSMGenerator {

    /** The main place to put generated code. */
    private CodeBuilder code = new CodeBuilder();

    /** LF Generator context */
    public final LFGeneratorContext context;

    /** The directory where the generated files are placed */
    public Path outputDir;

    /** A list of paths to the uclid files generated */
    public List<Path> generatedFiles = new ArrayList<>();

    /** The main reactor instance */
    public ReactorInstance main;

    public UclidFSMGenerator(LFGeneratorContext context) {
        this.context = context;
    }

    public void doGenerate(TargetConfig targetConfig, Instantiation mainDef, ReactorInstance main) {
        this.main = main;
        if (main == null) {
            throw new RuntimeException("No main reactor instance found.");
        }
        setupDirectories();
        generateStateSpace(this.main);
    }

    ////////////////////////////////////////////////////////////
    //// Private methods
    
    private void generateStateSpace(ReactorInstance main) {
        StateSpaceExplorer explorer = new StateSpaceExplorer(main);
        explorer.explore(
            new Tag(0, 0, true), true // findLoop
            );
        StateSpaceDiagram diagram = explorer.diagram;
        diagram.display();

        // FIXME: reactionsInvoked is currently a set. We need to sort
        // the reactions by their levels and "execute" in a Uclid
        // procedure based on their levels, effectively a linearization
        // of the partial order for all logically simultaneous reactions.

        // Generate a dot file.
        try {
            CodeBuilder dot = diagram.generateDot();
            Path file = this.outputDir.resolve("state_space.dot");
            String filename = file.toString();
            dot.writeToFile(filename);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void setupDirectories() {
        // Make sure the target directory exists.
        Path cbmcModelGenDir = context.getFileConfig().getModelGenPath().resolve("uclid");
        this.outputDir = Paths.get(cbmcModelGenDir.toString());
        try {
            Files.createDirectories(outputDir);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        System.out.println("The models will be located in: " + outputDir);
    }
    
}
