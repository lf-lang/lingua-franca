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
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.target.TargetConfig;

public class UclidFSMGenerator {

    /** The main place to put generated code. */
    private CodeBuilder code;

    /** LF Generator context */
    public final LFGeneratorContext context;

    /** The target configuration */
    public TargetConfig targetConfig;

    /** The directory where the generated files are placed */
    public Path outputDir;

    /** The directory where the generated models are placed */
    public Path modGenDir;

    /** A list of paths to the uclid files generated */
    public List<Path> generatedFiles = new ArrayList<>();

    /** The main reactor instance */
    public ReactorInstance main;

    /** A list of reactors in the LF program */
    public List<Reactor> reactors = new ArrayList<>();

    /** A list of reactions in the LF program */
    public List<Reaction> reactions = new ArrayList<>();

    public UclidFSMGenerator(LFGeneratorContext context) {
        this.context = context;
        this.modGenDir = context.getFileConfig().getModelGenPath();
        this.targetConfig = context.getTargetConfig();
        this.reactors = ASTUtils.getAllReactors(targetConfig.getMainResource());
        this.reactions = this.reactors.stream().map(it -> it.getReactions()).flatMap(List::stream).toList();
    }

    public void doGenerate(TargetConfig targetConfig, Instantiation mainDef, ReactorInstance main) {
        this.main = main;
        if (main == null) {
            throw new RuntimeException("No main reactor instance found.");
        }
        setupDirectories();
        generateStateSpace(this.main);
        generateUclidFile();
    }

    ////////////////////////////////////////////////////////////
    //// Private methods
    
    private void generatePythonMainFunction() {
        code.pr(String.join("\n",
            "if __name__ == \"__main__\":",
            "    m = MainModule()",
            "    ucl_module = m.buildUclidModule()",
            "    print(ucl_module.__inject__())"
        ));
    }

    private void generateUclidMainModule() {
        code.pr("class MainModule(ModuleWithExternalProcedures):");
        code.indent();
        generateInitFunction();
        code.unindent();
    }

    private void generateInitFunction() {
        code.pr("def __init__(self, delete=True, verbose=False):");
        code.indent();
        code.pr("super().__init__(\"main\", delete, verbose)");
        
        /**
         * Example:
         * pedestrian_reaction_1 = ExternalProcedure(
         *     name="pedestrian_reaction_1",
         *     lang=Lang.C,
         *     filepath="test/c/traffic_light/pedestrian_reaction_1.c",
         *     jsonpath="test/json/traffic_light/pedestrian_reaction_1.json",
         * )
         */
        for (Reactor reactorDef : this.reactors) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (int index = 0; index < reactionDefs.size(); index++) {
                Reaction reactionDef = reactionDefs.get(index);
                String reactionName = reactorDef.getName() + "_reaction_" + index;
                code.pr(reactionName + " = ExternalProcedure(");
                code.indent();
                code.pr("name=\"" + reactionName + "\",");
                code.pr("lang=Lang.C,");
                code.pr("filepath=\"" + this.modGenDir + "/c/" + reactionName + ".c\",");
                code.pr("jsonpath=\"" + this.modGenDir + "/json/" + reactionName + ".json\"");
                code.unindent();
                code.pr(")");
            }
        }

        /**
         * Example:
         * self.ext_procs = {
         *     pedestrian_reaction_1.name: pedestrian_reaction_1,
         *     traffic_light_reaction_1.name: traffic_light_reaction_1,
         *     traffic_light_reaction_2.name: traffic_light_reaction_2,
         *     traffic_light_reaction_3.name: traffic_light_reaction_3,
         * }
         */
        code.pr("self.ext_procs = {");
        code.indent();
        for (Reactor reactorDef : this.reactors) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (int index = 0; index < reactionDefs.size(); index++) {
                String reactionName = reactorDef.getName() + "_reaction_" + index;
                code.pr(reactionName + ".name: " + reactionName + ",");
            }
        }

        code.unindent();
        code.pr("}");

        code.unindent();
    }

    private void generatePreambles() {
        code.pr(String.join("\n",
            "from uclid.builder import *",
            "from uclid.builder_sugar import *",
            "from ext_module import ModuleWithExternalProcedures",
            "from ext_procedure import ExternalProcedure",
            "from utils import Lang"
        ));
    }

    /**
     * Generate UCLID5 using the Python API.
     */
    private void generateUclidCode() {
        generatePreambles();
        generateUclidMainModule();
        generatePythonMainFunction();
    }

    /** Generate the Uclid model. */
    protected void generateUclidFile() {
        try {
            // Generate main.ucl and print to file
            code = new CodeBuilder();
            Path file = this.outputDir.resolve("main.py");
            String filePath = file.toString();
            generateUclidCode();
            code.writeToFile(filePath);
            this.generatedFiles.add(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

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
