package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.target.TargetConfig;

public class CbmcGenerator {

    /** The main place to put generated code. */
    private CodeBuilder code = new CodeBuilder();

    /** LF Generator context */
    public final LFGeneratorContext context;

    /** The directory where the generated files are placed */
    public Path outputDir;

    /** A list of paths to the uclid files generated */
    public List<Path> generatedFiles = new ArrayList<>();

    public CbmcGenerator(LFGeneratorContext context) {
        this.context = context;
    }

    public void doGenerate(TargetConfig targetConfig, Instantiation mainDef) {
        setupDirectories();
        List<Reactor> reactorDefs = ASTUtils.getAllReactors(targetConfig.getMainResource());
        for (Reactor reactorDef : reactorDefs) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (int index = 0; index < reactionDefs.size(); index++) {
                Reaction reactionDef = reactionDefs.get(index);
                generateCbmcFile(reactorDef, reactionDef, index);
            }
        }
    }

    ////////////////////////////////////////////////////////////
    //// Protected methods

    /** Generate the Uclid model. */
    protected void generateCbmcFile(Reactor reactorDef, Reaction reactionDef, int reactionIndex) {
        try {
            // Generate main.ucl and print to file
            code = new CodeBuilder();
            String reactionName = "reaction_" + reactorDef.getName() + "_" + reactionIndex;
            Path file = this.outputDir.resolve(reactionName + ".c");
            String filePath = file.toString();
            generateCbmcCodeForReaction(reactionName, reactionDef);
            code.writeToFile(filePath);
            this.generatedFiles.add(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    protected void generateCbmcCodeForReaction(String name, Reaction reaction) {
        generateIncludes();
        generateAPIFunctions();
        for (var t : reaction.getTriggers()) {
            if (t instanceof VarRef varRef) {
                Variable v = varRef.getVariable();
                if (v instanceof TypedVariable tv
                    && tv instanceof Port p) {
                    generateAuxiliaryStruct(p);
                }
            }
        }
        generateReactionFunction(name, reaction.getCode().getBody());
    }

    protected void generateIncludes() {
        code.pr(String.join("\n",
            "#include <stdlib.h>",
            "#include <stdbool.h>",
            "#include <assert.h>"
        ));
    }

    protected void generateAPIFunctions() {
        code.pr(String.join("\n",
            "#define lf_set(out, val) \\",
            "do { \\",
            "out->value = val; \\ ",
            "out->is_present = true; \\",
            "} while (0)"
        ));
    }

    /**
     * Generate the struct type definitions for the port of the reactor
     */
    private void generateAuxiliaryStruct(Port port) {
        code.pr("typedef struct {");
        code.indent();
        // NOTE: The following fields are required to be the first ones so that
        // pointer to this struct can be cast to a (lf_port_base_t*) or to
        // (token_template_t*) to access these fields for any port.
        // IMPORTANT: These must match exactly the fields defined in port.h!!
        code.pr(
            String.join(
                "\n",
                "// lf_token_t* token;", // From token_template_t
                "// size_t length;", // From token_template_t
                "bool is_present;",
                port.getType().getId() + " value;"
            )
        );
        code.unindent();
        var name = port.getName() + "_t";
        code.pr("} " + name + ";");
    }

    protected void generateReactionFunction(String name, String body) {
        code.pr("void " + name + "() {");
        code.indent();
        code.pr(body);
        code.unindent();
        code.pr("}");
    }

    ////////////////////////////////////////////////////////////
    //// Private methods
    
    private void setupDirectories() {
        // Make sure the target directory exists.
        Path cbmcModelGenDir = context.getFileConfig().getModelGenPath().resolve("c");
        this.outputDir = Paths.get(cbmcModelGenDir.toString());
        try {
            Files.createDirectories(outputDir);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        System.out.println("The models will be located in: " + outputDir);
    }
    
}
