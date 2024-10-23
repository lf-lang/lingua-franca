package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Stream;

import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.target.TargetConfig;

public class CbmcGenerator {

    /** The main place to put generated code. */
    private CodeBuilder code;

    /** LF Generator context */
    public final LFGeneratorContext context;

    /** The directory where the generated files are placed */
    public Path outputDir;

    /** A list of paths to the CBMC files generated */
    public List<Path> generatedFiles = new ArrayList<>();

    /** CTypes. FIXME: Could this be static? */
    private CTypes types = new CTypes();

    public CbmcGenerator(LFGeneratorContext context) {
        this.context = context;
    }

    public void doGenerate(TargetConfig targetConfig) {
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
            String reactionName = reactorDef.getName() + "_reaction_" + reactionIndex;
            Path file = this.outputDir.resolve(reactionName + ".c");
            String filePath = file.toString();
            generateCbmcCodeForReaction(reactorDef, reactionDef, reactionName);
            code.writeToFile(filePath);
            this.generatedFiles.add(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    protected void generateCbmcCodeForReaction(Reactor reactor, Reaction reaction, String reactionName) {
        generateIncludes();
        generateAPIFunctions();
        // Define and instantiate input ports.
        List<Port> inputPorts = getAllInputPorts(reaction);
        for (var p : inputPorts) {
            generatePortStructAndNondet(p);
            instantiatePortStruct(p);
        }
        // Define and instantiate output port.
        List<Port> outputPorts = getAllOutputPorts(reaction);
        for (var p : outputPorts) {
            generatePortStructAndNondet(p);
            instantiatePortStruct(p);
        }
        // Define and instantiate self struct.
        generateSelfStructAndNondet(reactor);
        instantiateSelfStruct();
        generateReactionFunction(reactionName, reaction.getCode().getBody());
        generateMainFunction(reactor, reaction, reactionName);
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
     * Generate the struct type definitions for an input port
     */
    private void generatePortStructAndNondet(Port port) {
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
        // Generate a CBMC nondet function.
        code.pr(name + " " + "nondet_" + name + "();");
    }

    /**
     * Instantiate the struct type for an input port.
     */
    private void instantiatePortStruct(Port port) {
        var name = port.getName() + "_t";
        code.pr(name + "* " + port.getName() + ";");
    }

    protected void generateReactionFunction(String name, String body) {
        code.pr("void " + name + "() {");
        code.indent();
        code.pr(body);
        code.unindent();
        code.pr("}");
    }

    protected void generateSelfStructAndNondet(Reactor reactor) {
        code.pr("typedef struct " + "self_t" + " {");
        code.indent();
        for (Parameter p : reactor.getParameters()) {
            code.pr(types.getTargetType(p) + " " + p.getName() + ";");
        }
        for (StateVar s : reactor.getStateVars()) {
            code.pr(types.getTargetType(s) + " " + s.getName() + ";");
        }
        code.unindent();
        code.pr("} " + "self_t" + ";");
        // Declare a CBMC nondet function.
        code.pr("self_t nondet_self_t();");
    }

    protected void instantiateSelfStruct() {
        code.pr("self_t * init_self;");
        code.pr("self_t * self;");
    }

    private void generateMainFunction(Reactor reactor, Reaction reaction, String reactionName) {
        
        List<Port> inputs = getAllInputPorts(reaction);
        List<Port> outputs = getAllOutputPorts(reaction);
        List<Port> allPorts = Stream.of(inputs, outputs).flatMap(Collection::stream).toList();
        
        code.pr("int main() {");
        code.indent();

        code.pr("// calloc for inputs and the self struct.");
        for (var p : allPorts) {
            String name = p.getName();
            String type = name + "_t";
            code.pr(name + " = " + "calloc(1, sizeof(" + type + "));");
        }
        code.pr("init_self  = calloc(1, sizeof(self_t));");
        code.pr("self       = calloc(1, sizeof(self_t));");
        
        code.pr("// Assume that there are no NULL pointers.");
        code.pr("__CPROVER_assume(");
        code.indent();
        code.pr(
            String.join(" && ", 
                Stream.of(
                    List.of("init_self", "self"), 
                    allPorts.stream().map(it -> it.getName()).toList()
                ).flatMap(Collection::stream).toList()));
        code.unindent();
        code.pr(");");

        code.pr("// Initialize ports and self structs with nondeterministic values.");
        for (var p : inputs) {
            String name = p.getName();
            String type = name + "_t";
            code.pr("*" + name + " = " + "nondet_" + type + "();");
        }
        code.pr("*init_self = nondet_self_t();");
        code.pr("*self = nondet_self_t();");

        code.pr("// Set state variables to nondeterministic initial values.");
        for (StateVar s : reactor.getStateVars()) {
            code.pr("self->" + s.getName() + " = " + "init_self->" + s.getName() + ";");
        }

        code.pr("// CBMC checks pre/post-conditions.");
        code.pr("__CPROVER_assume(PRECONDITION);");
        code.pr(reactionName + "();");
        code.pr("assert(POSTCONDITION);");

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

    // FIXME: Add this to ASTUtils.java in a principled way.
    private List<Port> getAllInputPorts(Reaction reaction) {
        return reaction.getTriggers().stream()
                .filter(it -> (it instanceof VarRef))
                .map(it -> (VarRef) it)
                .map(it -> it.getVariable())
                .filter(it -> (it instanceof TypedVariable tv && tv instanceof Port p))
                .map(it -> (Port) it).toList();
    }

    // FIXME: Add this to ASTUtils.java in a principled way.
    private List<Port> getAllOutputPorts(Reaction reaction) {
        return reaction.getEffects().stream()
                .map(it -> it.getVariable())
                .filter(it -> (it instanceof TypedVariable tv && tv instanceof Port p))
                .map(it -> (Port) it).toList();
    }
    
}
