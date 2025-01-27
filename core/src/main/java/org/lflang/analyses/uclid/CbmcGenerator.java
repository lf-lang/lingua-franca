package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;
import org.lflang.AttributeUtils;
import org.lflang.analyses.uclid.ReactionData.Argument;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Action;
import org.lflang.lf.Attribute;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.target.TargetConfig;
import org.lflang.util.StringUtil;

public class CbmcGenerator {

  /** The main place to put generated code. */
  private CodeBuilder code;

  /** LF Generator context */
  public final LFGeneratorContext context;

  /** Target language */
  public final String targetLanguage = "c";

  /** The directory where the generated files are placed */
  public Path outputDir;

  /** A list of paths to the CBMC files generated */
  public List<Path> generatedFiles = new ArrayList<>();

  public HashMap<String, ReactionData> reactionDataMap;

  public CbmcGenerator(LFGeneratorContext context, HashMap<String, ReactionData> reactionDataMap) {
    this.context = context;
    this.reactionDataMap = reactionDataMap;
  }

  public void doGenerate(TargetConfig targetConfig) {
    setupDirectories();
    List<Reactor> reactorDefs = ASTUtils.getAllReactors(targetConfig.getMainResource());
    for (Reactor reactorDef : reactorDefs) {
      String targetLanguage = getTargetLanguage(reactorDef);
      if (targetLanguage.equalsIgnoreCase(this.targetLanguage)) {
        System.out.println("Generating CBMC files for " + reactorDef.getName());
        List<Reaction> reactionDefs = reactorDef.getReactions();
        for (int index = 0; index < reactionDefs.size(); index++) {
          Reaction reactionDef = reactionDefs.get(index);
          generateCbmcFile(reactorDef, reactionDef, index);
        }
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
      String reactionName = getReactionName(reactorDef, reactionIndex);
      Path file = this.outputDir.resolve(reactionName + ".c");
      String filePath = file.toString();
      generateCbmcCodeForReaction(reactorDef, reactionDef, reactionName);
      code.writeToFile(filePath);
      this.generatedFiles.add(file);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  protected void generateCbmcCodeForReaction(
      Reactor reactor, Reaction reaction, String reactionName) {
    this.reactionDataMap.put(reactionName, new ReactionData(reactionName));
    generateIncludes();
    generatePreamble(reactor);
    generateAPIFunctions();
    // Define and instantiate input ports and input logical actions.
    List<? extends TypedVariable> inputs = getAllInputs(reaction);
    for (var p : inputs) {
      generatePortOrActionStructAndNondet(p, reactor, reactionName);
      instantiatePortOrActionStruct(p, true, reactor, reactionName);
    }
    // Define and instantiate output port and output logical actions.
    List<? extends TypedVariable> outputs = getAllOutputs(reaction);
    for (var p : outputs) {
      generatePortOrActionStructAndNondet(p, reactor, reactionName);
      instantiatePortOrActionStruct(p, false, reactor, reactionName);
    }
    // Define and instantiate self struct.
    if (reactor.getParameters().size() > 0 || reactor.getStateVars().size() > 0) {
      generateSelfStructAndNondet(reactor, reactionName);
      instantiateSelfStruct();
    }
    generateReactionFunction(reactionName, reaction.getCode().getBody());
    generateMainFunction(reactor, reaction, reactionName);
  }

  protected void generateIncludes() {
    code.pr(
        String.join("\n", "#include <stdlib.h>", "#include <stdbool.h>", "#include <assert.h>"));
  }

  // FIXME: Code duplication with generateUserPreamblesForReactor() from CGenerator.java.
  protected void generatePreamble(Reactor reactor) {
    for (Preamble p : ASTUtils.allFileLevelPreambles(reactor)) {
      code.pr("// *********** From the global preamble, verbatim:");
      code.pr(ASTUtils.toText(p.getCode()));
      code.pr("\n// *********** End of preamble.");
    }
    for (Preamble p : ASTUtils.allPreambles(reactor)) {
      code.pr("// *********** From the reactor preamble, verbatim:");
      code.pr(ASTUtils.toText(p.getCode()));
      code.pr("\n// *********** End of preamble.");
    }
  }

  protected void generateAPIFunctions() {
    code.pr("// lf_set");
    code.pr(
        String.join(
            "\n",
            "#define lf_set(__out, __val) \\",
            "do { \\",
            "__out->value = __val; \\",
            "__out->is_present = true; \\",
            "} while (0)"));
    code.pr("// lf_set_present");
    code.pr(
        String.join(
            "\n",
            "#define lf_set_present(__out) \\",
            "do { \\",
            "__out->is_present = true; \\",
            "} while (0)"));
    code.pr("// lf_schedule: only supports delay = 0");
    code.pr(
        String.join(
            "\n",
            "#define lf_schedule(__action, __delay) \\",
            "do { \\",
            "assert(__delay == 0); \\",
            "__action->is_present = true; \\",
            "} while (0)"));
    code.pr("// lf_schedule_int: only supports delay = 0");
    code.pr(
        String.join(
            "\n",
            "#define lf_schedule_int(__action, __delay, __val) \\",
            "do { \\",
            "assert(__delay == 0); \\",
            "__action->is_present = true; \\",
            "__action->value = __val; \\",
            "} while (0)"));
  }

  /** Generate the struct type definitions for a port */
  private void generatePortOrActionStructAndNondet(
      TypedVariable tv, Reactor reactor, String reactionName) {
    assert tv instanceof Port || tv instanceof Action; // Only ports and actions are allowed.
    boolean hasTypedValue = tv.getType() != null;
    code.pr("typedef struct {");
    code.indent();
    // NOTE: The following fields are required to be the first ones so that
    // pointer to this struct can be cast to a (lf_port_base_t*) or to
    // (token_template_t*) to access these fields for any port.
    // IMPORTANT: These must match exactly the fields defined in port.h!!
    String struct_body =
        String.join(
            "\n",
            // "// lf_port_base_t base;", // From port.h
            // "// lf_token_t* token;", // From token_template_t
            // "// size_t length;", // From token_template_t
            "bool is_present;",
            hasTypedValue ? tv.getType().getId() + " value;" : "");
    code.pr(struct_body);
    code.unindent();
    String name = getTypeName(reactor, tv);
    code.pr("} " + name + ";");
    // Generate a CBMC nondet function.
    code.pr(name + " " + "nondet_" + name + "();");

    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    reactionData.addType(name);
    Argument is_present = reactionData.new Argument();
    is_present.setTgtType("bool");
    reactionData.getType(name).put("is_present", is_present);
    if (hasTypedValue) {
      Argument value = reactionData.new Argument();
      value.setTgtType(tv.getType().getId());
      reactionData.getType(name).put("value", value);
    }
  }

  /** Instantiate the struct type for an input port. */
  private void instantiatePortOrActionStruct(
      TypedVariable tv, Boolean input, Reactor reactor, String reactionName) {
    assert tv instanceof Port || tv instanceof Action; // Only ports and actions are allowed.
    assert !(tv instanceof Action) && !input; // Actions are outputs.
    String name = getTypeName(reactor, tv);
    code.pr(name + " * " + tv.getName() + ";");

    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    if (input) {
      reactionData.addInput();
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtName(tv.getName());
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtType(name + " *");
    } else {
      reactionData.addOutput();
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtName(tv.getName());
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtType(name + " *");
    }
  }

  protected void generateReactionFunction(String name, String body) {
    code.pr("void " + name + "() {");
    code.indent();
    if (body != null) code.pr(body);
    code.unindent();
    code.pr("}");
  }

  protected void generateSelfStructAndNondet(Reactor reactor, String reactionName) {
    String reactorSelfName = reactor.getName() + "_self_t";
    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    reactionData.addType(reactorSelfName);
    code.pr("// Define the self struct.");
    code.pr("typedef struct " + "self_t" + " {");
    code.indent();
    for (Parameter p : reactor.getParameters()) {
      String targetType = p.getType().getId();
      code.pr(targetType + " " + p.getName() + ";");
      Argument arg = reactionData.new Argument();
      arg.setTgtType(targetType);
      reactionData.getType(reactorSelfName).put(p.getName(), arg);
    }
    for (StateVar s : reactor.getStateVars()) {
      String targetType = s.getType().getId();
      code.pr(targetType + " " + s.getName() + ";");
      Argument arg = reactionData.new Argument();
      arg.setTgtType(targetType);
      reactionData.getType(reactorSelfName).put(s.getName(), arg);
    }
    code.unindent();
    code.pr("} " + "self_t" + ";");
    // Declare a CBMC nondet function.
    code.pr("self_t nondet_self_t();");
    // Add self to inputs and outputs in reactionData.
    reactionData.addInput();
    reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtName("init_self");
    reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtType(reactorSelfName + " *");
    reactionData.addOutput();
    reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtName("self");
    reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtType(reactorSelfName + " *");
  }

  protected void instantiateSelfStruct() {
    code.pr("self_t * init_self;");
    code.pr("self_t * self;");
  }

  private void generateMainFunction(Reactor reactor, Reaction reaction, String reactionName) {

    List<? extends TypedVariable> inputs = getAllInputs(reaction);
    List<? extends TypedVariable> outputs = getAllOutputs(reaction);
    List<? extends TypedVariable> all =
        Stream.of(inputs, outputs).flatMap(Collection::stream).toList();
    Boolean hasSelf = reactor.getParameters().size() > 0 || reactor.getStateVars().size() > 0;

    code.pr("int main() {");
    code.indent();

    code.pr("// calloc for inputs and the self struct.");
    for (var p : all) {
      String name = p.getName();
      String type = getTypeName(reactor, p);
      code.pr(name + " = " + "calloc(1, sizeof(" + type + "));");
    }
    if (hasSelf) {
      code.pr("init_self  = calloc(1, sizeof(self_t));");
      code.pr("self       = calloc(1, sizeof(self_t));");
    }

    code.pr("// Assume that there are no NULL pointers.");
    code.pr("__CPROVER_assume(");
    code.indent();
    code.pr(String.join(" && ", all.stream().map(it -> it.getName()).toList()));
    if (hasSelf) {
      code.pr((all.size() > 0 ? "&& " : "") + "init_self && self");
    }
    code.unindent();
    code.pr(");");

    code.pr(
        "// Initialize input ports, input logical actions, and self structs with nondeterministic"
            + " values.");
    for (var p : inputs) {
      String name = p.getName();
      var type = getTypeName(reactor, p);
      code.pr("*" + name + " = " + "nondet_" + type + "();");
    }
    if (hasSelf) {
      code.pr("*init_self = nondet_self_t();");
      code.pr("*self = nondet_self_t();");
    }

    if (hasSelf) {
      code.pr("// Set state variables to nondeterministic initial values.");
      for (Parameter p : reactor.getParameters()) {
        code.pr("self->" + p.getName() + " = " + "init_self->" + p.getName() + ";");
      }
      for (StateVar s : reactor.getStateVars()) {
        code.pr("self->" + s.getName() + " = " + "init_self->" + s.getName() + ";");
      }
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

  /** Get the target language of a reactor */
  private String getTargetLanguage(Reactor reactor) {
    List<Attribute> langList =
        AttributeUtils.getAttributes(reactor).stream()
            .filter(attr -> attr.getAttrName().equals("lang"))
            .toList();
    if (langList.isEmpty()) {
      throw new RuntimeException(
          "Reactor " + reactor.getName() + " does not have a `lang` attribute.");
    }
    String lang = langList.get(0).getAttrParms().get(0).getValue();
    System.out.println("Target language for " + reactor.getName() + " is " + lang);
    return StringUtil.removeQuotes(lang);
  }

  private void setupDirectories() {
    // Make sure the target directory exists.
    Path cbmcModelGenDir = context.getFileConfig().getModelGenPath().resolve(this.targetLanguage);
    this.outputDir = Paths.get(cbmcModelGenDir.toString());
    try {
      Files.createDirectories(outputDir);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    System.out.println("The models will be located in: " + outputDir);
  }

  private String getTypeName(Reactor reactor, TypedVariable tv) {
    return reactor.getName() + "_" + tv.getName() + "_t";
  }

  private String getReactionName(Reactor reactor, int index) {
    return reactor.getName() + "_reaction_" + (index + 1);
  }

  // FIXME: Add this to ASTUtils.java in a principled way.
  // Can typed variables be other things than ports and actions?
  private List<? extends TypedVariable> getAllInputs(Reaction reaction) {
    return Stream.concat(
            reaction.getTriggers().stream().filter(it -> (it instanceof VarRef)),
            reaction.getSources().stream())
        .map(it -> (VarRef) it)
        .map(it -> it.getVariable())
        .filter(
            it ->
                (it instanceof TypedVariable tv
                    && (tv instanceof Port p || tv instanceof Action a)))
        .map(it -> (TypedVariable) it)
        .toList();
  }

  // FIXME: Add this to ASTUtils.java in a principled way.
  // Can typed variables be other things than ports and actions?
  private List<? extends TypedVariable> getAllOutputs(Reaction reaction) {
    return reaction.getEffects().stream()
        .map(it -> it.getVariable())
        .filter(
            it ->
                (it instanceof TypedVariable tv
                    && (tv instanceof Port p || tv instanceof Action a)))
        .map(it -> (TypedVariable) it)
        .toList();
  }
}
