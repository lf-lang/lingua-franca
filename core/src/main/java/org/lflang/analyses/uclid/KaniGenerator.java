package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
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
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.target.TargetConfig;
import org.lflang.util.StringUtil;

public class KaniGenerator {

  /** The main place to put generated code. */
  private CodeBuilder code;

  /** LF Generator context */
  public final LFGeneratorContext context;

  /** Target language */
  public final String targetLanguage = "rust";

  /** The directory where the generated files are placed */
  public Path outputDir;

  /** A list of paths to the kani files generated */
  public List<Path> generatedFiles = new ArrayList<>();

  public HashMap<String, ReactionData> reactionDataMap;

  public KaniGenerator(LFGeneratorContext context, HashMap<String, ReactionData> reactionDataMap) {
    this.context = context;
    this.reactionDataMap = reactionDataMap;
  }

  public void doGenerate(TargetConfig targetConfig) {
    setupDirectories();
    List<Reactor> reactorDefs = ASTUtils.getAllReactors(targetConfig.getMainResource());
    for (Reactor reactorDef : reactorDefs) {
      String lang = getTargetLanguage(reactorDef);
      if (lang.equalsIgnoreCase(this.targetLanguage)) {
        List<Reaction> reactionDefs = reactorDef.getReactions();
        for (int index = 0; index < reactionDefs.size(); index++) {
          Reaction reactionDef = reactionDefs.get(index);
          generateKaniFile(reactorDef, reactionDef, index);
        }
      }
    }
  }

  ////////////////////////////////////////////////////////////
  //// Protected methods
  /** Get the target language of the reactor */
  /** Generate the Uclid model. */
  protected void generateKaniFile(Reactor reactorDef, Reaction reactionDef, int reactionIndex) {
    try {
      // Generate main.ucl and print to file
      code = new CodeBuilder();
      String reactionName = getReactionName(reactorDef, reactionIndex);
      Path file = this.outputDir.resolve(reactionName + ".rs");
      String filePath = file.toString();
      generateKaniCodeForReaction(reactorDef, reactionDef, reactionName);
      code.writeToFile(filePath);
      this.generatedFiles.add(file);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  protected void generateKaniCodeForReaction(
      Reactor reactor, Reaction reaction, String reactionName) {
    this.reactionDataMap.put(reactionName, new ReactionData(reactionName));
    generatePolymorphicTypesAndContext();
    code.pr("// Type aliases for specific port and action types.");
    // Define and instantiate input ports and input logical actions.
    List<? extends TypedVariable> inputs = getAllInputs(reaction);
    for (var p : inputs) {
      instantiatePortAndActionTypes(p, reactor, reactionName);
    }
    // Define and instantiate output port and output logical actions.
    List<? extends TypedVariable> outputs = getAllOutputs(reaction);
    for (var p : outputs) {
      instantiatePortAndActionTypes(p, reactor, reactionName);
    }
    // Define and instantiate self struct.
    generateSelfStruct(reactor, reactionName);
    generateReactionFunction(reactor, reaction, reactionName);
    generateMainFunction(reactor, reaction, reactionName);
  }

  protected void generatePolymorphicTypesAndContext() {
    // Set up Port<T> and Action<T> structs and their functions.
    code.pr("#[derive(Debug, Copy, Clone, Default)]");
    code.pr("#[cfg_attr(kani, derive(kani::Arbitrary))]");
    code.pr("struct Port<T> {");
    code.indent();
    code.pr("is_present: bool,");
    code.pr("value: T,");
    code.unindent();
    code.pr("}");
    code.pr("");

    code.pr("#[derive(Debug, Copy, Clone, Default)]");
    code.pr("#[cfg_attr(kani, derive(kani::Arbitrary))]");
    code.pr("struct Action {");
    code.indent();
    code.pr("is_present: bool,");
    code.unindent();
    code.pr("}");
    code.pr("");

    code.pr("#[derive(Debug, Copy, Clone, Default)]");
    code.pr("#[cfg_attr(kani, derive(kani::Arbitrary))]");
    code.pr("struct ActionWithValue<T> {");
    code.indent();
    code.pr("is_present: bool,");
    code.pr("value: T,");
    code.unindent();
    code.pr("}");

    code.pr("struct Context {}");
    code.pr("impl Context {");
    code.indent();
    code.pr("// Function to set the value of a Port<T>");
    code.pr("fn set<T>(&mut self, p: &mut Port<T>, v: T) {");
    code.indent();
    code.pr("p.is_present = true;");
    code.pr("p.value = v;");
    code.unindent();
    code.pr("}");
    code.pr("// Function to get the value from a Port<T>");
    code.pr("fn get<T>(p: &Port<T>) -> &T {");
    code.indent();
    code.pr("&p.value");
    code.unindent();
    code.pr("}");
    code.pr("// Function to schedule an action without setting a value");
    code.pr("fn schedule(&mut self, a: &mut Action, delay: u64) {");
    code.indent();
    code.pr("assert!(delay == 0, \"Delays other than 0 are not supported\");");
    code.pr("a.is_present = true;");
    code.unindent();
    code.pr("}");
    code.pr("// Function to schedule an action and set a value");
    code.pr("fn schedule_int<T>(&mut self, a: &mut ActionWithValue<T>, delay: u64, v: T) {");
    code.indent();
    code.pr("assert!(delay == 0, \"Delays other than 0 are not supported\");");
    code.pr("a.is_present = true;");
    code.pr("a.value = v;");
    code.unindent();
    code.pr("}");
    code.unindent();
    code.pr("}");
  }

  /** Generate the struct type definitions for a port */
  private void instantiatePortAndActionTypes(
      TypedVariable tv, Reactor reactor, String reactionName) {
    assert tv instanceof Port || tv instanceof Action; // Only ports and actions are allowed.
    String typeName = getTypeName(reactor, tv);
    if (tv instanceof Port) {
      String targetType = tv.getType().getId();
      code.pr("type " + typeName + " = Port<" + targetType + ">;");
    } else if (tv instanceof Action) {
      if (tv.getType() != null) {
        String targetType = tv.getType().getId();
        code.pr("type " + typeName + " = ActionWithValue<" + targetType + ">;");
      } else {
        code.pr("type " + typeName + " = Action;");
      }
    }

    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    reactionData.addType(typeName);
    Argument is_present = reactionData.new Argument();
    is_present.setTgtType("bool");
    reactionData.getType(typeName).put("is_present", is_present);
    if (tv.getType() != null) {
      Argument value = reactionData.new Argument();
      value.setTgtType(tv.getType().getId());
      reactionData.getType(typeName).put("value", value);
    }
  }

  /** Instantiate the struct type for an input port. */
  private void instantiatePortOrActionStruct(
      TypedVariable tv, Boolean input, Reactor reactor, String reactionName) {
    assert tv instanceof Port || tv instanceof Action; // Only ports and actions are allowed.
    String typeName = getTypeName(reactor, tv);
    if (input) { // havoc input
      code.pr("let mut " + tv.getName() + " : " + typeName + " = kani::any();");
    } else { // default output
      code.pr("let mut " + tv.getName() + " = " + typeName + "::default();");
    }

    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    if (input) {
      reactionData.addInput();
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtName(tv.getName());
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtType(typeName);
    } else {
      reactionData.addOutput();
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtName(tv.getName());
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtType(typeName);
    }
  }

  protected void generateSelfStruct(Reactor reactor, String reactionName) {
    String reactorSelfName = getSelfTypeName(reactor);
    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    // Only add the self struct if it has parameters or state variables.
    if (reactor.getParameters().size() > 0 || reactor.getStateVars().size() > 0) {
      reactionData.addType(reactorSelfName);
    }

    code.pr("// Struct to hold the state");
    code.pr("#[derive(Debug, Copy, Clone, Default)]");
    code.pr("#[cfg_attr(kani, derive(kani::Arbitrary))]");
    code.pr("struct " + reactorSelfName + " {");
    code.indent();
    for (Parameter p : reactor.getParameters()) {
      String targetType = p.getType().getId();
      code.pr(p.getName() + ": " + targetType + ",");
      Argument arg = reactionData.new Argument();
      arg.setTgtType(targetType);
      reactionData.getType(reactorSelfName).put(p.getName(), arg);
    }
    for (StateVar s : reactor.getStateVars()) {
      String targetType = s.getType().getId();
      code.pr(s.getName() + ": " + targetType + ",");
      Argument arg = reactionData.new Argument();
      arg.setTgtType(targetType);
      reactionData.getType(reactorSelfName).put(s.getName(), arg);
    }
    code.unindent();
    code.pr("}");
  }

  protected void instantiateSelfStruct(Reactor reactor, String reactionName) {
    String reactorSelfName = getSelfTypeName(reactor);
    ReactionData reactionData = this.reactionDataMap.get(reactionName);
    code.pr("// Create the initial state");
    code.pr("let mut _init_self : " + getSelfTypeName(reactor) + " = kani::any();");
    code.pr("// Create _self to be a deep copy of _init_self");
    code.pr("let mut _self : " + getSelfTypeName(reactor) + " = _init_self.clone();");
    code.pr("");

    // Add self to inputs and outputs in reactionData.
    if (reactor.getParameters().size() > 0 || reactor.getStateVars().size() > 0) {
      reactionData.addInput();
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtName("_init_self");
      reactionData.inputs.get(reactionData.inputs.size() - 1).setTgtType(reactorSelfName);
      reactionData.addOutput();
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtName("_self");
      reactionData.outputs.get(reactionData.outputs.size() - 1).setTgtType(reactorSelfName);
    }
  }

  protected void generateReactionFunction(Reactor reactor, Reaction reaction, String name) {
    String reactorSelfName = getSelfTypeName(reactor);
    code.pr("// The function");
    code.pr("impl " + reactorSelfName + " {");
    code.indent();
    code.pr("fn " + name + "(");
    code.indent();
    code.pr("&mut self,");
    code.pr("ctx: &mut Context,");
    for (var inp : getAllInputs(reaction)) {
      code.pr(inp.getName() + ": &" + getTypeName(reactor, inp) + ",");
    }
    for (var out : getAllOutputs(reaction)) {
      code.pr(out.getName() + ": &mut " + getTypeName(reactor, out) + ",");
    }
    code.unindent();
    code.pr(") {");
    code.indent(); // code
    code.pr(reaction.getCode().getBody());
    code.unindent(); // code
    code.pr("}");
    code.unindent();
    code.pr("}");
  }

  private void generateMainFunction(Reactor reactor, Reaction reaction, String reactionName) {

    List<? extends TypedVariable> inputs = getAllInputs(reaction);
    List<? extends TypedVariable> outputs = getAllOutputs(reaction);
    Boolean hasSelf = reactor.getParameters().size() > 0 || reactor.getStateVars().size() > 0;

    code.pr("#[kani::proof]");
    code.pr("fn main() {");
    code.indent(); // main
    code.pr("let mut ctx = Context {};");

    // Create instances of input triggers
    code.pr("// Havoc inputs");
    for (var p : inputs) {
      instantiatePortOrActionStruct(p, true, reactor, reactionName);
    }
    // Create instances of output effects
    code.pr("// Default outputs");
    for (var p : outputs) {
      instantiatePortOrActionStruct(p, false, reactor, reactionName);
    }

    // Create the self struct
    instantiateSelfStruct(reactor, reactionName);

    // Assume the precondition
    code.pr("// Assume the precondition");
    code.pr("kani::assume(precondition(");
    code.indent(); // precondition
    for (var p : inputs) {
      code.pr("&" + p.getName() + ",");
    }
    if (hasSelf) {
      code.pr("&_init_self,");
    }
    code.unindent(); // precondition
    code.pr("));");
    // Run the reaction
    code.pr("// Execute the reaction");
    code.pr("_self." + reactionName + "(");
    code.indent(); // reaction
    code.pr("&mut ctx,");
    for (var p : inputs) {
      code.pr("&" + p.getName() + ",");
    }
    for (var p : outputs) {
      code.pr("&mut " + p.getName() + ",");
    }
    code.unindent(); // reaction
    code.pr(");");
    // Assert the postcondition
    code.pr("// Assert the postcondition");
    code.pr("assert!(postcondition(");
    code.indent(); // postcondition
    for (var p : inputs) {
      code.pr("&" + p.getName() + ",");
    }
    if (hasSelf) {
      code.pr("&_init_self,");
    }
    for (var p : outputs) {
      code.pr("&" + p.getName() + ",");
    }
    if (hasSelf) {
      code.pr("&_self,");
    }
    code.unindent(); // postcondition
    code.pr("));");
    code.unindent(); // main
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
    Path kaniModelGenDir = context.getFileConfig().getModelGenPath().resolve(this.targetLanguage);
    this.outputDir = Paths.get(kaniModelGenDir.toString());
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

  private String getSelfTypeName(Reactor reactor) {
    return reactor.getName() + "_self_t";
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
