package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.analyses.statespace.Tag;
import org.lflang.analyses.uclid.ReactionData.UclCall;
import org.lflang.analyses.uclid.UclidGenerator.Tactic;
import org.lflang.ast.ASTUtils;
import org.lflang.dsl.MTLLexer;
import org.lflang.dsl.MTLParser;
import org.lflang.dsl.MTLParser.MtlContext;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.Action;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Literal;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.target.TargetConfig;
import org.lflang.util.StringUtil;

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

  /** CTypes. FIXME: Could this be static? */
  private CTypes types = new CTypes();

  /** The main reactor instance */
  public ReactorInstance main;

  /** A list of MTL properties represented in Attributes. */
  private List<Attribute> properties;

  /** The name of the property */
  private String property_name;

  private Tactic tactic = Tactic.BMC;

  /** The specification of the property */
  private String spec;

  /** The expected result of the property */
  private String expect;

  /** The number of steps to check the property */
  private int CT;

  /**
   * The horizon (the total time interval required for evaluating an MTL property, which is derived
   * from the MTL spec), the completeness threshold (CT) (the number of transitions required for
   * evaluating the FOL spec in the trace), and the transpiled FOL spec.
   */
  private long horizon = 0; // in nanoseconds

  /** First-Order Logic formula matching the Safety MTL property */
  private String FOLSpec = "";

  /** Maximum CT supported. This is a hardcoded value. */
  private static final int CT_MAX_SUPPORTED = 100;

  /**
   * If true, use logical time-based semantics; otherwise, use event-based semantics, as described
   * in Sirjani et. al (2020). This is currently always false and serves as a placeholder for a
   * future version that supports logical time-based semantics.
   */
  private boolean logicalTimeBased = true;

  /** A list of reactors in the LF program */
  public List<Reactor> reactors = new ArrayList<>();

  /** A list of reactions in the LF program */
  public List<Reaction> reactions = new ArrayList<>();

  public HashMap<String, ReactionData> reactionDataMap;
  public HashMap<ReactorInstance, Integer> reactorInst2Cnt = new HashMap<>(); // FIXME: Can be local
  public HashMap<ReactionInstance, Integer> reactionInst2Cnt = new HashMap<>();
  public HashMap<ReactorInstance, Integer> reactorInst2Index = new HashMap<>();
  public HashMap<ReactionInstance, Integer> reactionInst2Index = new HashMap<>();

  /** State space diagram for the LF program */
  StateSpaceDiagram diagram;

  public UclidFSMGenerator(
      LFGeneratorContext context,
      List<Attribute> properties,
      HashMap<String, ReactionData> reactionDataMap) {
    this.context = context;
    this.modGenDir = context.getFileConfig().getModelGenPath();
    this.targetConfig = context.getTargetConfig();
    this.properties = properties;
    this.reactors = ASTUtils.getAllReactors(targetConfig.getMainResource());
    this.reactions =
        this.reactors.stream().map(it -> it.getReactions()).flatMap(List::stream).toList();
    this.reactionDataMap = reactionDataMap;
  }

  public void doGenerate(TargetConfig targetConfig, Instantiation mainDef, ReactorInstance main) {
    this.main = main;
    if (main == null) {
      throw new RuntimeException("No main reactor instance found.");
    }
    setupDirectories();
    generateStateSpace(this.main);
    // Generate a Uclid model for each property.
    for (Attribute prop : this.properties) {
      this.property_name =
          StringUtil.removeQuotes(
              prop.getAttrParms().stream()
                  .filter(attr -> attr.getName().equals("name"))
                  .findFirst()
                  .get()
                  .getValue());
      String tacticStr =
          StringUtil.removeQuotes(
              prop.getAttrParms().stream()
                  .filter(attr -> attr.getName().equals("tactic"))
                  .findFirst()
                  .get()
                  .getValue());
      if (tacticStr.equals("bmc")) this.tactic = Tactic.BMC;
      this.spec =
          StringUtil.removeQuotes(
              prop.getAttrParms().stream()
                  .filter(attr -> attr.getName().equals("spec"))
                  .findFirst()
                  .get()
                  .getValue());

      processMTLSpec();

      Optional<AttrParm> CTAttr =
          prop.getAttrParms().stream().filter(attr -> attr.getName().equals("CT")).findFirst();
      if (CTAttr.isPresent()) {
        this.CT = Integer.parseInt(CTAttr.get().getValue());
      } else {
        computeCT();
      }
      // For automating data collection, print the CT to stderr.
      System.err.println("CT: " + this.CT);
      if (this.CT > CT_MAX_SUPPORTED) {
        System.out.println(
            "ERROR: The maximum steps supported is "
                + CT_MAX_SUPPORTED
                + " but checking this property requires "
                + this.CT
                + " steps. "
                + "This property will NOT be checked.");
        continue;
      }

      Optional<AttrParm> ExpectAttr =
          prop.getAttrParms().stream().filter(attr -> attr.getName().equals("expect")).findFirst();
      if (ExpectAttr.isPresent()) this.expect = ExpectAttr.get().getValue();

      generateUclidFile();
    }
    // generateUclidFile();
  }

  ////////////////////////////////////////////////////////////
  //// Private methods

  private void generatePythonMainFunction() {
    code.pr(
        String.join(
            "\n",
            "if __name__ == \"__main__\":",
            "    m = MainModule()",
            "    # print(m.buildUclidModule().__inject__())",
            "    mc = getModelChecker(m)",
            "    res = mc.check()",
            "    mc.report(\"result.json\")"));
  }

  private void generateUclidMainModule() {
    code.pr("class MainModule(ModuleWithExternalProcedures):");
    code.indent();
    generateInitFunction();
    generateUclidModule();
    code.unindent();
  }

  private void generateInitFunction() {
    code.pr("def __init__(self, delete=True, verbose=False):");
    code.indent();
    code.pr("super().__init__(\"main\", delete, verbose)");

    /**
     * Example: pedestrian_reaction_1 = ExternalProcedure( name="pedestrian_reaction_1",
     * lang=Lang.C, filepath="test/c/traffic_light/pedestrian_reaction_1.c",
     * jsonpath="test/json/traffic_light/pedestrian_reaction_1.json", )
     */
    for (Reactor reactorDef : this.reactors) {
      List<Reaction> reactionDefs = reactorDef.getReactions();
      for (int index = 0; index < reactionDefs.size(); index++) {
        String reactionName = getReactionName(reactorDef, index);
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
     * Example: self.ext_procs = { pedestrian_reaction_1.name: pedestrian_reaction_1,
     * traffic_light_reaction_1.name: traffic_light_reaction_1, traffic_light_reaction_2.name:
     * traffic_light_reaction_2, traffic_light_reaction_3.name: traffic_light_reaction_3, }
     */
    code.pr("self.ext_procs = {");
    code.indent();
    for (Reactor reactorDef : this.reactors) {
      List<Reaction> reactionDefs = reactorDef.getReactions();
      for (int index = 0; index < reactionDefs.size(); index++) {
        String reactionName = getReactionName(reactorDef, index);
        code.pr(reactionName + ".name: " + reactionName + ",");
      }
    }

    code.unindent();
    code.pr("}");

    code.unindent();
  }

  private void generateUclidModule() {
    code.pr("def buildUclidModule(self) -> UclidModule:");
    code.indent();
    code.pr("m = UclidModule(\"main\")");
    code.pr("UBoolFalse = UclidBooleanLiteral(False)");
    code.pr("UBoolTrue = UclidBooleanLiteral(True)");

    // Define trace indices as a group,
    // so that we can use finite quantifiers.
    // Example: if this.CT = 3, then indices = {0, 1, 2, 3}
    // String indices = String.join(", ", Stream.iterate(0, i -> i + 1).limit(this.CT +
    // 1).map(Object::toString).toList());
    code.pr("indices = m.mkGroup(\"indices\", UInt, list(range(" + (this.CT + 1) + ")))");

    generateRecordTypes();

    /** Update reactionData with uclid information */
    for (Reactor reactorDef : this.reactors) {
      List<Reaction> reactionDefs = reactorDef.getReactions();
      for (int i = 0; i < reactionDefs.size(); i++) {
        String reactionName = getReactionName(reactorDef, i);
        Reaction reactionDef = reactionDefs.get(i);
        List<? extends TypedVariable> all =
            Stream.of(getAllInputs(reactionDef), getAllOutputs(reactionDef))
                .flatMap(List::stream)
                .toList();
        ReactionData reactionData = this.reactionDataMap.get(reactionName);
        for (int j = 0; j < all.size(); j++) {
          TypedVariable tv = all.get(j);
          String type = reactorDef.getName() + "_" + tv.getName() + "_t";
          reactionData.types.get(type).get("is_present").setUclType("boolean");
          boolean hasTypedValue = tv.getType() != null;
          if (hasTypedValue) {
            String uclid_type = getUclidTypeFromCType(tv.getType().getId(), false);
            reactionData.types.get(type).get("value").setUclType(uclid_type);
          }
        }
        Boolean hasSelf = reactorDef.getParameters().size() + reactorDef.getStateVars().size() > 0;
        if (hasSelf) {
          String reactorSelfType = reactorDef.getName() + "_self_t";
          reactionData
              .types
              .get(reactorSelfType)
              .forEach(
                  (key, value) -> {
                    String uclid_type = getUclidTypeFromCType(value.getTgtType(), false);
                    value.setUclType(uclid_type);
                  });
        }
      }
    }

    generateNoInlineProcedures();

    /** Get number of reactorInstances and reactionInstances */
    StateSpaceNode node = diagram.head;
    while (true) {
      List<ReactionInstance> reactionInsts = new ArrayList<>(node.getReactionsInvoked());
      // Increment the counter for the reactor instance
      for (ReactionInstance reactionInst : reactionInsts) {
        ReactorInstance reactorInst = reactionInst.getParent();
        this.reactorInst2Cnt.put(
            reactorInst, this.reactorInst2Cnt.getOrDefault(reactorInst, 0) + 1);
        this.reactionInst2Cnt.put(
            reactionInst, this.reactionInst2Cnt.getOrDefault(reactionInst, 0) + 1);
      }

      if (node == diagram.tail) {
        break;
      } else {
        node = diagram.getDownstreamNode(node);
      }
    }

    generateVariableDeclarations();

    generateResetFireProcedure();

    /** Generate a state procedure for each state */
    node = diagram.head;
    while (true) {
      generateStateProcedure(node);
      if (node == diagram.tail) {
        break;
      } else {
        node = diagram.getDownstreamNode(node);
      }
    }

    generateStateMachineProcedure();

    generateInitBlock();

    generateNextBlock();

    generateProperty();

    generateControlBlock();

    /** Return statement */
    code.pr("return m");

    code.unindent();
  }

  private void generateRecordTypes() {
    /** For each reactor */
    for (Reactor reactorDef : this.reactors) {
      /** Generate a type for each port. */
      List<? extends TypedVariable> all =
          Stream.of(reactorDef.getInputs(), reactorDef.getOutputs(), reactorDef.getActions())
              .flatMap(List::stream)
              .toList();
      String reactor_name = reactorDef.getName();
      for (TypedVariable tv : all) {
        String name = reactor_name + "_" + tv.getName();
        String type = name + "_t";
        code.pr(type + " = m.mkRecordType(");
        code.indent();
        code.pr("\"" + type + "\",");
        code.pr("[");
        code.indent();
        code.pr("(\"is_present\", UBool),");
        boolean hasTypedValue = tv.getType() != null;
        if (hasTypedValue) {
          String dtype = tv.getType().getId();
          String uclid_type = getUclidTypeFromCType(dtype, true);
          code.pr("(\"value\", " + uclid_type + "),");
        } else {
          code.pr(
              "(\"value\", "
                  + "UclidBVType(1)"
                  + "), # Dummy field"); // Print a dummy field here to fix the Uclid parse error.
        }
        code.unindent();
        code.pr("]");
        code.unindent();
        code.pr(")");
        /** Create a port variable for function input and output arguments. */
        code.pr(name + " = UclidLiteral(\"" + name + "\")");
      }

      /** Generate a self type that includes state variables and parameters. */
      String self_name = reactor_name + "_self";
      String self_type = self_name + "_t";
      code.pr(self_type + " = m.mkRecordType(");
      code.indent();
      code.pr("\"" + self_type + "\",");
      code.pr("[");
      code.indent();
      for (Parameter p : reactorDef.getParameters()) {
        String type = types.getTargetType(p);
        String uclid_type = getUclidTypeFromCType(type, true);
        code.pr("(\"" + p.getName() + "\", " + uclid_type + "),");
      }
      for (StateVar s : reactorDef.getStateVars()) {
        String type = types.getTargetType(s);
        String uclid_type = getUclidTypeFromCType(type, true);
        code.pr("(\"" + s.getName() + "\", " + uclid_type + "),");
      }
      // Add a dummy variable if there are no parameters or state variables
      if (reactorDef.getParameters().size() + reactorDef.getStateVars().size() == 0) {
        code.pr("(\"_dummy\", UBool),");
      }
      code.unindent();
      code.pr("]");
      code.unindent();
      code.pr(")");
      /** Create a self variables for function input and output arguments. */
      String self_prestate = self_name + "_prestate";
      code.pr(self_prestate + " = UclidLiteral(\"" + self_prestate + "\")");
      String self_poststate = self_name + "_poststate";
      code.pr(self_poststate + " = UclidLiteral(\"" + self_poststate + "\")");

      /**
       * Generate reactor type that includes ports and self type (state variables and parameters).
       */
      String reactor_type = getReactorType(reactorDef);
      code.pr(reactor_type + " = m.mkRecordType(");
      code.indent();
      code.pr("\"" + reactor_type + "\",");
      code.pr("[");
      code.indent();
      for (TypedVariable tv : all) {
        String type = reactor_name + "_" + tv.getName() + "_t";
        code.pr("(\"" + tv.getName() + "\", " + type + "),");
      }
      code.pr("(\"self\", " + self_type + "),");
      code.unindent();
      code.pr("]");
      code.unindent();
      code.pr(")");
    }
  }

  private void generateNoInlineProcedures() {
    /** Generate noinline procedure for each reaction. */
    for (Reactor reactorDef : this.reactors) {
      List<Reaction> reactionDefs = reactorDef.getReactions();
      for (int i = 0; i < reactionDefs.size(); i++) {
        String reactionName = getReactionName(reactorDef, i);
        String requires = reactionName + "_requires";
        String ensures = reactionName + "_ensures";
        String sig = reactionName + "_sig";
        String proc = reactionName + "_proc";
        List<? extends TypedVariable> inputs = getAllInputs(reactionDefs.get(i));
        List<? extends TypedVariable> outputs = getAllOutputs(reactionDefs.get(i));
        ReactionData reactionData = this.reactionDataMap.get(reactionName);
        /** Creates requires expression */
        code.pr(requires + " = UclidRaw(");
        code.indent();
        code.pr("self.ext_procs[\"" + reactionName + "\"].getLatestUclidRequiresString()");
        code.unindent();
        code.pr(")");
        /** Creates ensures expression */
        code.pr(ensures + " = UclidRaw(");
        code.indent();
        code.pr("self.ext_procs[\"" + reactionName + "\"].getLatestUclidEnsuresString()");
        code.unindent();
        code.pr(")");
        /** Creates function signature */
        code.pr(sig + " = UclidProcedureSig(");
        code.indent();
        code.pr("inputs=[");
        code.indent();
        for (int j = 0; j < inputs.size(); j++) {
          TypedVariable tv = inputs.get(j);
          String name = reactorDef.getName() + "_" + tv.getName();
          String type = name + "_t";
          code.pr("(" + name + ", " + type + "),");
          reactionData.inputs.get(j).setUclName(name);
          reactionData.inputs.get(j).setUclType(type);
        }
        Boolean hasSelf = reactorDef.getParameters().size() + reactorDef.getStateVars().size() > 0;
        String reactorSelfType = reactorDef.getName() + "_self_t";
        String reactorSelfInput = reactorDef.getName() + "_self_prestate";
        code.pr("(" + reactorSelfInput + ", " + reactorSelfType + "),");
        if (hasSelf) {
          reactionData.inputs.get(reactionData.inputs.size() - 1).setUclName(reactorSelfInput);
          reactionData.inputs.get(reactionData.inputs.size() - 1).setUclType(reactorSelfType);
        }
        code.unindent();
        code.pr("],");
        code.pr("returns=[");
        code.indent();
        for (int j = 0; j < outputs.size(); j++) {
          TypedVariable tv = outputs.get(j);
          String name = reactorDef.getName() + "_" + tv.getName();
          String type = name + "_t";
          code.pr("(" + name + ", " + type + "),");
          reactionData.outputs.get(j).setUclName(name);
          reactionData.outputs.get(j).setUclType(type);
        }
        String reactorSelfOutput = reactorDef.getName() + "_self_poststate";
        code.pr("(" + reactorSelfOutput + ", " + reactorSelfType + "),");
        if (hasSelf) {
          reactionData.outputs.get(reactionData.outputs.size() - 1).setUclName(reactorSelfOutput);
          reactionData.outputs.get(reactionData.outputs.size() - 1).setUclType(reactorSelfType);
        }
        code.unindent();
        code.pr("],");
        code.pr("requires=" + requires + ",");
        code.pr("ensures=" + ensures + ",");
        code.pr("noinline=True,");
        code.unindent();
        code.pr(")");

        code.pr(proc + " = m.mkProcedure(");
        code.indent();
        code.pr("\"" + reactionName + "\",");
        code.pr(sig + ",");
        code.pr("UclidBlockStmt([]),");
        code.unindent();
        code.pr(")");
      }
    }
  }

  private void generateVariableDeclarations() {
    /** Create snapshots, a delay buffer, and an end of step array for each reactor instance */
    for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
      ReactorInstance reactorInst = entry.getKey();
      String reactorInstName = reactorInst.getName();
      String reactorType = getReactorType(reactorInst.reactorDefinition);
      String reactorInstSnapshotArray = getReactorInstSnapshotArray(reactorInst);
      String reactorInstArray = getReactorInstArray(reactorInst);
      code.pr(
          reactorInstSnapshotArray
              + " = [m.mkVar(\""
              + reactorInstName
              + "_\" + str(i), "
              + reactorType
              + ") for i in range("
              + (2 * entry.getValue() + 1)
              + ")]");
      /**
       * Create a buffer-versioned variable for each reactor instance in case there are delayed
       * connectons or actions. FIXME: This only handles the case where the delay is larger than all
       * timer intervals. This is a limitation of the current implementation.
       */
      code.pr(
          getReactorInstDelayBuffer(reactorInst)
              + " = m.mkVar(\""
              + getReactorInstDelayBuffer(reactorInst)
              + "\", "
              + reactorType
              + ")");
      /** Create an array storing the value at the end of the step */
      code.pr(
          reactorInstArray
              + " = m.mkVar(\""
              + reactorInstArray
              + "\", UclidArrayType(UInt, "
              + reactorType
              + "))");
    }
    /** Group snapshots and delay buffers into python arrays */
    code.pr(
        "snapshot_arrays = sum(["
            + this.reactorInst2Cnt.keySet().stream()
                .map(it -> getReactorInstSnapshotArray(it))
                .reduce((a, b) -> a + ", " + b)
                .get()
            + "], [])");
    code.pr(
        "delay_buffers = ["
            + this.reactorInst2Cnt.keySet().stream()
                .map(it -> getReactorInstDelayBuffer(it))
                .reduce((a, b) -> a + ", " + b)
                .get()
            + "]");
    /** Create an integer for time to track current time an array for timestamps */
    code.pr("time = m.mkVar(\"time\", UInt)");
    code.pr("timestamps = m.mkVar(\"timestamps\", UclidArrayType(UInt, UInt))");
    List<String> reactionFiredNames = new ArrayList<>();
    /** Create a variable for each reaction that indicates whether the reaction has fired */
    for (HashMap.Entry<ReactionInstance, Integer> entry : this.reactionInst2Cnt.entrySet()) {
      String reactionFired = getReactionFiredName(entry.getKey());
      String reactionFiredArray = getReactionFiredArray(entry.getKey());
      String reactionFiredInstArray = getReactionFiredInstArray(entry.getKey());
      code.pr(reactionFired + " = m.mkVar(\"" + reactionFired + "\", UBool)");
      reactionFiredNames.add(reactionFired);
      /** Create an array storing the value at the end of the step */
      code.pr(
          reactionFiredArray
              + " = m.mkVar(\""
              + reactionFiredArray
              + "\", UclidArrayType(UInt, UBool))");
      code.pr(
          reactionFiredInstArray
              + " = [m.mkVar(\""
              + reactionFired
              + "_inst_\" + str(i), UBool) for i in range("
              + entry.getValue()
              + ")]");
    }
    /**
     * Create array of strings for "fired" variables including reactionFiredNames and
     * reactionFiredInst_i
     */
    code.pr("fired = [");
    code.indent();
    code.pr(String.join(", ", reactionFiredNames) + ",");
    code.pr(
        String.join(
            ", ",
            this.reactionInst2Cnt.keySet().stream()
                .map(it -> "*" + getReactionFiredInstArray(it))
                .reduce((a, b) -> a + ", " + b)
                .get()));
    code.unindent();
    code.pr("]");
  }

  private void generateResetFireProcedure() {
    /** Signature for reset_fire */
    code.pr("reset_fire_sig = UclidProcedureSig(");
    code.indent();
    code.pr("inputs=[],");
    code.pr("modifies=fired,");
    code.pr("returns=[],");
    code.pr("noinline=False,");
    code.unindent();
    code.pr(")");
    /** Procedure declaration for reset_fire */
    code.pr("reset_fire_proc = m.mkProcedure(");
    code.indent();
    code.pr("\"reset_fire\",");
    code.pr("reset_fire_sig,");
    code.pr("UclidBlockStmt([UclidAssignStmt(f, UBoolFalse) for f in fired])");
    code.unindent();
    code.pr(")");
  }

  private void generateStateMachineProcedure() {
    /**
     * State machine procedure num_states: number of states; equal to the number of nodes in the
     * state space diagram cycle_start: the index of the loop node plus one if the loop node is not
     * null otherwise is the number of states stepNum: the number of steps taken
     *
     * <p>The state machine starts from state **1** and transitions to the next state until it
     * reaches the tail node (tail node is the last state before the loop node if the loop node
     * exists; otherwise it is the last state) If there is a loop node, it transitions to another
     * state (id = num_states) representing the loopNodeNext in the diagram, and then transitions to
     * the state that follows the loop node.
     */
    int numStates = diagram.tail.getIndex() + 1;
    int cycle_start = diagram.loopNode == null ? numStates : diagram.loopNode.getIndex() + 1;
    /** Declare variables and constants needed for state machine */
    code.pr("state = m.mkVar(\"state\", UInt)");
    code.pr("num_states = m.mkConst(\"num_states\", UInt, UclidIntegerLiteral(" + numStates + "))");
    code.pr(
        "cycle_start = m.mkConst(\"cycle_start\", UInt, UclidIntegerLiteral(" + cycle_start + "))");
    code.pr("stepNum = m.mkVar(\"stepNum\", UInt)");
    code.pr("END = m.mkConst(\"END\", UInt, UclidIntegerLiteral(" + this.CT + "))");
    /** State machine signature */
    code.pr("state_machine_sig = UclidProcedureSig(");
    code.indent();
    code.pr("inputs=[],");
    List<ReactorInstance> reactorInsts = new ArrayList<>(this.reactorInst2Cnt.keySet());
    code.pr(
        "modifies=[state, time, timestamps, stepNum] + fired + snapshot_arrays + delay_buffers"
            + " + ["
            + reactorInsts.stream()
                .map(it -> getReactorInstArray(it))
                .reduce((a, b) -> a + ", " + b)
                .get()
            + "]" // end of step arrays
            + " + ["
            + reactionInst2Cnt.keySet().stream()
                .map(it -> getReactionFiredArray(it))
                .reduce((a, b) -> a + ", " + b)
                .get()
            + "],"); // reaction fired arrays
    code.pr("returns=[],");
    code.pr("noinline=False,");
    code.unindent();
    code.pr(")");
    /** State machine procedure */
    code.pr("state_machine_proc = m.mkProcedure(");
    code.indent(); // Procedure
    code.pr("\"state_machine\",");
    code.pr("state_machine_sig,");
    code.pr("UclidBlockStmt([");
    code.indent(); // Block statement
    /** Reset variables indicating whether procedures have fired */
    code.pr("UclidProcedureCallStmt(reset_fire_proc, [], []),");
    /** Increment step number */
    code.pr("UclidAssignStmt(stepNum, Uadd([stepNum, UclidIntegerLiteral(1)])),");
    /** State transition */
    code.pr("UclidITEStmt(");
    code.indent(); // ITE statement
    // code.pr("Ugte([state, Usub([num_states, UclidIntegerLiteral(1)])]),");
    code.pr("Ugte([state, num_states]),");
    code.pr("UclidBlockStmt([");
    code.indent(); // Block statement
    code.pr("UclidComment(\"cycle_start is the state after the loop node\"),");
    code.pr("UclidComment(\"If there is no loop, it will be num_states\"),");
    code.pr("UclidAssignStmt(state, cycle_start),");
    code.unindent(); // Block statement
    code.pr("]),");
    code.pr("UclidAssignStmt(state, Uadd([state, UclidIntegerLiteral(1)])),");
    code.unindent(); // ITE statement
    code.pr("),");
    code.pr("UclidCaseStmt(");
    code.indent(); // Case statement
    code.pr("[");
    code.indent(); // Conditions
    for (int i = 1; i <= diagram.tail.getIndex() + 1; ++i) {
      code.pr("Ueq([state, UclidIntegerLiteral(" + i + ")]),");
    }
    code.unindent(); // Conditions
    code.pr("],");
    code.pr("[");
    code.indent(); // Actions
    StateSpaceNode node = diagram.head, lastNode;
    /** Starts from the second node because the first is executed in the init block */
    while (node != diagram.tail) {
      lastNode = node;
      node = diagram.getDownstreamNode(node);
      long timeElapsed = node.getTag().timestamp - lastNode.getTag().timestamp;
      code.pr("UclidBlockStmt([");
      code.indent();
      if (node == diagram.loopNode) {
        code.pr("UclidComment(\"Loop node\"),");
      }
      code.pr("UclidProcedureCallStmt(state_" + node.getIndex() + "_proc, [], []),");
      code.pr("UclidAssignStmt(time, Uadd([time, UclidIntegerLiteral(" + timeElapsed + ")])),");
      code.unindent();
      code.pr("]),");
    }
    if (diagram.loopNodeNext != null) {
      long timeElapsed = diagram.loopNodeNext.getTag().timestamp - node.getTag().timestamp;
      code.pr("UclidBlockStmt([");
      code.indent();
      code.pr("UclidComment(\"Loop node next\"),");
      code.pr("UclidProcedureCallStmt(state_" + diagram.loopNode.getIndex() + "_proc, [], []),");
      code.pr("UclidAssignStmt(time, Uadd([time, UclidIntegerLiteral(" + timeElapsed + ")])),");
      code.unindent();
      code.pr("]),");
    } else {
      code.pr("UclidBlockStmt([");
      code.indent();
      code.pr("UclidComment(\"No such state\"),");
      code.unindent();
      code.pr("]),"); // Default case
    }
    code.unindent(); // Actions
    code.pr("],");
    code.unindent(); // Case statement
    code.pr("),");
    // Record state after initialization
    for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
      code.pr(
          "UclidAssignStmt("
              + UclidArraySelect(getReactorInstArray(entry.getKey()), "stepNum")
              + ", "
              + getReactorInstSnapshot(entry.getKey(), 0)
              + "),");
    }
    for (HashMap.Entry<ReactionInstance, Integer> entry : this.reactionInst2Cnt.entrySet()) {
      code.pr(
          "UclidAssignStmt("
              + UclidArraySelect(getReactionFiredArray(entry.getKey()), "stepNum")
              + ", "
              + getReactionFiredName(entry.getKey())
              + "),");
    }
    code.pr("UclidAssignStmt(" + UclidArraySelect("timestamps", "stepNum") + ", time),");
    code.unindent(); // Block statement
    code.pr("])");
    code.unindent(); // Procedure
    code.pr(")");
  }

  private void generateInitBlock() {
    /** Uclid init block */
    code.pr("m.setInit(UclidInitBlock([");
    code.indent();
    /** Havoc each variable and assign the value of the variable to all other snapshots */
    for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
      ReactorInstance reactorInst = entry.getKey();
      String reactorInstOrig = getReactorInstSnapshot(reactorInst, 0);
      code.pr("UclidHavocStmt(" + reactorInstOrig + "),");
      for (ParameterInstance paramInst : reactorInst.parameters) {
        Parameter paramDef = paramInst.getDefinition();
        String paramOrig =
            UclidRecordSelect(UclidRecordSelect(reactorInstOrig, "self"), paramInst.getName());
        // FIXME: handle types other than Literal
        Literal paramLit = (Literal) paramInst.getActualValue().getExpr();
        String paramString =
            getUclidValueFromCValue(paramLit.getLiteral(), types.getTargetType(paramDef));
        code.pr("UclidAssignStmt(" + paramOrig + ", " + paramString + "),");
      }
      for (StateVar stateVar : reactorInst.reactorDefinition.getStateVars()) {
        String stateVarOrig =
            UclidRecordSelect(UclidRecordSelect(reactorInstOrig, "self"), stateVar.getName());
        // FIXME: handle types other than Literal
        Initializer stateVarInit = stateVar.getInit();
        if (stateVarInit != null) {
          Literal stateVarLit = (Literal) stateVarInit.getExpr();
          String stateVarLitString =
              getUclidValueFromCValue(stateVarLit.getLiteral(), types.getTargetType(stateVar));
          code.pr("UclidAssignStmt(" + stateVarOrig + ", " + stateVarLitString + "),");
        }
      }
      code.pr(
          "*[UclidAssignStmt(v, "
              + reactorInstOrig
              + ") for v in "
              + getReactorInstSnapshotArray(reactorInst)
              + "[1:]],");
    }
    /** Reset fire variables */
    code.pr("UclidProcedureCallStmt(reset_fire_proc, [], []),");
    /** Call initial state procedure */
    code.pr("UclidProcedureCallStmt(state_0_proc, [], []),");
    code.pr("UclidAssignStmt(state, UclidIntegerLiteral(0)),");
    code.pr("UclidAssignStmt(stepNum, UclidIntegerLiteral(0)),");
    code.pr("UclidAssignStmt(time, UclidIntegerLiteral(0)),");
    // Record state after initialization
    for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
      code.pr(
          "UclidAssignStmt("
              + UclidArraySelect(getReactorInstArray(entry.getKey()), "stepNum")
              + ", "
              + getReactorInstSnapshot(entry.getKey(), 0)
              + "),");
    }
    for (HashMap.Entry<ReactionInstance, Integer> entry : this.reactionInst2Cnt.entrySet()) {
      code.pr(
          "UclidAssignStmt("
              + UclidArraySelect(getReactionFiredArray(entry.getKey()), "stepNum")
              + ", "
              + getReactionFiredName(entry.getKey())
              + "),");
    }
    code.pr("UclidAssignStmt(" + UclidArraySelect("timestamps", "stepNum") + ", time),");
    code.unindent();
    code.pr("]))");
  }

  private void generateNextBlock() {
    /** Perform state transition in next block. */
    code.pr("m.setNext(UclidNextBlock(UclidProcedureCallStmt(state_machine_proc, [], [])))");
  }

  private void generateProperty() {
    /** Property */
    code.pr("property_sig = UclidFunctionSig([(\"i\", UInt)], UBool)");
    code.pr("property_def = m.mkDefine(\"PROPERTY\", property_sig, UclidRaw(\"\"\"");
    code.indent();
    code.pr(this.FOLSpec);
    code.unindent();
    code.pr("\"\"\"))");
    code.pr("m.mkProperty(");
    code.indent();
    code.pr("\"" + this.tactic + "_" + this.property_name + "\",");
    code.pr("UclidRaw(\"stepNum == END ==> PROPERTY(0)\"),");
    code.unindent();
    code.pr(")");
  }

  private void generateControlBlock() {
    /** Control block */
    code.pr("m.setControl(UclidControlBlock([");
    code.indent(); // Control block
    code.pr("UclidBMCCommand(\"v\", " + this.CT + "),");
    code.pr("UclidCheckCommand(),");
    code.pr("UclidPrintResultsCommand(),");
    code.pr("UclidPrintCexJSONCommand(\"v\", sum(");
    code.indent(); // sum
    code.pr("[");
    code.indent(); //
    code.pr("fired,");
    code.pr("[state],");
    for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
      // List of ports that contain both input and output
      ReactorInstance reactorInst = entry.getKey();
      Reactor reactorDef = reactorInst.reactorDefinition;
      List<? extends TypedVariable> portsAndActions =
          Stream.of(reactorDef.getInputs(), reactorDef.getOutputs(), reactorDef.getActions())
              .flatMap(List::stream)
              .toList();
      // List of attributes for each port and actions
      code.pr("[");
      code.indent();
      code.pr("UclidRecordSelect(UclidRecordSelect(v, p), attr)");
      code.pr("for attr in [\"is_present\", \"value\"]");
      code.pr(
          "for p in ["
              + String.join(
                  ", ", portsAndActions.stream().map(it -> "\"" + it.getName() + "\"").toList())
              + "]");
      code.pr(
          "for v in "
              + getReactorInstSnapshotArray(reactorInst)
              + " + ["
              + getReactorInstDelayBuffer(reactorInst)
              + "]");
      code.unindent();
      code.pr("],");
      // List of attributes for self
      code.pr("[");
      code.indent();
      code.pr("UclidRecordSelect(UclidRecordSelect(v, \"self\"), attr)");
      List<String> self_attrs =
          Stream.of(
                  reactorDef.getParameters().stream().map(it -> it.getName()).toList(),
                  reactorDef.getStateVars().stream().map(it -> it.getName()).toList())
              .flatMap(List::stream)
              .toList();
      code.pr(
          "for attr in ["
              + String.join(", ", self_attrs.stream().map(it -> "\"" + it + "\"").toList())
              + "]");
      code.pr(
          "for v in "
              + getReactorInstSnapshotArray(reactorInst)
              + " + ["
              + getReactorInstDelayBuffer(reactorInst)
              + "]");
      code.unindent();
      code.pr("],");
    }
    code.unindent(); //
    code.pr("], []),");
    code.unindent(); // sum
    code.pr(")");
    code.unindent(); // Control block
    code.pr("]))");
  }

  private void generateStateProcedure(StateSpaceNode node) {
    /**
     * Sort invoked reactions by the smallest number in the set returned by
     * reactionInstance.getLevels()
     */
    List<ReactionInstance> reactionInsts = new ArrayList<>(node.getReactionsInvoked());
    reactionInsts.sort(
        (r1, r2) ->
            r1.getLevels().stream().min(Integer::compare).get()
                - r2.getLevels().stream().min(Integer::compare).get());

    String state_name = "state_" + node.getIndex();
    String sig = state_name + "_sig";
    String proc = state_name + "_proc";
    /** Generate function signature */
    code.pr(sig + " = UclidProcedureSig(");
    code.indent();
    code.pr("inputs=[],");
    code.pr("modifies=fired + delay_buffers + snapshot_arrays,");
    code.pr("returns=[],");
    code.pr("noinline=False,");
    code.unindent();
    code.pr(")");
    /** Generate procedure */
    code.pr(proc + " = m.mkProcedure(");
    code.indent(); // Procedure
    code.pr("\"" + state_name + "\",");
    code.pr(sig + ",");
    code.pr("UclidBlockStmt([");
    code.indent(); // Block statement
    /**
     * Procedure body Assumes that each reaction only modifies the state of the reactor which the
     * reaction belongs to i.e. reaction only reads from input ports and writes to output ports of
     * the reactor (and also modifies the state of the reactor). Thus we only need to store the
     * state before and after the reaction is invoked. First assign to ports or actions values that
     * are delayed due to the `after` keyword or min-delay. Then invoke each reaction in the state
     * based on the order of the levels. For each invocation, we follow the following steps: 1.
     * Store the state of the reactor before invoking the reaction. 2. Check if the input triggers
     * are present. 3. If the input triggers are present, call the external procedure and record
     * that the reaction is fired. 4. Assign the value to downstream ports. 5. Store the state of
     * the reactor after invoking the reaction.
     */

    /** 1. Store the state of the reactor before invoking the reaction. */
    Set<TriggerInstance<? extends Variable>> updates = node.getUpdateInstances();
    for (TriggerInstance<? extends Variable> inst : updates) {
      ReactorInstance reactorInst = inst.getParent();
      String name = inst.getName();
      code.pr(
          "UclidAssignStmt("
              + UclidRecordSelect(getReactorInstSnapshot(reactorInst, 0), name)
              + ", "
              + UclidRecordSelect(getReactorInstDelayBuffer(reactorInst), name)
              + "),");
    }

    for (ReactionInstance reactionInst : reactionInsts) {
      ReactorInstance reactorInst = reactionInst.getParent();
      Reactor reactorDef = reactorInst.reactorDefinition;
      Reaction reactionDef = reactionInst.getDefinition();
      List<? extends TypedVariable> triggers = getAllInputs(reactionDef);
      List<? extends TypedVariable> effects = getAllOutputs(reactionDef);
      Boolean hasSelf = reactorDef.getParameters().size() + reactorDef.getStateVars().size() > 0;
      String reactionName = getReactionName(reactorInst.reactorDefinition, reactionInst.index);
      ReactionData reactionData = this.reactionDataMap.get(reactionName);
      UclCall uclCall = reactionData.new UclCall();
      /** Store reactor state before invoking reaction. */
      String reactorInstOrigName = getReactorInstSnapshot(reactorInst, 0);
      int preStateIndex = getNextReactorInstIndex(reactorInst);
      code.pr("# Store reactor pre-state");
      code.pr(
          "UclidAssignStmt("
              + getReactorInstSnapshot(reactorInst, preStateIndex)
              + ", "
              + reactorInstOrigName
              + "),");
      /** Check if input triggers are present. */
      if (triggers.size() > 0) {
        code.pr("# Check if input triggers are present");
        code.pr("UclidITEStmt(");
        code.indent(); // ITE statement
        code.pr("Uor([");
        code.indent(); // Conditions
        for (TypedVariable tv : triggers) {
          String present =
              UclidRecordSelect(UclidRecordSelect(reactorInstOrigName, tv.getName()), "is_present");
          code.pr(present + ",");
        }
        code.unindent(); // Conditions
        code.pr("]),");
        code.pr("UclidBlockStmt([");
        code.indent(); // Block statement
      }
      /** Assign true to variable that indicates whether a reaction has fired */
      String reactionFired = getReactionFiredName(reactionInst);
      code.pr("UclidAssignStmt(" + reactionFired + ", UBoolTrue),");
      int idx = getNextReactionInstIndex(reactionInst);
      String fired = getReactionFiredInstArray(reactionInst) + "[" + idx + "]";
      code.pr("UclidAssignStmt(" + fired + ", UBoolTrue),");
      String firedInst = getReactionFiredInst(reactionInst, idx);
      uclCall.flag = firedInst;
      /** Call external procedure */
      code.pr("# Call external procedure");
      code.pr("UclidProcedureCallStmt(");
      code.indent(); // Procedure call
      code.pr(getReactionName(reactorInst.reactorDefinition, reactionInst.index) + "_proc,");
      /** Input triggers */
      code.pr("[");
      code.indent(); // input triggers
      for (TypedVariable tv : triggers) {
        String name = UclidRecordSelect(reactorInstOrigName, tv.getName());
        code.pr(name + ",");
        uclCall.inputs.add(getReactorInstCopy(reactorInst, preStateIndex) + "." + tv.getName());
      }
      code.pr(UclidRecordSelect(reactorInstOrigName, "self") + ",");
      if (hasSelf) uclCall.inputs.add(reactorInst.getName() + "_" + preStateIndex + "." + "self");
      code.unindent(); // input triggers
      code.pr("],");
      /** Output effects */
      int postStateIndex = getNextReactorInstIndex(reactorInst);
      code.pr("[");
      code.indent(); // output effects
      for (TypedVariable tv : effects) {
        String name = UclidRecordSelect(reactorInstOrigName, tv.getName());
        String uclOutput = getReactorInstCopy(reactorInst, postStateIndex) + "." + tv.getName();
        /**
         * If the effect is an action, first assign to the buffer variable. The value will be
         * assigned to the actual variable at the correct time afterwards.
         */
        if (tv instanceof Action) {
          name = UclidRecordSelect(getReactorInstDelayBuffer(reactorInst), tv.getName());
          uclOutput = getReactorInstDelayBuffer(reactorInst) + "." + tv.getName();
        }
        code.pr(name + ",");
        /** Use buffer for Actions because there may be time delays */
        uclCall.outputs.add(uclOutput);
      }
      code.pr(UclidRecordSelect(reactorInstOrigName, "self") + ",");
      if (hasSelf) uclCall.outputs.add(reactorInst.getName() + "_" + postStateIndex + "." + "self");
      code.unindent(); // output effects
      code.pr("]");
      code.unindent(); // Procedure call
      code.pr("),");
      /** Assign value to downstream ports */
      code.pr("# Assign values to downstream ports");
      for (TypedVariable tv : effects) {
        if (tv instanceof Port) {
          Port port = (Port) tv;
          PortInstance portInst = reactorInst.lookupPortInstance(port);
          for (SendRange range : portInst.getDependentPorts()) {
            PortInstance source = range.instance;
            ReactorInstance sourceReactorInst = source.getParent();
            String sourceReactorInstName = getReactorInstSnapshot(sourceReactorInst, 0);
            Connection connection = range.connection;
            List<RuntimeRange<PortInstance>> destinations = range.destinations;
            for (RuntimeRange<PortInstance> d : destinations) {
              PortInstance dest = d.instance;
              ReactorInstance destReactorInst = dest.getParent();
              String destReactorInstName = getReactorInstSnapshot(destReactorInst, 0);
              // Extract delay value
              // long delay = 0;
              // If the delay is nonzero, we need to store the value in a buffer
              // assign the value to the destination port at the correct time.
              if (connection.getDelay() != null) {
                // Somehow delay is an Expression,
                // which makes it hard to convert to nanoseconds.
                Expression delayExpr = connection.getDelay();
                if (delayExpr instanceof Time) {
                  // long interval = ((Time) delayExpr).getInterval();
                  // String unit = ((Time) delayExpr).getUnit();
                  // TimeValue timeValue = new TimeValue(interval, TimeUnit.fromName(unit));
                  // delay = timeValue.toNanoSeconds();
                  code.pr(
                      "UclidAssignStmt("
                          + UclidRecordSelect(
                              getReactorInstDelayBuffer(destReactorInst), dest.getName())
                          + ", "
                          + UclidRecordSelect(sourceReactorInstName, source.getName())
                          + "),");
                } else {
                  throw new RuntimeException("Unsupported delay expression: " + delayExpr);
                }
              } else {
                code.pr(
                    "UclidAssignStmt("
                        + UclidRecordSelect(destReactorInstName, dest.getName())
                        + ", "
                        + UclidRecordSelect(sourceReactorInstName, source.getName())
                        + "),");
              }
            }
          }
        } else if (tv instanceof Action) {
          Action action = (Action) tv;
          ActionInstance actionInst = reactorInst.lookupActionInstance(action);
          TimeValue min_delay = actionInst.getMinDelay();
          if (min_delay == TimeValue.ZERO) {
            code.pr(
                "UclidAssignStmt("
                    + UclidRecordSelect(getReactorInstSnapshot(reactorInst, 0), action.getName())
                    + ", "
                    + UclidRecordSelect(getReactorInstDelayBuffer(reactorInst), action.getName())
                    + "),");
          } // Shouldn't need to do anything if min_delay is not zero, since the value is already
          // stored in the buffer
        }
      }
      if (triggers.size() > 0) {
        code.unindent(); // Block statement
        code.pr("]),");
        code.unindent(); // ITE statement
        code.pr("),");
      }
      /** Store reactor post-state */
      code.pr("# Store reactor post-state");
      code.pr(
          "UclidAssignStmt("
              + getReactorInstSnapshot(reactorInst, postStateIndex)
              + ", "
              + reactorInstOrigName
              + "),");
      reactionData.uclCalls.add(uclCall);
    }
    code.unindent(); // Block statement
    code.pr("])");
    code.unindent(); // Procedure
    code.pr(")");
  }

  private String getReactionName(Reactor reactor, int index) {
    return reactor.getName() + "_reaction_" + (index + 1);
  }

  private String getReactorInstDelayBuffer(ReactorInstance reactorInst) {
    return reactorInst.getName() + "_delay_buffer";
  }

  private String getReactorInstCopy(ReactorInstance reactorInst, int i) {
    return reactorInst.getName() + "_" + i;
  }

  private String getReactorType(Reactor reactor) {
    return reactor.getName() + "_t";
  }

  private String getReactorInstSnapshotArray(ReactorInstance reactorInst) {
    return reactorInst.getName() + "_snapshot";
  }

  private String getReactorInstSnapshot(ReactorInstance reactorInst, int i) {
    return getReactorInstSnapshotArray(reactorInst) + "[" + i + "]";
  }

  private String getReactorInstArray(ReactorInstance reactorInst) {
    return reactorInst.getName() + "_array";
  }

  /**
   * Get the next index for the reactor instance.
   *
   * @param reactorInst The reactor instance.
   * @return The next index for the reactor instance (starts from 1).
   */
  private int getNextReactorInstIndex(ReactorInstance reactorInst) {
    int index = this.reactorInst2Index.getOrDefault(reactorInst, 1);
    this.reactorInst2Index.put(reactorInst, index + 1);
    return index;
  }

  /**
   * Get the name of the variable that indicates whether a reaction has fired.
   *
   * @param reactionInst Reaction instance that is fired
   * @return The name of the variable that indicates whether a reaction has fired.
   */
  private String getReactionFiredName(ReactionInstance reactionInst) {
    return reactionInst.getParent().getName() + "_reaction_" + (reactionInst.index + 1);
  }

  private String getReactionFiredArray(ReactionInstance reactionInst) {
    return getReactionFiredName(reactionInst) + "_array";
  }

  private String getReactionFiredInstArray(ReactionInstance reactionInst) {
    return getReactionFiredName(reactionInst) + "_insts";
  }

  private String getReactionFiredInst(ReactionInstance reactionInst, int i) {
    return getReactionFiredName(reactionInst) + "_inst_" + i;
  }

  /**
   * Get the next index for the reactor instance.
   *
   * @param reactorInst The reactor instance.
   * @return The next index for the reactor instance (starts from 1).
   */
  private int getNextReactionInstIndex(ReactionInstance reactionInst) {
    int index = this.reactionInst2Index.getOrDefault(reactionInst, 0);
    this.reactionInst2Index.put(reactionInst, index + 1);
    return index;
  }

  /** Match the C type to the Uclid type */
  private String getUclidTypeFromCType(String type, Boolean api) {
    return switch (type) {
      case "bool" -> api ? "UBool" : "boolean";
      case "int", "int32_t", "unsigned", "unsigned int", "uint32_t" ->
          api ? "UclidBVType(32)" : "bv32";
      case "int64_t", "uint64_t" -> api ? "UclidBVType(64)" : "bv64";
      case "float" -> api ? "UclidFloatType()" : "single";
      default -> throw new RuntimeException("Unsupported type: " + type);
    };
  }

  private String getUclidValueFromCValue(String value, String type) {
    return switch (type) {
      case "bool" -> value.equals("true") ? "UBoolTrue" : "UBoolFalse";
      case "int", "int32_t", "unsigned", "unsigned int", "uint32_t" ->
          "UclidBVLiteral(" + value + ", 32)";
      case "int64_t", "uint64_t" -> "UclidBVLiteral(" + value + ", 64)";
      case "float" -> "UclidFloatLiteral(" + value + ")";
      default -> throw new RuntimeException("Unsupported type: " + type);
    };
  }

  /** Helper function for record select */
  private String UclidRecordSelect(String record, String field) {
    return "UclidRecordSelect(" + record + ", \"" + field + "\")";
  }

  private String UclidArraySelect(String array, String index) {
    return "UclidArraySelect(" + array + ", [" + index + "])";
  }

  private void generatePreambles() {
    code.pr(
        String.join(
            "\n",
            "from uclid.builder import *",
            "from uclid.builder_sugar import *",
            "from polyver.ext_module import ModuleWithExternalProcedures",
            "from polyver.ext_procedure import ExternalProcedure",
            "from polyver.utils import Lang",
            "from polyver.main import getModelChecker"));
  }

  /** Generate UCLID5 using the Python API. */
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
      Path file = this.outputDir.resolve(this.tactic + "_" + this.property_name + ".py");
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
    diagram = explorer.diagram;
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

  /**
   * Compute a completeness threadhold for each property by simulating a worst-case execution by
   * traversing the reactor instance graph and building a state space diagram.
   */
  private void computeCT() {

    StateSpaceExplorer explorer = new StateSpaceExplorer(this.main);
    explorer.explore(
        new Tag(this.horizon, 0, true), true // findLoop
        );
    StateSpaceDiagram diagram = explorer.diagram;

    // Generate a dot file.
    try {
      CodeBuilder dot = diagram.generateDot();
      Path file = this.outputDir.resolve(this.tactic + "_" + this.property_name + ".dot");
      String filename = file.toString();
      dot.writeToFile(filename);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    //// Compute CT
    if (!explorer.loopFound) {
      StateSpaceNode node = diagram.head;
      this.CT = 0;
      while (node != diagram.tail) {
        this.CT += 1;
        node = diagram.getDownstreamNode(node);
        if (node == null || node.getTag().timestamp > this.horizon) {
          break;
        }
      }
    }
    // Over-approximate CT by estimating the number of loop iterations required.
    else {
      // Subtract the non-periodic logical time
      // interval from the total horizon.
      long horizonRemained = Math.subtractExact(this.horizon, diagram.loopNode.getTag().timestamp);

      // Check how many loop iteration is required
      // to check the remaining horizon.
      int loopIterations = 0;
      if (diagram.loopPeriod == 0 && horizonRemained != 0)
        throw new RuntimeException(
            "ERROR: Zeno behavior detected while the horizon is non-zero. The program has no"
                + " finite CT.");
      else if (diagram.loopPeriod == 0 && horizonRemained == 0) {
        // Handle this edge case.
        throw new RuntimeException("Unhandled case: both the horizon and period are 0!");
      } else {
        loopIterations = (int) Math.floor((double) horizonRemained / diagram.loopPeriod);
      }

      // System.out.println("horizonRemained: " + horizonRemained);
      horizonRemained =
          Math.subtractExact(
              horizonRemained, Math.multiplyExact(loopIterations, diagram.loopPeriod));
      StateSpaceNode node = diagram.loopNode, next;
      // FIXME: use safer arithmetic operations.
      this.CT =
          diagram.loopNode.getIndex()
              + (diagram.tail.getIndex() - diagram.loopNode.getIndex() + 1) * loopIterations;
      // System.out.println("loopPeriod: " + diagram.loopPeriod);
      // System.out.println("loopIterations: " + loopIterations);
      // System.out.println("loopNode: " + diagram.loopNode.getIndex());
      // System.out.println("loopNodeNext: " + diagram.loopNodeNext.getIndex());
      // System.out.println("tail: " + diagram.tail.getIndex());
      // System.out.println("CT: " + this.CT);
      // System.out.println("horizonRemained: " + horizonRemained);
      while (true) {
        next = diagram.getDownstreamNode(node);
        if (next == diagram.loopNode) {
          next = diagram.loopNodeNext;
        }
        long timeElapsed = next.getTag().timestamp - node.getTag().timestamp;
        if (horizonRemained < timeElapsed) break;
        else horizonRemained -= timeElapsed;
        this.CT += 1;
        node = next;
      }
    }
  }

  /** Process an MTL property. */
  private void processMTLSpec() {
    MTLLexer lexer = new MTLLexer(CharStreams.fromString(this.spec));
    // Print lexing results
    CommonTokenStream tokens = new CommonTokenStream(lexer);
    MTLParser parser = new MTLParser(tokens);
    MtlContext mtlCtx = parser.mtl();
    MTLVisitor visitor = new MTLVisitor(this.tactic, false);

    // The visitor transpiles the MTL into a Uclid axiom.
    this.FOLSpec = visitor.visitMtl(mtlCtx, "i", 0, "0", 0);
    System.out.println("FOLSpec: " + this.FOLSpec);
    this.horizon = visitor.getHorizon();
    System.out.println("Horizon: " + this.horizon);
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

  // FIXME: Add this to ASTUtils.java in a principled way.
  // Can typed variables be other things than ports and actions?
  private List<? extends TypedVariable> getAllInputs(Reaction reaction) {
    return reaction.getTriggers().stream()
        .filter(it -> (it instanceof VarRef))
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
