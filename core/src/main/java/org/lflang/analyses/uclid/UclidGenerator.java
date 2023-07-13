/*************
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.Target;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.c.AstUtils;
import org.lflang.analyses.c.BuildAstParseTreeVisitor;
import org.lflang.analyses.c.CAst;
import org.lflang.analyses.c.CToUclidVisitor;
import org.lflang.analyses.c.IfNormalFormAstVisitor;
import org.lflang.analyses.c.VariablePrecedenceVisitor;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.analyses.statespace.Tag;
import org.lflang.ast.ASTUtils;
import org.lflang.dsl.CLexer;
import org.lflang.dsl.CParser;
import org.lflang.dsl.CParser.BlockItemListContext;
import org.lflang.dsl.MTLLexer;
import org.lflang.dsl.MTLParser;
import org.lflang.dsl.MTLParser.MtlContext;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.StateVariableInstance;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.TimerInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Time;
import org.lflang.util.StringUtil;

/** (EXPERIMENTAL) Generator for Uclid5 models. */
public class UclidGenerator extends GeneratorBase {

  ////////////////////////////////////////////
  //// Public fields
  /** A list of reaction runtime instances. */
  public List<ReactionInstance.Runtime> reactionInstances =
      new ArrayList<ReactionInstance.Runtime>();

  /** A list of action instances */
  public List<ActionInstance> actionInstances = new ArrayList<ActionInstance>();

  /** Joint lists of the lists above. */
  public List<TriggerInstance> triggerInstances; // Triggers = ports + actions + timers

  public List<NamedInstance> namedInstances; // Named instances = triggers + state variables

  /** A list of paths to the uclid files generated */
  public List<Path> generatedFiles = new ArrayList<>();

  public Map<Path, String> expectations = new HashMap<>();

  /** The directory where the generated files are placed */
  public Path outputDir;

  /** A runner for the generated Uclid files */
  public UclidRunner runner;

  /** Completeness threshold */
  public int CT = 0;

  ////////////////////////////////////////////
  //// Private fields
  /** A list of reactor runtime instances. */
  private List<ReactorInstance> reactorInstances = new ArrayList<ReactorInstance>();

  /** State variables in the system */
  private List<StateVariableInstance> stateVariables = new ArrayList<StateVariableInstance>();

  /** A list of input port instances */
  private List<PortInstance> inputInstances = new ArrayList<PortInstance>();

  /** A list of output port instances */
  private List<PortInstance> outputInstances = new ArrayList<PortInstance>();

  /** A list of input AND output port instances */
  private List<PortInstance> portInstances = new ArrayList<PortInstance>();

  /** A list of timer instances */
  private List<TimerInstance> timerInstances = new ArrayList<TimerInstance>();

  /** A list of MTL properties represented in Attributes. */
  private List<Attribute> properties;

  /** The main place to put generated code. */
  private CodeBuilder code = new CodeBuilder();

  /** Strings from the property attribute */
  private String name;

  /**
   * A tactic used to verify properties. Currently, only BMC (bounded model checking) is functional.
   * FIXME: For a future version that supports multiple tactics, the tactics should be stored in a
   * list.
   */
  enum Tactic {
    BMC,
    INDUCTION
  }

  private Tactic tactic;

  /** Safety MTL property to be verified */
  private String spec;

  /** A property's ground truth value, for debugging the verifier */
  private String expect;

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
  private boolean logicalTimeBased = false;

  /** Constructor */
  public UclidGenerator(LFGeneratorContext context, List<Attribute> properties) {
    super(context);
    this.properties = properties;
    this.runner = new UclidRunner(this);
  }

  ////////////////////////////////////////////////////////////
  //// Public methods
  public void doGenerate(Resource resource, LFGeneratorContext context) {

    // Reuse parts of doGenerate() from GeneratorBase.
    super.printInfo(context.getMode());
    ASTUtils.setMainName(context.getFileConfig().resource, context.getFileConfig().name);
    super.createMainInstantiation();
    super.setReactorsAndInstantiationGraph(context.getMode());

    // Create the main reactor instance if there is a main reactor.
    this.main =
        ASTUtils.createMainReactorInstance(mainDef, reactors, messageReporter, targetConfig);

    // Extract information from the named instances.
    populateDataStructures();

    // Create the src-gen directory
    setupDirectories();

    // Generate a Uclid model for each property.
    for (Attribute prop : this.properties) {
      this.name =
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
  }

  ////////////////////////////////////////////////////////////
  //// Protected methods

  /** Generate the Uclid model. */
  protected void generateUclidFile() {
    try {
      // Generate main.ucl and print to file
      code = new CodeBuilder();
      Path file = this.outputDir.resolve(this.tactic + "_" + this.name + ".ucl");
      String filename = file.toString();
      generateUclidCode();
      code.writeToFile(filename);
      this.generatedFiles.add(file);
      if (this.expect != null) this.expectations.put(file, this.expect);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** The main function that generates Uclid code. */
  protected void generateUclidCode() {
    code.pr(
        String.join(
            "\n",
            "/*******************************",
            " * Auto-generated UCLID5 model *",
            " ******************************/"));

    code.pr("module main {");
    code.indent();

    // Timing semantics
    generateTimingSemantics();

    // Trace definition
    generateTraceDefinition();

    // Reaction IDs and state variables
    generateReactionIdsAndStateVars();

    // Reactor semantics
    generateReactorSemantics();

    // Triggers and reactions
    generateTriggersAndReactions();

    // Initial Condition
    generateInitialConditions();

    // Reaction bodies
    generateReactionAxioms();

    // Properties
    generateProperty();

    // Control block
    generateControlBlock();

    code.unindent();
    code.pr("}");
  }

  /** Macros for timing semantics */
  protected void generateTimingSemantics() {
    code.pr(
        String.join(
            "\n",
            "/*******************************",
            " * Time and Related Operations *",
            " ******************************/",
            "type timestamp_t = integer;                     // The unit is nanoseconds",
            "type microstep_t = integer;",
            "type tag_t = {",
            "    timestamp_t,",
            "    microstep_t",
            "};",
            "type interval_t  = tag_t;",
            "",
            "// Projection macros",
            "define pi1(t : tag_t) : timestamp_t = t._1;     // Get timestamp from tag",
            "define pi2(t : tag_t) : microstep_t = t._2;     // Get microstep from tag",
            "",
            "// Interval constructor",
            "define zero() : interval_t",
            "= {0, 0};",
            "define startup() : interval_t",
            "= zero();",
            "define mstep() : interval_t",
            "= {0, 1};",
            "define nsec(t : integer) : interval_t",
            "= {t, 0};",
            "define usec(t : integer) : interval_t",
            "= {t * 1000, 0};",
            "define msec(t : integer) : interval_t",
            "= {t * 1000000, 0};",
            "define sec(t : integer) : interval_t",
            "= {t * 1000000000, 0};",
            "define inf() : interval_t",
            "= {-1, 0};",
            "",
            "// Helper function",
            "define isInf(i : interval_t) : boolean",
            "= pi1(i) < 0;",
            "",
            "// Tag comparison",
            "define tag_later(t1 : tag_t, t2 : tag_t) : boolean",
            "= pi1(t1) > pi1(t2)",
            "    || (pi1(t1) == pi1(t2) && pi2(t1) > pi2(t2))",
            "    || (isInf(t1) && !isInf(t2));",
            "",
            "define tag_same(t1 : tag_t, t2 : tag_t) : boolean",
            "= t1 == t2;",
            "",
            "define tag_earlier(t1 : tag_t, t2 : tag_t) : boolean",
            "= pi1(t1) < pi1(t2)",
            "    || (pi1(t1) == pi1(t2) && pi2(t1) < pi2(t2))",
            "    || (!isInf(t1) && isInf(t2));",
            "",
            "// Used for incrementing a tag through an action",
            "define tag_schedule(t : tag_t, i : integer) : tag_t",
            "= if (i == 0) then { pi1(t), pi2(t)+1 } else { pi1(t)+i, 0 };",
            "",
            "// Used for incrementing a tag along a connection",
            "define tag_delay(t : tag_t, i : integer) : tag_t",
            "= if (i == 0) then { pi1(t), pi2(t) } else { pi1(t)+i, 0 };",
            "",
            "// Only consider timestamp for now.",
            "define tag_diff(t1, t2: tag_t) : interval_t",
            "= if (!isInf(t1) && !isInf(t2))",
            "    then { pi1(t1) - pi1(t2), pi2(t1) - pi2(t2) }",
            "    else inf();",
            "\n"));
  }

  /** Macros, type definitions, and variable declarations for trace (path) */
  protected void generateTraceDefinition() {
    //// Why do we have a padding at the end of the trace?
    //
    // To handle bounded traces for a potentially unbounded execution
    // (due to, for example, timers or systems forming a loop),
    // we need to let certain axioms "terminate" the execution and
    // put any "spilled-over" states in the trace padding.
    //
    // For example, an axiom could say "if a reaction is triggered
    // at step i, it schedules an action that appears 1 sec later
    // in some future step." Assuming our completeness threshold is 10,
    // this axiom can block the model (i.e. returns TRIVIALLY true)
    // at step 10 because there are not any more steps in the bounded trace.
    // To avoid this, a padding of the size of the number of reactions
    // is added to the trace. In addition, an antecedent of the form
    // "i >= START && i <= END" is added to all the axioms. The padding
    // will store all the spilled-over states in order to prevent
    // model blocking.
    int traceEndIndex = this.CT + this.reactionInstances.size();

    // Define various constants.
    code.pr(
        String.join(
            "\n",
            "/********************",
            " * Trace Definition *",
            " *******************/",
            "const START : integer = 0; // The start index of the trace.",
            "const END : integer = "
                + String.valueOf(this.CT)
                + "; // The end index of the trace (without padding)",
            "const END_TRACE : integer = "
                + String.valueOf(traceEndIndex)
                + "; // The end index of the trace with padding",
            "\n"));

    // Define trace indices as a group,
    // so that we can use finite quantifiers.
    code.pr("group indices : integer = {");
    code.indent();
    String indices = "";
    for (int i = 0; i <= traceEndIndex; i++) {
      indices += String.valueOf(i) + (i == traceEndIndex ? "" : ", ");
    }
    code.pr(indices);
    code.unindent();
    code.pr("};\n\n");

    // Define step, event, and trace types.
    code.pr(
        String.join(
            "\n",
            "// Define step and event types.",
            "type step_t = integer;",
            "type event_t = { rxn_t, tag_t, state_t, trigger_t, sched_t, payload_t };",
            "",
            "// Create a bounded trace with length " + String.valueOf(this.CT)));
    code.pr("// Potentially unbounded trace, we bound this later.");
    code.pr("type trace_t  = [integer]event_t;");

    // Declare start time and trace.
    code.pr(
        String.join(
            "\n",
            "// Declare start time.",
            "var start_time : timestamp_t;",
            "",
            "// Declare trace.",
            "var trace : trace_t;"));

    // Start generating helper macros.
    code.pr(String.join("\n", "/*****************", " * Helper Macros *", " ****************/"));

    // Define a getter for uclid arrays.
    String initialReactions = "";
    if (this.reactionInstances.size() > 0) {
      initialReactions = "false, ".repeat(this.reactionInstances.size());
      initialReactions = initialReactions.substring(0, initialReactions.length() - 2);
    } else {
      // Initialize a dummy variable just to make the code compile.
      initialReactions = "false";
    }
    initialReactions = "{" + initialReactions + "}";
    String initialStates = "";
    if (this.namedInstances.size() > 0) {
      initialStates = "0, ".repeat(this.namedInstances.size());
      initialStates = initialStates.substring(0, initialStates.length() - 2);
    } else {
      // Initialize a dummy variable just to make the code compile.
      initialStates = "0";
    }
    String initialTriggerPresence = "";
    if (this.triggerInstances.size() > 0) {
      initialTriggerPresence = "false, ".repeat(this.triggerInstances.size());
      initialTriggerPresence =
          initialTriggerPresence.substring(0, initialTriggerPresence.length() - 2);
    } else {
      // Initialize a dummy variable just to make the code compile.
      initialTriggerPresence = "false";
    }
    String initialActionsScheduled = "";
    if (this.actionInstances.size() > 0) {
      initialActionsScheduled = "false, ".repeat(this.actionInstances.size());
      initialActionsScheduled =
          initialActionsScheduled.substring(0, initialActionsScheduled.length() - 2);
    } else {
      // Initialize a dummy variable just to make the code compile.
      initialActionsScheduled = "false";
    }
    String initialActionsScheduledPayload = "";
    if (this.actionInstances.size() > 0) {
      initialActionsScheduledPayload = "0, ".repeat(this.actionInstances.size());
      initialActionsScheduledPayload =
          initialActionsScheduledPayload.substring(0, initialActionsScheduledPayload.length() - 2);
    } else {
      // Initialize a dummy variable just to make the code compile.
      initialActionsScheduledPayload = "0";
    }
    code.pr("// Helper macro that returns an element based on index.");
    code.pr("define get(tr : trace_t, i : step_t) : event_t =");
    code.pr("if (i >= START && i <= END_TRACE) then tr[i] else");
    code.pr(
        "{ "
            + initialReactions
            + ", inf(), { "
            + initialStates
            + " }, { "
            + initialTriggerPresence
            + " }, {"
            + initialActionsScheduled
            + " }, {"
            + initialActionsScheduledPayload
            + "} };");

    // Define an event getter from the trace.
    code.pr(
        String.join(
            "\n",
            "define elem(i : step_t) : event_t",
            "= get(trace, i);",
            "",
            "// projection macros",
            "define rxn      (i : step_t) : rxn_t        = elem(i)._1;",
            "define g        (i : step_t) : tag_t        = elem(i)._2;",
            "define s        (i : step_t) : state_t      = elem(i)._3;",
            "define t        (i : step_t) : trigger_t    = elem(i)._4;",
            "define d        (i : step_t) : sched_t      = elem(i)._5;",
            "define pl       (i : step_t) : payload_t    = elem(i)._6;",
            "define isNULL   (i : step_t) : boolean      = rxn(i) == " + initialReactions + ";"));
  }

  /** Type definitions for runtime reaction Ids and state variables */
  protected void generateReactionIdsAndStateVars() {
    // Encode the components and the logical delays
    // present in a reactor system.
    code.pr(
        String.join(
            "\n",
            "/**********************************",
            " * Reaction IDs & State Variables *",
            " *********************************/",
            "",
            "//////////////////////////",
            "// Application Specific"));

    // Enumerate over all reactions.
    code.pr(String.join("\n", "// Reactions", "type rxn_t = {"));
    code.indent();
    for (var i = 0; i < this.reactionInstances.size(); i++) {
      // Print a list of reaction IDs.
      // Add a comma if not last.
      code.pr(
          "boolean"
              + ((i == this.reactionInstances.size() - 1) ? "" : ",")
              + "\t// "
              + this.reactionInstances.get(i));
    }
    code.unindent();
    code.pr("};\n");

    // Generate projection macros.
    code.pr("// Reaction projection macros");
    for (var i = 0; i < this.reactionInstances.size(); i++) {
      code.pr(
          "define "
              + this.reactionInstances.get(i).getReaction().getFullNameWithJoiner("_")
              + "(n : rxn_t) : boolean = n._"
              + (i + 1)
              + ";");
    }
    code.pr("\n"); // Newline

    // State variables and triggers
    // FIXME: expand to data types other than integer
    code.pr("// State variables and triggers");
    code.pr("type state_t = {");
    code.indent();
    if (this.namedInstances.size() > 0) {
      for (var i = 0; i < this.namedInstances.size(); i++) {
        code.pr(
            "integer"
                + ((i == this.namedInstances.size() - 1) ? "" : ",")
                + "\t// "
                + this.namedInstances.get(i));
      }
    } else {
      code.pr(
          String.join(
              "\n",
              "// There are no ports or state variables.",
              "// Insert a dummy integer to make the model compile.",
              "integer"));
    }
    code.unindent();
    code.pr("};");
    code.pr("// State variable projection macros");
    for (var i = 0; i < this.namedInstances.size(); i++) {
      code.pr(
          "define "
              + this.namedInstances.get(i).getFullNameWithJoiner("_")
              + "(s : state_t) : integer = s._"
              + (i + 1)
              + ";");
    }
    code.pr("\n"); // Newline

    // A boolean tuple indicating whether triggers are present.
    code.pr("// A boolean tuple indicating whether triggers are present.");
    code.pr("type trigger_t = {");
    code.indent();
    if (this.triggerInstances.size() > 0) {
      for (var i = 0; i < this.triggerInstances.size(); i++) {
        code.pr(
            "boolean"
                + ((i == this.triggerInstances.size() - 1) ? "" : ",")
                + "\t// "
                + this.triggerInstances.get(i));
      }
    } else {
      code.pr(
          String.join(
              "\n",
              "// There are no triggers.",
              "// Insert a dummy boolean to make the model compile.",
              "boolean"));
    }
    code.unindent();
    code.pr("};");
    code.pr("// Trigger presence projection macros");
    for (var i = 0; i < this.triggerInstances.size(); i++) {
      code.pr(
          "define "
              + this.triggerInstances.get(i).getFullNameWithJoiner("_")
              + "_is_present"
              + "(t : trigger_t) : boolean = t._"
              + (i + 1)
              + ";");
    }

    // A boolean tuple indicating whether actions are scheduled by reactions
    // at the instant when they are triggered.
    code.pr(
        String.join(
            "\n",
            "// A boolean tuple indicating whether actions are scheduled by reactions",
            "// at the instant when they are triggered."));
    code.pr("type sched_t = {");
    code.indent();
    if (this.actionInstances.size() > 0) {
      for (var i = 0; i < this.actionInstances.size(); i++) {
        code.pr(
            "boolean"
                + ((i == this.actionInstances.size() - 1) ? "" : ",")
                + "\t// "
                + this.actionInstances.get(i));
      }
    } else {
      code.pr(
          String.join(
              "\n",
              "// There are no actions.",
              "// Insert a dummy boolean to make the model compile.",
              "boolean"));
    }
    code.unindent();
    code.pr("};");
    code.pr("// Projection macros for action schedule flag");
    for (var i = 0; i < this.actionInstances.size(); i++) {
      code.pr(
          "define "
              + this.actionInstances.get(i).getFullNameWithJoiner("_")
              + "_scheduled"
              + "(d : sched_t) : boolean = d._"
              + (i + 1)
              + ";");
    }

    // A integer tuple indicating the integer payloads scheduled by reactions
    // at the instant when they are triggered.
    code.pr(
        String.join(
            "\n",
            "// A integer tuple indicating the integer payloads scheduled by reactions",
            "// at the instant when they are triggered."));
    code.pr("type payload_t = {");
    code.indent();
    if (this.actionInstances.size() > 0) {
      for (var i = 0; i < this.actionInstances.size(); i++) {
        code.pr(
            "integer"
                + ((i == this.actionInstances.size() - 1) ? "" : ",")
                + "\t// "
                + this.actionInstances.get(i));
      }
    } else {
      code.pr(
          String.join(
              "\n",
              "// There are no actions.",
              "// Insert a dummy integer to make the model compile.",
              "integer"));
    }
    code.unindent();
    code.pr("};");
    code.pr("// Projection macros for scheduled payloads");
    for (var i = 0; i < this.actionInstances.size(); i++) {
      code.pr(
          "define "
              + this.actionInstances.get(i).getFullNameWithJoiner("_")
              + "_scheduled_payload"
              + "(payload : payload_t) : integer = payload._"
              + (i + 1)
              + ";");
    }
  }

  /** Axioms for reactor semantics */
  protected void generateReactorSemantics() {
    code.pr(
        String.join(
            "\n",
            "/*********************",
            " * Reactor Semantics *",
            " *********************/",
            ""));

    // Non-federated "happened-before"
    code.pr(
        String.join(
            "\n",
            "// Non-federated \"happened-before\"",
            "define hb(e1, e2 : event_t) : boolean",
            "= tag_earlier(e1._2, e2._2)"));
    if (!this.logicalTimeBased) {
      code.indent();
      // Get happen-before relation between two reactions.
      code.pr("|| (tag_same(e1._2, e2._2) && ( false");
      // Iterate over reactions based on upstream/downstream relations.
      for (var upstreamRuntime : this.reactionInstances) {
        var downstreamReactions = upstreamRuntime.getReaction().dependentReactions();
        for (var downstream : downstreamReactions) {
          // If the downstream reaction and the upstream
          // reaction are in the same reactor, skip, since
          // they will be accounted for in the for loop below.
          if (downstream.getParent().equals(upstreamRuntime.getReaction().getParent())) continue;
          for (var downstreamRuntime : downstream.getRuntimeInstances()) {
            code.pr(
                "|| ("
                    + upstreamRuntime.getReaction().getFullNameWithJoiner("_")
                    + "(e1._1)"
                    + " && "
                    + downstreamRuntime.getReaction().getFullNameWithJoiner("_")
                    + "(e2._1)"
                    + ")");
          }
        }
      }
      // Iterate over reactions based on priorities.
      for (var reactor : this.reactorInstances) {
        for (int i = 0; i < reactor.reactions.size(); i++) {
          for (int j = i + 1; j < reactor.reactions.size(); j++) {
            code.pr(
                "|| ("
                    + reactor.reactions.get(i).getFullNameWithJoiner("_")
                    + "(e1._1)"
                    + " && "
                    + reactor.reactions.get(j).getFullNameWithJoiner("_")
                    + "(e2._1)"
                    + ")");
          }
        }
      }
      code.unindent();
      code.pr("))");
    }
    code.pr(";");

    code.pr(
        String.join(
            "\n",
            "/** transition relation **/",
            "// transition relation constrains future states",
            "// based on previous states.",
            "",
            "// Events are ordered by \"happened-before\" relation.",
            "axiom(finite_forall (i : integer) in indices :: (i >= START && i <= END_TRACE) ==>"
                + " (finite_forall (j : integer) in indices ::",
            "    (j >= START && j <= END_TRACE) ==> (hb(elem(i), elem(j)) ==> i < j)));",
            "",
            "// Tags should be non-negative.",
            "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE)",
            "    ==> pi1(g(i)) >= 0);",
            "",
            "// Microsteps should be non-negative.",
            "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE)",
            "    ==> pi2(g(i)) >= 0);",
            "",
            "// Begin the frame at the start time specified.",
            "define start_frame(i : step_t) : boolean =",
            "    (tag_same(g(i), {start_time, 0}) || tag_later(g(i), {start_time, 0}));",
            "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE)",
            "    ==> start_frame(i));",
            "",
            "// NULL events should appear in the suffix, except for START.",
            "axiom(finite_forall (j : integer) in indices :: (j > START && j <= END_TRACE) ==> (",
            "    !isNULL(j)) ==> ",
            "        (finite_forall (i : integer) in indices :: (i > START && i < j) ==>"
                + " (!isNULL(i))));",
            ""));

    //// Axioms for the event-based semantics
    if (!this.logicalTimeBased) {
      code.pr("//// Axioms for the event-based semantics");

      // the same event can only trigger once in a logical instant
      code.pr(
          String.join(
              "\n",
              "// the same event can only trigger once in a logical instant",
              "axiom(finite_forall (i : integer) in indices :: (i >= START && i <= END_TRACE) ==>"
                  + " (finite_forall (j : integer) in indices ::",
              "    (j >= START && j <= END_TRACE) ==> ((rxn(i) == rxn(j) && i != j)",
              "        ==> !tag_same(g(i), g(j)))));",
              ""));

      // Only one reaction gets triggered at a time.
      ArrayList<String> reactionsStatus = new ArrayList<>();
      for (int i = 0; i < this.reactionInstances.size(); i++) reactionsStatus.add("false");
      code.pr(
          String.join(
              "\n",
              "// Only one reaction gets triggered at a time.",
              "axiom(finite_forall (i : integer) in indices :: (i >= START && i <= END_TRACE) ==>"
                  + " (",
              "    isNULL(i)"));
      code.indent();
      for (int i = 0; i < this.reactionInstances.size(); i++) {
        String[] li = reactionsStatus.toArray(String[]::new);
        li[i] = "true";
        code.pr("|| rxn(i) == " + "{" + String.join(", ", li) + "}");
      }
      code.unindent();
      code.pr("));");
    }
  }

  /** Axioms for all triggers */
  protected void generateTriggersAndReactions() {
    generateConnectionAxioms();
    generateActionAxioms();
    generateTimerAxioms();
    generateReactionTriggerAxioms();
  }

  /** Generate axiomatic semantics for connections */
  protected void generateConnectionAxioms() {
    code.pr(String.join("\n", "/***************", " * Connections *", " ***************/"));
    // FIXME: Support banks and multiports. Figure out how to work with ranges.
    // Iterate over all the ports. Generate an axiom for each connection
    // (i.e. a pair of input-output ports).
    // A "range" holds the connection information.
    // See connectPortInstances() in ReactorInstance.java for more details.
    for (var port : this.portInstances) {
      for (SendRange range : port.getDependentPorts()) {
        PortInstance source = range.instance;
        Connection connection = range.connection;
        List<RuntimeRange<PortInstance>> destinations = range.destinations;

        // Extract delay value
        long delay = 0;
        if (connection.getDelay() != null) {
          // Somehow delay is an Expression,
          // which makes it hard to convert to nanoseconds.
          Expression delayExpr = connection.getDelay();
          if (delayExpr instanceof Time) {
            long interval = ((Time) delayExpr).getInterval();
            String unit = ((Time) delayExpr).getUnit();
            TimeValue timeValue = new TimeValue(interval, TimeUnit.fromName(unit));
            delay = timeValue.toNanoSeconds();
          }
        }

        for (var portRange : destinations) {
          var destination = portRange.instance;

          // We have extracted the source, destination, and connection AST node.
          // Now we are ready to generate an axiom for this triple.
          code.pr(
              "// "
                  + source.getFullNameWithJoiner("_")
                  + " "
                  + (connection.isPhysical() ? "~>" : "->")
                  + " "
                  + destination.getFullNameWithJoiner("_"));
          code.pr(
              String.join(
                  "\n",
                  "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END) ==> (",
                  "// If "
                      + source.getFullNameWithJoiner("_")
                      + " is present, then "
                      + destination.getFullNameWithJoiner("_")
                      + " will be present.",
                  "// with the same value after some fixed delay of " + delay + " nanoseconds.",
                  "(" + source.getFullNameWithJoiner("_") + "_is_present" + "(t(i)) ==> ((",
                  "    finite_exists (j : integer) in indices :: j > i && j <= END_TRACE",
                  "    && " + destination.getFullNameWithJoiner("_") + "_is_present" + "(t(j))",
                  "    && "
                      + destination.getFullNameWithJoiner("_")
                      + "(s(j)) == "
                      + source.getFullNameWithJoiner("_")
                      + "(s(i))",
                  connection.isPhysical() ? "" : "&& g(j) == tag_delay(g(i), " + delay + ")",
                  ")",
                  ")) // Closes ("
                      + source.getFullNameWithJoiner("_")
                      + "_is_present"
                      + "(t(i)) ==> ((.",
                  //// Separator
                  "// If "
                      + destination.getFullNameWithJoiner("_")
                      + " is present, there exists an "
                      + source.getFullNameWithJoiner("_")
                      + " in prior steps.",
                  "// This additional term establishes a one-to-one relationship between two ports'"
                      + " signals.",
                  "&& (" + destination.getFullNameWithJoiner("_") + "_is_present" + "(t(i)) ==> (",
                  "    finite_exists (j : integer) in indices :: j >= START && j < i",
                  "    && " + source.getFullNameWithJoiner("_") + "_is_present" + "(t(j))",
                  "    && "
                      + source.getFullNameWithJoiner("_")
                      + "(s(j)) == "
                      + destination.getFullNameWithJoiner("_")
                      + "(s(i))",
                  connection.isPhysical() ? "" : "    && g(i) == tag_delay(g(j), " + delay + ")",
                  ")) // Closes the one-to-one relationship.",
                  "));"));

          // If destination is not present, then its value resets to 0.
          code.pr(
              String.join(
                  "\n",
                  "// If "
                      + destination.getFullNameWithJoiner("_")
                      + " is not present, then its value resets to 0.",
                  "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE &&"
                      + " !isNULL(i)) ==> (",
                  "    (!"
                      + destination.getFullNameWithJoiner("_")
                      + "_is_present"
                      + "(t(i)) ==> (",
                  "        " + destination.getFullNameWithJoiner("_") + "(s(i)) == 0",
                  "    ))",
                  "));"));
        }
      }
    }
  }

  /** Generate axiomatic semantics for actions */
  protected void generateActionAxioms() {
    if (this.actionInstances.size() > 0) {
      code.pr(String.join("\n", "/***********", " * Actions *", " ***********/"));
      for (var action : this.actionInstances) {
        Set<ReactionInstance> dependsOnReactions = action.getDependsOnReactions();
        String comment =
            "If "
                + action.getFullNameWithJoiner("_")
                + " is present, these reactions could schedule it: ";
        String triggerStr = "";
        for (var reaction : dependsOnReactions) {
          comment += reaction.getFullNameWithJoiner("_") + ", ";
          triggerStr +=
              String.join(
                  "\n",
                  "// " + reaction.getFullNameWithJoiner("_"),
                  // OR because only any present trigger can trigger the reaction.
                  "|| (" + action.getFullNameWithJoiner("_") + "_is_present" + "(t(i)) ==> (",
                  "    finite_exists (j : integer) in indices :: j >= START && j < i",
                  "    && " + reaction.getFullNameWithJoiner("_") + "(rxn(j))",
                  "    && g(i) == tag_schedule(g(j), " + action.getMinDelay().toNanoSeconds() + ")",
                  "    && " + action.getFullNameWithJoiner("_") + "_scheduled" + "(d(j))",
                  "    && "
                      + action.getFullNameWithJoiner("_")
                      + "(s(i))"
                      + " == "
                      + action.getFullNameWithJoiner("_")
                      + "_scheduled_payload"
                      + "(pl(j))",
                  "))");
        }

        // After populating the string segments,
        // print the generated code string.
        code.pr(
            String.join(
                "\n",
                "// " + comment,
                "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE) ==>"
                    + " ( false",
                triggerStr,
                "));"));

        // If the action is not present, then its value resets to 0.
        code.pr(
            String.join(
                "\n",
                "// If "
                    + action.getFullNameWithJoiner("_")
                    + "  is not present, then its value resets to 0.",
                "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE &&"
                    + " !isNULL(i)) ==> (",
                "    (!" + action.getFullNameWithJoiner("_") + "_is_present" + "(t(i)) ==> (",
                "        " + action.getFullNameWithJoiner("_") + "(s(i)) == 0",
                "    ))",
                "));"));
      }
    }
  }

  /** Generate axiomatic semantics for timers */
  protected void generateTimerAxioms() {
    if (this.timerInstances.size() > 0) {
      code.pr(String.join("\n", "/**********", " * Timers *", " **********/"));

      for (var timer : this.timerInstances) {
        long offset = timer.getOffset().toNanoSeconds();
        long period = timer.getPeriod().toNanoSeconds();

        code.pr("//// Axioms for " + timer.getFullName());

        // An initial firing at {offset, 0}
        code.pr(
            String.join(
                "\n",
                "// " + timer.getFullName() + ": an initial firing at (" + offset + ", 0)",
                "axiom(",
                "    ((pi1(g(END)) >= " + offset + ") ==> (",
                "        finite_exists (j : integer) in indices :: (j > START && j <= END)",
                "            && " + timer.getFullNameWithJoiner("_") + "_is_present(t(j))",
                "            && tag_same(g(j), {" + offset + ", 0})",
                "     ))",
                "     && ((pi1(g(END)) < " + offset + ") ==> (",
                "        finite_forall (i : integer) in indices :: (i > START && i <= END)",
                "            ==> (!isNULL(i))",
                "     ))",
                ");"));

        // Schedule subsequent firings.
        code.pr(
            String.join(
                "\n",
                "// "
                    + timer.getFullName()
                    + ": schedule subsequent firings every "
                    + period
                    + " ns",
                "axiom(",
                "    finite_forall (i : integer) in indices :: (i >= START && i <= END) ==> (",
                "        " + timer.getFullNameWithJoiner("_") + "_is_present(t(i)) ==> (",
                "            (",
                "                finite_exists (j : integer) in indices :: (j >= START && j > i)",
                "                    && " + timer.getFullNameWithJoiner("_") + "_is_present(t(j))",
                "                    && (g(j) == tag_schedule(g(i), " + period + "))",
                "            )",
                "        )",
                "    )",
                ");"));

        // All firings must be evenly spaced out.
        code.pr(
            String.join(
                "\n",
                "// " + timer.getFullName() + ": all firings must be evenly spaced out.",
                "axiom(",
                "    finite_forall (i : integer) in indices :: (i >= START && i <= END) ==> (",
                "        " + timer.getFullNameWithJoiner("_") + "_is_present(t(i)) ==> (",
                "            // Timestamp must be offset + n * period.",
                "            (",
                "                exists (n : integer) :: (",
                "                    n >= 0 &&",
                "                    pi1(g(i)) == " + offset + " + n * " + period,
                "                )",
                "            )",
                "            // Microstep must be 0.",
                "            && (pi2(g(i)) == 0)",
                "        )",
                "    )",
                ");"));
      }
    }
  }

  /** Axioms for encoding how reactions are triggered. */
  protected void generateReactionTriggerAxioms() {
    code.pr(
        String.join(
            "\n",
            "/********************************",
            " * Reactions and Their Triggers *",
            " ********************************/"));
    // Iterate over all reactions, generate conditions for them
    // to be triggered.
    outerLoop:
    for (ReactionInstance.Runtime reaction : this.reactionInstances) {
      String str =
          String.join(
              "\n",
              "// "
                  + reaction.getReaction().getFullNameWithJoiner("_")
                  + " is invoked when any of it triggers are present.",
              "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END_TRACE) ==>"
                  + " ((",
              "    false");

      // Iterate over the triggers of the reaction.
      for (TriggerInstance trigger : reaction.getReaction().triggers) {
        String triggerPresentStr = "";

        if (trigger.isStartup()) {
          // FIXME: Treat startup as a variable.
          // Handle startup.
          code.pr(
              String.join(
                  "\n",
                  "// If startup is within frame (and all variables have default values),",
                  "// " + reaction.getReaction().getFullNameWithJoiner("_") + " must be invoked.",
                  "axiom(",
                  "    ((start_time == 0) ==> (",
                  "        finite_exists (i : integer) in indices :: i > START && i <= END",
                  "            && "
                      + reaction.getReaction().getFullNameWithJoiner("_")
                      + "(rxn(i))"
                      + " && tag_same(g(i), zero())",
                  "            && !(",
                  "            finite_exists (j : integer) in indices :: j > START && j <= END",
                  "            && "
                      + reaction.getReaction().getFullNameWithJoiner("_")
                      + "(rxn(j))",
                  "            && j != i",
                  "            )",
                  "    ))",
                  ");"));
          continue outerLoop;
        } else if (trigger.isShutdown()) {
          // FIXME: For a future version.
          System.out.println("Not implemented!");
        } else {
          // If the trigger is a port/action/timer.
          triggerPresentStr = trigger.getFullNameWithJoiner("_") + "_is_present" + "(t(i))";
        }

        // Check if the trigger triggers other reactions.
        // If so, we need to assert in the axioms that
        // the other reactions are excluded (not invoked),
        // to preserve the interleaving semantics.
        String exclusion = "";
        for (var instance : trigger.getDependentReactions()) {
          for (var runtime : ((ReactionInstance) instance).getRuntimeInstances()) {
            if (runtime == reaction) continue; // Skip the current reaction.
            exclusion += " && !" + runtime.getReaction().getFullNameWithJoiner("_") + "(rxn(i))";
          }
        }

        str += "\n|| (" + triggerPresentStr + exclusion + ")";
      }

      // If any of the above trigger is present, then trigger the reaction.
      str += "\n) <==> (" + reaction.getReaction().getFullNameWithJoiner("_") + "(rxn(i))" + ")));";
      code.pr(str);
    }
  }

  /** Macros for initial conditions */
  protected void generateInitialConditions() {
    code.pr(
        String.join(
            "\n",
            "/*********************",
            " * Initial Condition *",
            " *********************/",
            "define initial_condition() : boolean",
            "= start_time == 0",
            "    && isNULL(0)",
            "    && g(0) == {0, 0}"));
    code.indent();
    for (var v : this.stateVariables) {
      code.pr("&& " + v.getFullNameWithJoiner("_") + "(s(0))" + " == " + "0");
    }
    for (var t : this.triggerInstances) {
      code.pr("&& " + t.getFullNameWithJoiner("_") + "(s(0))" + " == " + "0");
    }
    for (var t : this.triggerInstances) {
      code.pr("&& !" + t.getFullNameWithJoiner("_") + "_is_present" + "(t(0))");
    }
    for (var d : this.actionInstances) {
      code.pr("&& !" + d.getFullNameWithJoiner("_") + "_scheduled" + "(d(0))");
    }
    code.unindent();
    code.pr(";\n");
  }

  /** Lift reaction bodies into Uclid axioms. */
  protected void generateReactionAxioms() {
    code.pr(String.join("\n", "/*************", " * Reactions *", " *************/"));

    for (ReactionInstance.Runtime reaction : this.reactionInstances) {
      String body = reaction.getReaction().getDefinition().getCode().getBody();

      // Generate a parse tree.
      CLexer lexer = new CLexer(CharStreams.fromString(body));
      CommonTokenStream tokens = new CommonTokenStream(lexer);
      CParser parser = new CParser(tokens);
      BlockItemListContext parseTree = parser.blockItemList();

      // Build an AST.
      BuildAstParseTreeVisitor buildAstVisitor = new BuildAstParseTreeVisitor(messageReporter);
      CAst.AstNode ast = buildAstVisitor.visitBlockItemList(parseTree);

      // VariablePrecedenceVisitor
      VariablePrecedenceVisitor precVisitor = new VariablePrecedenceVisitor();
      precVisitor.visit(ast);

      // Convert the AST to If Normal Form (INF).
      IfNormalFormAstVisitor infVisitor = new IfNormalFormAstVisitor();
      infVisitor.visit(ast, new ArrayList<CAst.AstNode>());
      CAst.StatementSequenceNode inf = infVisitor.INF;

      // For the variables that are USED inside this reaction, extract the conditions
      // for setting them, and take the negation of their conjunction
      // to get the condition for maintaining their values.
      List<StateVariableInstance> unusedStates = new ArrayList<>(this.stateVariables);
      List<PortInstance> unusedOutputs = new ArrayList<>(this.outputInstances);
      List<ActionInstance> unusedActions = new ArrayList<>(this.actionInstances);
      HashMap<NamedInstance, List<CAst.AstNode>> defaultBehaviorConditions = new HashMap<>();
      for (CAst.AstNode node : inf.children) {
        CAst.IfBlockNode ifBlockNode = (CAst.IfBlockNode) node;
        // Under the INF form, a C statement is
        // the THEN branch of an IF block.
        CAst.AstNode stmt = ((CAst.IfBodyNode) ifBlockNode.right).left;
        NamedInstance instance = null;
        // Match stmt with different cases
        if ((stmt instanceof CAst.AssignmentNode
            && ((CAst.AssignmentNode) stmt).left instanceof CAst.StateVarNode)) {
          CAst.StateVarNode n = (CAst.StateVarNode) ((CAst.AssignmentNode) stmt).left;
          instance =
              reaction.getReaction().getParent().states.stream()
                  .filter(s -> s.getName().equals(n.name))
                  .findFirst()
                  .get();
          unusedStates.remove(instance);
        } else if (stmt instanceof CAst.SetPortNode) {
          CAst.SetPortNode n = (CAst.SetPortNode) stmt;
          String name = ((CAst.VariableNode) n.left).name;
          instance =
              reaction.getReaction().getParent().outputs.stream()
                  .filter(s -> s.getName().equals(name))
                  .findFirst()
                  .get();
          unusedOutputs.remove(instance);
        } else if (stmt instanceof CAst.ScheduleActionNode) {
          CAst.ScheduleActionNode n = (CAst.ScheduleActionNode) stmt;
          String name = ((CAst.VariableNode) n.children.get(0)).name;
          instance =
              reaction.getReaction().getParent().actions.stream()
                  .filter(s -> s.getName().equals(name))
                  .findFirst()
                  .get();
          unusedActions.remove(instance);
        } else if (stmt instanceof CAst.ScheduleActionIntNode) {
          CAst.ScheduleActionIntNode n = (CAst.ScheduleActionIntNode) stmt;
          String name = ((CAst.VariableNode) n.children.get(0)).name;
          instance =
              reaction.getReaction().getParent().actions.stream()
                  .filter(s -> s.getName().equals(name))
                  .findFirst()
                  .get();
          unusedStates.remove(instance);
          unusedActions.remove(instance);
        } else continue;
        // Create a new entry in the list if there isn't one yet.
        if (defaultBehaviorConditions.get(instance) == null) {
          defaultBehaviorConditions.put(instance, new ArrayList<CAst.AstNode>());
        }
        defaultBehaviorConditions.get(instance).add(ifBlockNode.left);
        // System.out.println("DEBUG: Added a reset condition: " + ifBlockNode.left);
      }

      // Generate Uclid axiom for the C AST.
      CToUclidVisitor c2uVisitor = new CToUclidVisitor(this, reaction);
      String axiom = c2uVisitor.visit(inf);
      code.pr(
          String.join(
              "\n",
              "// Reaction body of " + reaction,
              "axiom(finite_forall (i : integer) in indices :: (i > START && i <= END) ==> (",
              "    (" + reaction.getReaction().getFullNameWithJoiner("_") + "(rxn(i))" + ")",
              "        ==> " + "(" + "(" + axiom + ")",
              "&& " + "( " + "true"));
      for (NamedInstance key : defaultBehaviorConditions.keySet()) {
        CAst.AstNode disjunction = AstUtils.takeDisjunction(defaultBehaviorConditions.get(key));
        CAst.LogicalNotNode notNode = new CAst.LogicalNotNode();
        notNode.child = disjunction;
        String resetCondition = c2uVisitor.visit(notNode);

        // Check for invalid reset conditions.
        // If found, stop the execution.
        // FIXME: A more systematic check is needed
        // to ensure that the generated Uclid file
        // is valid.
        if (resetCondition.contains("null")) {
          throw new IllegalStateException("Null detected in a reset condition. Stop.");
        }

        code.pr("// Unused state variables and ports are reset by default.");
        code.pr("&& " + "(" + "(" + resetCondition + ")" + " ==> " + "(");
        if (key instanceof StateVariableInstance) {
          StateVariableInstance n = (StateVariableInstance) key;
          code.pr(
              n.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + n.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + "-1"
                  + ")"
                  + ")");
        } else if (key instanceof PortInstance) {
          PortInstance n = (PortInstance) key;
          code.pr(
              "("
                  + " true"
                  // Reset value
                  + "\n&& "
                  + n.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + "0" // Default value
                  // Reset presence
                  + "\n&& "
                  + n.getFullNameWithJoiner("_")
                  + "_is_present"
                  + "("
                  + "t"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + "false" // default presence
                  + ")");
        } else if (key instanceof ActionInstance) {
          ActionInstance n = (ActionInstance) key;
          code.pr(
              n.getFullNameWithJoiner("_")
                  + "_scheduled"
                  + "("
                  + "d"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + "false");
        } else {
          throw new AssertionError("unreachable");
        }
        code.pr("))");
      }

      // For state variables and ports that are NOT used in this reaction,
      // their values stay the same by default.
      code.pr("// By default, the value of the variables used in this reaction stay the same.");
      if (this.logicalTimeBased) {
        // If all other reactions that can modify the SAME state variable
        // are not triggered, then the state variable stay the same.
        //
        // FIXME: What if two reactions modifying the same state variable
        // are triggered at the same time?
        // How to use axioms to model reaction priority?
        // The main difficulty of logical time based semantics is composing
        // the effect of simultaneous reactions.
        //
        // A path way to implement it in the future:
        // 1. For each variable, port, and action, determine a list of
        //    reactions that can modify/schedule it.
        // 2. Reaction axioms should be generated wrt each reactor.
        //    For example, assuming a reactor with two input ports,
        //    each triggering a distinct reaction. The axioms will need
        //    to handle four cases: i. reaction 1 gets triggered and 2
        //    does not; ii. reaction 2 gets triggered and 1 does not;
        //    iii. both reactions get triggered; iv. none of them get
        //    triggered. Since it is hard to specify in an independent way,
        //    due to reaction priorities,
        //    what happens when two reactions (modifying the same state var.)
        //    get triggered simultaneously, some combinatorial blowup will
        //    be incurred. In this example, four axioms (instead of two),
        //    each handling one case, seems needed. The good news is that
        //    axioms across reactors may be specified independently.
        //    For example, if there is another reactor of the same class,
        //    Only four more axioms need to be added (in total 2^2 + 2^2),
        //    instead of 16 axioms (2^4).
      } else {
        for (StateVariableInstance s : unusedStates) {
          code.pr("&& (true ==> (");
          code.pr(
              s.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + s.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + "-1"
                  + ")"
                  + ")");
          code.pr("))");
        }
        for (PortInstance p : unusedOutputs) {
          code.pr("&& (true ==> (");
          code.pr(
              "("
                  + " true"
                  // Reset value
                  + "\n&& "
                  + p.getFullNameWithJoiner("_")
                  + "("
                  + "s"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + "0" // Default value
                  // Reset presence
                  + "\n&& "
                  + p.getFullNameWithJoiner("_")
                  + "_is_present"
                  + "("
                  + "t"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + false // default presence
                  + ")");
          code.pr("))");
        }
        for (ActionInstance a : unusedActions) {
          code.pr("&& (true ==> (");
          code.pr(
              a.getFullNameWithJoiner("_")
                  + "_scheduled"
                  + "("
                  + "d"
                  + "("
                  + "i"
                  + ")"
                  + ")"
                  + " == "
                  + "false");
          code.pr("))");
        }
        code.pr("))));");
      }
    }
  }

  protected void generateProperty() {
    code.pr(String.join("\n", "/************", " * Property *", " ************/"));

    code.pr("// The FOL property translated from user-defined MTL property:");
    code.pr("// " + this.spec);
    code.pr("define p(i : step_t) : boolean =");
    code.indent();
    code.pr(this.FOLSpec + ";");
    code.unindent();

    if (this.tactic == Tactic.BMC) {
      code.pr(
          String.join(
              "\n",
              "// BMC",
              "property " + "bmc_" + this.name + " : " + "initial_condition() ==> p(0);"));
    } else {
      code.pr(
          String.join(
              "\n",
              "// Induction: initiation step",
              "property " + "initiation_" + this.name + " : " + "initial_condition() ==> p(0);",
              "// Induction: consecution step",
              "property " + "consecution_" + this.name + " : " + "p(0) ==> p(1);"));
    }
  }

  /** Uclid5 control block */
  protected void generateControlBlock() {
    code.pr(
        String.join(
            "\n",
            "control {",
            "    v = bmc(0);",
            "    check;",
            "    print_results;",
            "    v.print_cex_json;",
            "}"));
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

  /** Populate the data structures. */
  private void populateDataStructures() {
    // Populate lists of reactor/reaction instances,
    // state variables, actions, ports, and timers.
    populateLists(this.main);

    // Join actions, ports, and timers into a list of triggers.
    this.triggerInstances = new ArrayList<TriggerInstance>(this.actionInstances);
    this.triggerInstances.addAll(portInstances);
    this.triggerInstances.addAll(timerInstances);

    // Join state variables and triggers
    this.namedInstances = new ArrayList<NamedInstance>(this.stateVariables);
    namedInstances.addAll(this.triggerInstances);
  }

  private void populateLists(ReactorInstance reactor) {
    // Reactor and reaction instances
    this.reactorInstances.add(reactor);
    for (var reaction : reactor.reactions) {
      this.reactionInstances.addAll(reaction.getRuntimeInstances());
    }

    // State variables, actions, ports, timers.
    for (var state : reactor.states) {
      this.stateVariables.add(state);
    }
    for (var action : reactor.actions) {
      this.actionInstances.add(action);
    }
    for (var port : reactor.inputs) {
      this.inputInstances.add(port);
      this.portInstances.add(port);
    }
    for (var port : reactor.outputs) {
      this.outputInstances.add(port);
      this.portInstances.add(port);
    }
    for (var timer : reactor.timers) {
      this.timerInstances.add(timer);
    }

    // Recursion
    for (var child : reactor.children) {
      populateLists(child);
    }
  }

  /**
   * Compute a completeness threadhold for each property by simulating a worst-case execution by
   * traversing the reactor instance graph and building a state space diagram.
   */
  private void computeCT() {

    StateSpaceExplorer explorer = new StateSpaceExplorer(this.main);
    explorer.explore(
        new Tag(this.horizon, 0, false), true // findLoop
        );
    StateSpaceDiagram diagram = explorer.diagram;
    diagram.display();

    // Generate a dot file.
    try {
      CodeBuilder dot = diagram.generateDot();
      Path file = this.outputDir.resolve(this.tactic + "_" + this.name + ".dot");
      String filename = file.toString();
      dot.writeToFile(filename);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    //// Compute CT
    if (!explorer.loopFound) {
      if (this.logicalTimeBased) this.CT = diagram.nodeCount();
      else {
        // FIXME: This could be much more efficient with
        // a linkedlist implementation. We can go straight
        // to the next node.
        StateSpaceNode node = diagram.head;
        this.CT = diagram.head.getReactionsInvoked().size();
        while (node != diagram.tail) {
          node = diagram.getDownstreamNode(node);
          this.CT += node.getReactionsInvoked().size();
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
        loopIterations = (int) Math.ceil((double) horizonRemained / diagram.loopPeriod);
      }

      if (this.logicalTimeBased) {
        /*
        CT = steps required for the non-periodic part
             + steps required for the periodic part

        this.CT = (diagram.loopNode.index + 1)
            + (diagram.tail.index - diagram.loopNode.index + 1) * loopIterations;

        An overflow-safe version of the line above
        */
        int t0 = Math.addExact(diagram.loopNode.getIndex(), 1);
        int t1 = Math.subtractExact(diagram.tail.getIndex(), diagram.loopNode.getIndex());
        int t2 = Math.addExact(t1, 1);
        int t3 = Math.multiplyExact(t2, loopIterations);
        this.CT = Math.addExact(t0, t3);

      } else {
        // Get the number of events before the loop starts.
        // This stops right before the loopNode is encountered.
        StateSpaceNode node = diagram.head;
        int numReactionInvocationsBeforeLoop = 0;
        while (node != diagram.loopNode) {
          numReactionInvocationsBeforeLoop += node.getReactionsInvoked().size();
          node = diagram.getDownstreamNode(node);
        }
        // Account for the loop node in numReactionInvocationsBeforeLoop.
        numReactionInvocationsBeforeLoop += node.getReactionsInvoked().size();

        // Count the events from the loop node until
        // loop node is reached again.
        int numReactionInvocationsInsideLoop = 0;
        do {
          node = diagram.getDownstreamNode(node);
          numReactionInvocationsInsideLoop += node.getReactionsInvoked().size();
        } while (node != diagram.loopNode);

        /*
        CT = steps required for the non-periodic part
             + steps required for the periodic part

        this.CT = numReactionInvocationsBeforeLoop
            + numReactionInvocationsInsideLoop * loopIterations;

        An overflow-safe version of the line above
        */
        // System.out.println("DEBUG: numReactionInvocationsBeforeLoop: " +
        // numReactionInvocationsBeforeLoop);
        // System.out.println("DEBUG: numReactionInvocationsInsideLoop: " +
        // numReactionInvocationsInsideLoop);
        // System.out.println("DEBUG: loopIterations: " + loopIterations);
        int t0 = Math.multiplyExact(numReactionInvocationsInsideLoop, loopIterations);
        this.CT = Math.addExact(numReactionInvocationsBeforeLoop, t0);
      }
    }
  }

  /** Process an MTL property. */
  private void processMTLSpec() {
    MTLLexer lexer = new MTLLexer(CharStreams.fromString(this.spec));
    CommonTokenStream tokens = new CommonTokenStream(lexer);
    MTLParser parser = new MTLParser(tokens);
    MtlContext mtlCtx = parser.mtl();
    MTLVisitor visitor = new MTLVisitor(this.tactic);

    // The visitor transpiles the MTL into a Uclid axiom.
    this.FOLSpec = visitor.visitMtl(mtlCtx, "i", 0, "0", 0);
    this.horizon = visitor.getHorizon();
  }

  /////////////////////////////////////////////////
  //// Functions from generatorBase

  @Override
  public Target getTarget() {
    return Target.C; // Works with a C subset.
  }

  @Override
  public TargetTypes getTargetTypes() {
    throw new UnsupportedOperationException(
        "This method is not applicable for this generator since Uclid5 is not an LF target.");
  }
}
