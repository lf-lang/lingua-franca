package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.analyses.statespace.Tag;
import org.lflang.analyses.uclid.ReactionData.UclCall;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.Action;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
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

import jakarta.enterprise.inject.Typed;

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

    /** A list of reactors in the LF program */
    public List<Reactor> reactors = new ArrayList<>();

    /** A list of reactions in the LF program */
    public List<Reaction> reactions = new ArrayList<>();

    public HashMap<String, ReactionData> reactionDataMap;
    public HashMap<ReactorInstance, Integer> reactorInst2Cnt = new HashMap<>(); // FIXME: Can be local
    public HashMap<ReactionInstance, Integer> reactionInst2Cnt = new HashMap<>();
    public HashMap<ReactorInstance, Integer> reactorInst2Index = new HashMap<>();

    /** State space diagram for the LF program */
    StateSpaceDiagram diagram;

    public UclidFSMGenerator(LFGeneratorContext context, HashMap<String, ReactionData> reactionDataMap) {
        this.context = context;
        this.modGenDir = context.getFileConfig().getModelGenPath();
        this.targetConfig = context.getTargetConfig();
        this.reactors = ASTUtils.getAllReactors(targetConfig.getMainResource());
        this.reactions = this.reactors.stream().map(it -> it.getReactions()).flatMap(List::stream).toList();
        this.reactionDataMap = reactionDataMap;
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
        generateUclidModule();
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

        /** For each reactor */
        for (Reactor reactorDef : this.reactors) {
            /** Generate a type for each port. */
            List<? extends TypedVariable> all = Stream.of(reactorDef.getInputs(), reactorDef.getOutputs(), reactorDef.getActions()).flatMap(List::stream).toList();
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
                if (tv instanceof Port) {
                    String dtype = tv.getType().getId();
                    String uclid_type = getUclidTypeFromCType(dtype, true);
                    code.pr("(\"value\", " + uclid_type + "),");
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
            code.unindent();
            code.pr("]");
            code.unindent();
            code.pr(")");
            /** Create a self variables for function input and output arguments. */
            String self_input = self_name + "_input";
            code.pr(self_input + " = UclidLiteral(\"" + self_input + "\")");
            String self_output = self_name + "_output";
            code.pr(self_output + " = UclidLiteral(\"" + self_output + "\")");

            /** Generate reactor type that includes ports and self type (state variables and parameters). */
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

        for (Reactor reactorDef : this.reactors) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (int i = 0; i < reactionDefs.size(); i++) {
                String reaction_name = reactorDef.getName() + "_reaction_" + i;
                Reaction reactionDef = reactionDefs.get(i);
                List<? extends TypedVariable> all = Stream.of(getAllInputs(reactionDef), getAllOutputs(reactionDef)).flatMap(List::stream).toList();
                ReactionData reactionData = this.reactionDataMap.get(reaction_name);
                for (int j = 0; j < all.size(); j++) {
                    TypedVariable tv = all.get(j);
                    String type = reactorDef.getName() + "_" + tv.getName() + "_t";
                    reactionData.types.get(type).get("is_present").setUclType("boolean");
                    if (tv instanceof Port) {
                        String uclid_type = getUclidTypeFromCType(tv.getType().getId(), false);
                        reactionData.types.get(type).get("value").setUclType(uclid_type);
                    }
                }
                String reactorSelfType = reactorDef.getName() + "_self_t";
                reactionData.types.get(reactorSelfType).forEach((key, value) -> {
                    String uclid_type = getUclidTypeFromCType(value.getTgtType(), false);
                    value.setUclType(uclid_type);
                });
            }
        }

        /** Generate noinline procedure for each reaction. */
        for (Reactor reactorDef : this.reactors) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (int i = 0; i < reactionDefs.size(); i++) {
                String reaction_name = reactorDef.getName() + "_reaction_" + i;
                String requires = reaction_name + "_requires";
                String ensures = reaction_name + "_ensures";
                String sig = reaction_name + "_sig";
                String proc = reaction_name + "_proc";
                List<? extends TypedVariable> inputs = getAllInputs(reactionDefs.get(i));
                List<? extends TypedVariable> outputs = getAllOutputs(reactionDefs.get(i));
                ReactionData reactionData = this.reactionDataMap.get(reaction_name);
                /** Creates requires expression */
                code.pr(requires + " = UclidRaw(");
                code.indent();
                code.pr("self.ext_procs[\"" + reaction_name + "\"].getLatestUclidRequiresString()");
                code.unindent();
                code.pr(")");
                /** Creates ensures expression */
                code.pr(ensures + " = UclidRaw(");
                code.indent();
                code.pr("self.ext_procs[\"" + reaction_name + "\"].getLatestUclidEnsuresString()");
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
                String reactorSelfInput = reactorDef.getName() + "_self_input";
                String reactorSelfType = reactorDef.getName() + "_self_t";
                code.pr("(" + reactorSelfInput + ", " + reactorSelfType + "),");
                reactionData.inputs.get(reactionData.inputs.size() - 1).setUclName(reactorSelfInput);
                reactionData.inputs.get(reactionData.inputs.size() - 1).setUclType(reactorSelfType);
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
                String reactorSelfOutput = reactorDef.getName() + "_self_output";
                code.pr("(" + reactorSelfOutput + ", " + reactorSelfType + "),");
                reactionData.outputs.get(reactionData.outputs.size() - 1).setUclName(reactorSelfOutput);
                reactionData.outputs.get(reactionData.outputs.size() - 1).setUclType(reactorSelfType);
                code.unindent();
                code.pr("],");
                code.pr("requires=" + requires + ",");
                code.pr("ensures=" + ensures + ",");
                code.pr("noinline=True,");
                code.unindent();
                code.pr(")");

                code.pr(proc + " = m.mkProcedure(");
                code.indent();
                code.pr("\"" + reaction_name + "\",");
                code.pr(sig + ",");
                code.pr("UclidBlockStmt([]),");
                code.unindent();
                code.pr(")");

            }
        }

        /** Build procedure for each state in the state diagram. */
        StateSpaceNode node = diagram.head;
        while (true) {
            List<ReactionInstance> reactionInsts = new ArrayList<>(node.getReactionsInvoked());
            // Increment the counter for the reactor instance
            for (ReactionInstance reactionInst : reactionInsts) {
                ReactorInstance reactorInst = reactionInst.getParent();
                this.reactorInst2Cnt.put(reactorInst, this.reactorInst2Cnt.getOrDefault(reactorInst, 0) + 1);
                this.reactionInst2Cnt.put(reactionInst, this.reactionInst2Cnt.getOrDefault(reactionInst, 0) + 1);
            }
            
            if (node == diagram.tail) {
                break;
            } else {
                node = diagram.getDownstreamNode(node);
            }
        }

        for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
            System.out.println("Reactor: " + entry.getKey().getName() + " / " + entry.getKey().reactorDefinition.getName());
            ReactorInstance reactorInst = entry.getKey();
            String reactorInstName = reactorInst.getName();
            String reactorType = getReactorType(reactorInst.reactorDefinition);
            String reactorInstArrayName = getReactorInstArrayName(reactorInst);
            // for (int i = 0; i <= 2 * entry.getValue(); i++) {
            //     String reactorInstCopy = getReactorInstCopy(entry.getKey(), i);
            //     code.pr(reactorInstCopy + " = m.mkVar(\"" + reactorInstCopy + "\", " + reactorType + ")");
            // }
            code.pr(reactorInstArrayName + " = [m.mkVar(\"" + reactorInstName + "_\" + str(i), " + reactorType + ") for i in range(" + (2 * entry.getValue() + 1) + ")]");
            /** Create a buffer-versioned variable for each reactor instance in case there are delayed connectons or actions */
            code.pr(getReactorInstDelayBuffer(reactorInst) + " = m.mkVar(\"" + getReactorInstDelayBuffer(reactorInst) + "\", " + reactorType + ")");
        }
        List<String> reactionFiredNames = new ArrayList<>();
        for (HashMap.Entry<ReactionInstance, Integer> entry : this.reactionInst2Cnt.entrySet()) {
            System.out.println("Reaction: " + entry.getKey().getFullName());
            // String reactorInstName = entry.getKey().getParent().getName();
            // String reactionName = entry.getKey().getName();
            String reaction_fired = getReactionFiredName(entry.getKey());
            // String reactionInstArrayName = reactionInstName + "_array";
            code.pr(reaction_fired + " = m.mkVar(\"" + reaction_fired + "\", UBool)");
            reactionFiredNames.add(reaction_fired);
        }
        /** Create array of strings for "fired" variables */
        code.pr("fired = [" + String.join(", ", reactionFiredNames) + "]");
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


        node = diagram.head;
        while (true) {
            System.out.println("Generating state procedure for node " + node.getIndex());
            generateStateProcedure(node);

            if (node == diagram.tail) {
                break;
            } else {
                node = diagram.getDownstreamNode(node);
            }
        }

        // cycle_start is the index of the loop node if the loop node is not null otherwise is the number of states
        int cycle_start = diagram.loopNode == null? diagram.tail.getIndex() + 1 : diagram.loopNode.getIndex();
        /** Describe state transition in procedure. */
        code.pr("state = m.mkVar(\"state\", UInt)");
        code.pr("num_states = m.mkConst(\"num_states\", UInt, UclidIntegerLiteral(" + (diagram.tail.getIndex() + 1) + "))");
        code.pr("cycle_start = m.mkConst(\"cycle_start\", UInt, UclidIntegerLiteral(" + cycle_start + "))");
        code.pr("state_machine_sig = UclidProcedureSig(");
        code.indent();
        code.pr("inputs=[],");
        code.pr("modifies=[state] + fired + " + this.reactorInst2Cnt.keySet().stream().map(it -> it.getName() + "_array").reduce((a, b) -> a + " + " + b).get() + ",");
        code.pr("returns=[],");
        code.pr("noinline=False,");
        code.unindent();
        code.pr(")");

        code.pr("state_machine_proc = m.mkProcedure(");
        code.indent(); // Procedure
        code.pr("\"state_machine\",");
        code.pr("state_machine_sig,");
        code.pr("UclidBlockStmt([");
        code.indent(); // Block statement
        /** Reset variables indicating whether procedures have fired */
        code.pr("UclidProcedureCallStmt(reset_fire_proc, [], []),");
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
        for (int i = 1; i <= diagram.tail.getIndex(); ++i) {
            code.pr("UclidProcedureCallStmt(state_" + i + "_proc, [], []),");
        }
        code.pr("UclidBlockStmt([]),");
        code.unindent(); // Actions
        code.pr("],");
        code.unindent(); // Case statement
        code.pr("),");
        code.pr("UclidITEStmt(");
        code.indent(); // ITE statement
        code.pr("Ugte([state, Usub([num_states, UclidIntegerLiteral(1)])]),");
        code.pr("UclidAssignStmt(state, cycle_start),");
        code.pr("UclidAssignStmt(state, Uadd([state, UclidIntegerLiteral(1)])),");
        code.unindent(); // ITE statement
        code.pr("),");
        code.unindent(); // Block statement
        code.pr("])");
        code.unindent(); // Procedure
        code.pr(")");

        /** Perform state transition in next block. */
        code.pr("m.setNext(UclidProcedureCallStmt(state_machine_proc, [], []))");

        /** Uclid init block */
        code.pr("m.setInit(UclidInitBlock([");
        code.indent();
        for (HashMap.Entry<ReactorInstance, Integer> entry : this.reactorInst2Cnt.entrySet()) {
            String reactorInstOrig = getReactorInstArrayCopy(entry.getKey(), 0);
            code.pr("UclidHavocStmt(" + reactorInstOrig + "),");
            code.pr("*[UclidAssignStmt(v, " + reactorInstOrig + ") for v in " + getReactorInstArrayName(entry.getKey()) + "[1:]],");
            // for (int i = 1; i <= 2 * entry.getValue(); i++) {
            //     String reactorInstCopy = getReactorInstCopy(entry.getKey(), i);
            //     code.pr("UclidAssignStmt(" + reactorInstCopy + ", " + reactorInstOrig + "),");
            // }
        }
        code.pr("UclidProcedureCallStmt(reset_fire_proc, [], []),");
        code.pr("UclidProcedureCallStmt(state_0_proc, [], []),");
        code.pr("UclidAssignStmt(state, UclidIntegerLiteral(1)),");
        code.unindent();
        code.pr("]))");

        /** Property */
        code.pr("m.mkProperty(");
        code.indent();
        code.pr("\"PROPERTY\",");
        code.pr("UBoolTrue,");
        code.unindent();
        code.pr(")");

        /** Control block */
        int steps = 0;
        code.pr("m.setControl(UclidControlBlock([");
        code.indent(); // Control block
        code.pr("UclidBMCCommand(\"v\", " + steps + "),");
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
            List<Port> ports = new ArrayList<>();
            ports.addAll(entry.getKey().reactorDefinition.getInputs());
            ports.addAll(entry.getKey().reactorDefinition.getOutputs());
            code.pr("[");
            code.indent();
            code.pr("UclidRecordSelect(UclidRecordSelect(v, p), attr)");
            code.pr("for attr in [\"is_present\", \"value\"]");
            code.pr("for p in [" + String.join(", ", ports.stream().map(it -> "\"" + it.getName() + "\"").toList()) + "]");
            code.pr("for v in " + getReactorInstArrayName(entry.getKey()));
            code.unindent();
            code.pr("],");
            code.pr("[");
            code.indent();
            code.pr("UclidRecordSelect(UclidRecordSelect(v, \"self\"), attr)");
            List<String> self_attrs = new ArrayList<>();
            entry.getKey().reactorDefinition.getParameters().stream().map(it -> it.getName()).forEach(self_attrs::add);
            entry.getKey().reactorDefinition.getStateVars().stream().map(it -> it.getName()).forEach(self_attrs::add);
            code.pr("for attr in [" + String.join(", ", self_attrs.stream().map(it -> "\"" + it + "\"").toList()) + "]");
            code.pr("for v in " + getReactorInstArrayName(entry.getKey()));
            code.unindent();
            code.pr("],");
        }
        code.unindent(); //
        code.pr("], []),");
        code.unindent(); // sum
        code.pr(")");
        code.unindent(); // Control block
        code.pr("]))");
        /** Return statement */
        code.pr("return m");

        code.unindent();
    }


    private String getReactorInstCopy(ReactorInstance reactorInst, int i) {
        return reactorInst.getName() + "_" + i;
    }

    private String getReactorType(Reactor reactor) {
        return reactor.getName() + "_t";
    }

    private String getReactorInstArrayName(ReactorInstance reactorInst) {
        return reactorInst.getName() + "_array";
    }

    private String getReactorInstArrayCopy(ReactorInstance reactorInst, int i) {
        return getReactorInstArrayName(reactorInst) + "[" + i + "]";
    }

    /**
     * Get the next index for the reactor instance.
     * @param reactorInst The reactor instance.
     * @return The next index for the reactor instance (starts from 1).
     */
    private int getNextReactorInstIndex(ReactorInstance reactorInst) {
        int index = this.reactorInst2Index.getOrDefault(reactorInst, 1);
        this.reactorInst2Index.put(reactorInst, index + 1);
        return index;
    }

    private void generateStateProcedure(StateSpaceNode node) {
        /** 
         * Sort invoked reactions by the smallest number in the set 
         * returned by reactionInstance.getLevels() 
         */
        List<ReactionInstance> reactionInsts = new ArrayList<>(node.getReactionsInvoked());
        reactionInsts.sort((r1, r2) -> r1.getLevels().stream().min(Integer::compare).get() - r2.getLevels().stream().min(Integer::compare).get());

        String state_name = "state_" + node.getIndex();
        String sig = state_name + "_sig";
        String proc = state_name + "_proc";
        /** Generate function signature */
        code.pr(sig + " = UclidProcedureSig(");
        code.indent();
        code.pr("inputs=[],");
        code.pr("modifies=fired + " + this.reactorInst2Cnt.keySet().stream().map(it -> it.getName() + "_array").reduce((a, b) -> a + " + " + b).get() + ",");
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
        /** Procedure body 
         *  Assumes that each reaction only modifies the state of the reactor which the reaction belongs to 
         *  i.e. reaction only reads from input ports and writes to output ports of the reactor 
         *  (and also modifies the state of the reactor). Thus we only need to store the state before and after the reaction is invoked.
         *  First assign to ports or actions values that are delayed due to the `after` keyword or min-delay.
         *  Then invoke each reaction in the state based on the order of the levels.
         *  For each invocation, we follow the following steps:
         * 1. Store the state of the reactor before invoking the reaction.
         * 2. Check if the input triggers are present.
         * 3. If the input triggers are present, call the external procedure and record that the reaction is fired.
         * 4. Assign the value to downstream ports.
         * 5. Store the state of the reactor after invoking the reaction.
         */

        /** 1. Store the state of the reactor before invoking the reaction. */
        Set<TriggerInstance<? extends Variable>> updates = node.getUpdateInstances();
        for (TriggerInstance<? extends Variable> inst : updates) {
            ReactorInstance reactorInst = inst.getParent();
            String name = inst.getName();
            code.pr("UclidAssignStmt(" + UclidSelect(getReactorInstArrayCopy(reactorInst, 0), name) + ", " + UclidSelect(getReactorInstDelayBuffer(reactorInst), name) + "),");
        }

        for (ReactionInstance reactionInst : reactionInsts) {
            ReactorInstance reactorInst = reactionInst.getParent();
            System.out.println("Reactor: " + reactorInst.getName() + " / " + reactorInst.reactorDefinition.getName());
            Reaction reaction = reactionInst.getDefinition();
            List<? extends TypedVariable> triggers = getAllInputs(reaction);
            List<? extends TypedVariable> effects = getAllOutputs(reaction);
            String reactionName = getReactionName(reactorInst.reactorDefinition, reactionInst.index);
            ReactionData reactionData = this.reactionDataMap.get(reactionName);
            UclCall uclCall = reactionData.new UclCall();
            /** Store reactor state before invoking reaction. */
            String reactorInstOrigName = getReactorInstArrayCopy(reactorInst, 0);
            int preStateIndex = getNextReactorInstIndex(reactorInst);
            code.pr("# Store reactor pre-state");
            code.pr("UclidAssignStmt(" + getReactorInstArrayCopy(reactorInst, preStateIndex) + ", " + reactorInstOrigName + "),"); 
            /** Check if input triggers are present. */
            if (triggers.size() > 0) {
                code.pr("# Check if input triggers are present");
                code.pr("UclidITEStmt(");
                code.indent(); // ITE statement
                code.pr("Uor([");
                code.indent(); // Conditions
                for (TypedVariable tv : triggers) {
                    String present = UclidSelect(UclidSelect(reactorInstOrigName, tv.getName()), "is_present");
                    code.pr(present + ",");
                }
                code.unindent(); // Conditions
                code.pr("]),");
                code.pr("UclidBlockStmt([");
                code.indent(); // Block statement
            }
            /** Assign true to variable that indicates whether a reaction has fired */
            String reaction_fired = getReactionFiredName(reactionInst);
            code.pr("UclidAssignStmt(" + reaction_fired + ", UBoolTrue),");
            uclCall.flag = reaction_fired;
            /** Call external procedure */
            code.pr("# Call external procedure");
            code.pr("UclidProcedureCallStmt(");
            code.indent(); // Procedure call
            code.pr(reactorInst.reactorDefinition.getName() + "_reaction_" + reactionInst.index + "_proc,");
            /** Input triggers */
            code.pr("[");
            code.indent(); // input triggers
            for (TypedVariable tv : triggers) {
                String name = UclidSelect(reactorInstOrigName, tv.getName());
                code.pr(name + ",");
                uclCall.inputs.add(getReactorInstCopy(reactorInst, preStateIndex) + "." + tv.getName());
            }
            code.pr(UclidSelect(reactorInstOrigName, "self") + ",");
            uclCall.inputs.add(reactorInst.getName() + "_" + preStateIndex + "." + "self");
            code.unindent(); // input triggers
            code.pr("],");
            /** Output effects */
            int postStateIndex = getNextReactorInstIndex(reactorInst);
            code.pr("[");
            code.indent(); // output effects
            for (TypedVariable tv : effects) {
                String name = UclidSelect(reactorInstOrigName, tv.getName());
                /** If the effect is an action, first assign to the buffer variable.
                 * The value will be assigned to the actual variable at the correct time afterwards.
                 */
                if (tv instanceof Action) name = UclidSelect(getReactorInstDelayBuffer(reactorInst), tv.getName());
                code.pr(name + ",");
                uclCall.outputs.add(getReactorInstCopy(reactorInst, postStateIndex) + "." + tv.getName());
            }
            code.pr(UclidSelect(reactorInstOrigName, "self") + ",");
            uclCall.outputs.add(reactorInst.getName() + "_" + postStateIndex + "." + "self");
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
                        String sourceReactorInstName = getReactorInstArrayCopy(sourceReactorInst, 0);
                        Connection connection = range.connection;
                        List<RuntimeRange<PortInstance>> destinations = range.destinations;
                        System.out.println("Source: " + source.getFullName());
                        for (RuntimeRange<PortInstance> d : destinations) {
                            System.out.println("Destination: " + d.instance.getFullName());
                            PortInstance dest = d.instance;
                            ReactorInstance destReactorInst = dest.getParent();
                            String destReactorInstName = getReactorInstArrayCopy(destReactorInst, 0);
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
                                    code.pr("UclidAssignStmt(" + UclidSelect(getReactorInstDelayBuffer(destReactorInst), dest.getName()) + ", " + UclidSelect(sourceReactorInstName, source.getName()) + "),");
                                } else {
                                    throw new RuntimeException("Unsupported delay expression: " + delayExpr);
                                }
                            } else {
                                code.pr("UclidAssignStmt(" + UclidSelect(destReactorInstName, dest.getName()) + ", " + UclidSelect(sourceReactorInstName, source.getName()) + "),");
                            }
                        }
                    }
                }
                else if (tv instanceof Action) {
                    Action action = (Action) tv;
                    ActionInstance actionInst = reactorInst.lookupActionInstance(action);
                    TimeValue min_delay = actionInst.getMinDelay();
                    if (min_delay == TimeValue.ZERO) {
                        code.pr("UclidAssignStmt(" + UclidSelect(getReactorInstArrayCopy(reactorInst, 0), action.getName()) + ", " + UclidSelect(getReactorInstDelayBuffer(reactorInst), action.getName()) + "),");
                    } // Shouldn't need to do anything if min_delay is not zero, since the value is already stored in the buffer
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
            code.pr("UclidAssignStmt(" + getReactorInstArrayCopy(reactorInst, postStateIndex) + ", " + reactorInstOrigName + "),"); 
            reactionData.uclCalls.add(uclCall);
        }
        code.unindent(); // Block statement
        code.pr("])");
        code.unindent(); // Procedure
        code.pr(")");
    }

    private String getReactionName(Reactor reactor, int index) {
        return reactor.getName() + "_reaction_" + index;
    }

    private String getReactorInstDelayBuffer(ReactorInstance reactorInst) {
        return reactorInst.getName() + "_delay_buffer";
    }

    /**
     * Get the name of the variable that indicates whether a reaction has fired.
     * @param reactionInst Reaction instance that is fired
     * @return The name of the variable that indicates whether a reaction has fired.
     */
    private String getReactionFiredName(ReactionInstance reactionInst) {
        return reactionInst.getParent().getName() + "_reaction_" + reactionInst.index + "_fired";
    }

    /** Match the C type to the Uclid type */
    private String getUclidTypeFromCType(String type, Boolean api) {
        return switch (type) {
            case "bool" -> api? "UBool" : "boolean";
            case "int", "int32_t", "unsigned", "unsigned int", "uint32_t" -> api? "UclidBVType(32)" : "bv32";
            case "int64_t", "uint64_t" -> api? "UclidBVType(64)" : "bv64";
            case "float" -> api? "UclidFloatType()" : "single";
            default -> throw new RuntimeException("Unsupported type: " + type);
        };
    }

    /** Helper function for record select */
    private String UclidSelect(String record, String field) {
        return "UclidRecordSelect(" + record + ", \"" + field + "\")";
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
                .filter(it -> (it instanceof TypedVariable tv && (tv instanceof Port p || tv instanceof Action a)))
                .map(it -> (TypedVariable) it).toList();
    }

    // FIXME: Add this to ASTUtils.java in a principled way.
    // Can typed variables be other things than ports and actions?
    private List<? extends TypedVariable> getAllOutputs(Reaction reaction) {
        return reaction.getEffects().stream()
                .map(it -> it.getVariable())
                .filter(it -> (it instanceof TypedVariable tv && (tv instanceof Port p || tv instanceof Action a)))
                .map(it -> (TypedVariable) it).toList();
    }
    
}
