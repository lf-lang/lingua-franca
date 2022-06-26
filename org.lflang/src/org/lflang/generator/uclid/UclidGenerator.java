/*************
Copyright (c) 2021, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/** 
 * Generator for Uclid models.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.generator.uclid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.xbase.lib.Exceptions;

import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactionInstanceGraph;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TargetTypes;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

import static org.lflang.ASTUtils.*;

public class UclidGenerator extends GeneratorBase {

    ////////////////////////////////////////////
    //// Private variables

    // Data structures for storing info about the runtime instances.
    Set<ReactionInstance.Runtime> reactionInstances;

    // The reaction graph upon which the causality graph is built
    ReactionInstanceGraph reactionInstanceGraph;

    // String lists storing variable names of different types
    List<String> variableNames      = new LinkedList<String>();
    List<String> triggerNames       = new LinkedList<String>();
    List<String> triggerPresence    = new LinkedList<String>();

    // Data structures for storing properties
    List<String> properties         = new ArrayList<String>();

    ////////////////////////////////////////////
    //// Protected fields

    /** The main place to put generated code. */
    protected CodeBuilder code = new CodeBuilder();

    // Constructor
    public UclidGenerator(FileConfig fileConfig, ErrorReporter errorReporter) {
        super(fileConfig, errorReporter);
    }

    ////////////////////////////////////////////////////////////
    //// Public methods
    public void doGenerate(Resource resource, LFGeneratorContext context) {
        
        // The following generates code needed by all the reactors.
        super.doGenerate(resource, context);

        // Check for the specified k-induction steps, otherwise defaults to 1.
        // FIXME: To enable.
        // this.k = this.targetConfig.verification.steps;
        // this.tactic = this.targetConfig.verification.tactic;
        
        System.out.println("*** Start generating Uclid code.");

        // Create the main reactor instance if there is a main reactor.
        createMainReactorInstance();

        // FIXME: Build reaction instance graph and causality graph
        populateDataStructures();

        // Create the src-gen directory
        setUpDirectories();

        // FIXME: Identify properties in the attributes.
        // FIXME: Calculate the completeness threshold for each property.
        int CT = 5; // Placeholder

        // Generate a Uclid model for each property.
        // for (String prop : this.properties) {
        //     generateUclidFile(prop);
        // }
        generateUclidFile("test", "bmc", CT);

        // FIXME:
        // Generate runner script
        // code = new StringBuilder()
        // var filename = outputDir.resolve("run.sh").toString
        // generateRunnerScript()
        // JavaGeneratorUtils.writeSourceCodeToFile(getCode, filename)
    }

    ////////////////////////////////////////////////////////////
    //// Protected methods

    /**
     * Generate the Uclid model and a runner script.
     */
    protected void generateUclidFile(String property, String tactic, int CT) {   
        try {  
            // Generate main.ucl and print to file
            code = new CodeBuilder();
            String filename = this.fileConfig.getSrcGenPath()
                                .resolve(tactic + "_" + property + ".ucl").toString();
            generateUclidCode(CT);
            code.writeToFile(filename);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }
    }

    /**
     * The main function that generates Uclid code.
     */
    protected void generateUclidCode(int CT) {
        code.pr(String.join("\n", 
            "/*******************************",
            " * Auto-generated UCLID5 model *",
            " ******************************/"
        ));

        code.pr("module main {");
        code.indent();

        // Timing semantics
        generateTimingSemantics();

        // Trace definition
        generateTraceDefinition(CT);

        // Reaction IDs and state variables
        generateReactionIdsAndStateVars();

        // Reactor semantics
        generateReactorSemantics();

        // Connections
        generateConnectionsAndActions();

        // Topology
        generateTopologicalMacros();
        generateTopology();

        // Initial Condition
        generateInitialConditions();

        // Abstractions (i.e. contracts)
        generateReactorAbstractions();
        generateReactionAbstractions();

        // FIXME: Properties

        // Control block
        generateControlBlock();

        code.unindent();
        code.pr("}");
    }

    /**
     * FIXME
     */
    protected void generateTimingSemantics() {
        code.pr(String.join("\n", 
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
            "// mstep() produces a mstep delay. zero() produces no delay.",
            "define tag_schedule(t : tag_t, i : interval_t) : tag_t",
            "= if (!isInf(t) && !isInf(i) && pi1(i) == 0 && pi2(i) == 1)",
            "    then { pi1(t), pi2(t) + 1 } // microstep delay",
            "    else ( if (!isInf(t) && !isInf(i) && pi1(i) == 0 && pi2(i) == 0)",
            "            then t // no delay",
            "            else (",
            "                if (!isInf(t) && pi1(i) > 0 && !isInf(i))",
            "                then { pi1(t) + pi1(i), 0 }",
            "                else inf()",
            "            ));",
            "",
            "// Deprecated.",
            "define tag_delay(t : tag_t, i : interval_t) : tag_t",
            "= if (!isInf(t) && !isInf(i))",
            "    then { pi1(t) + pi1(i), pi2(t) + pi2(i) }",
            "    else inf();",
            "",
            "// Only consider timestamp for now.",
            "define tag_diff(t1, t2: tag_t) : interval_t",
            "= if (!isInf(t1) && !isInf(t2))",
            "    then { pi1(t1) - pi1(t2), pi2(t1) - pi2(t2) }",
            "    else inf();",
            "\n"
        ));
    }

    /**
     * FIXME
     */
    protected void generateTraceDefinition(int CT) {
        // Define various constants.
        code.pr(String.join("\n", 
            "/********************",
            " * Trace Definition *",
            " *******************/",
            "const START : integer = 0;",
            "const END : integer = " + String.valueOf(CT-1) + ";",
            "",
            "// trace length = k + N",
            "const k : integer = 1;    // 1-induction should be enough.",
            "const N : integer = " + String.valueOf(CT) + ";" + "// The property bound",
            "\n"
        ));

        // Define trace indices as a group,
        // so that we can use finite quantifiers.
        code.pr("group indices : integer = {");
        code.indent();
        for (int i = 0; i < CT; i++) {
            code.pr(String.valueOf(i) + (i == CT-1? "" : ","));
        }
        code.unindent();
        code.pr("};\n\n");

        // FIXME: Let's see if in_range() can be removed altogether.
        // define in_range(num : integer) : boolean
        // = num >= START && num <= END;

        // Define step, event, and trace types.
        code.pr(String.join("\n",
            "// Define step and event types.",
            "type step_t = integer;",
            "type event_t = { rxn_t, tag_t, state_t, trigger_t };",
            "",
            "// Create a bounded trace with length " + String.valueOf(CT)
        ));
        code.pr("type trace_t = {");
        code.indent();
        for (int i = 0; i < CT; i++) {
            code.pr("event_t" + (i == CT-1? "" : ","));
        }
        code.unindent();
        code.pr("};\n");

        // Declare start time and trace.
        code.pr(String.join("\n", 
            "// Declare start time.",
            "var start_time : timestamp_t;",
            "",
            "// Declare trace.",
            "var trace : trace_t;"
        ));

        // Start generating helper macros.
        code.pr(String.join("\n", 
            "/*****************",
            " * Helper Macros *",
            " ****************/"
        ));

        // Define a tuple getter.
        // FIXME: Support this in Uclid.
        String initialStates = "";
        String initialTriggers = "";
        if (this.variableNames.size() > 0) {
            initialStates = "0, ".repeat(this.variableNames.size());
            initialStates = initialStates.substring(0, initialStates.length() - 2);
        } else {
            // Initialize a dummy variable just to make the code compile.
            initialStates = "0";
        }
        if (this.triggerNames.size() > 0) {
            initialTriggers = "false, ".repeat(this.triggerNames.size());
            initialTriggers = initialTriggers.substring(0, initialTriggers.length() - 2);
        } else {
            // Initialize a dummy variable just to make the code compile.
            initialTriggers = "false";
        }
        code.pr("// Helper macro that returns an element based on index.");
        code.pr("define get(tr : trace_t, i : step_t) : event_t =");
        for (int i = 0; i < CT; i++) {
            code.pr("if (i == " + String.valueOf(i) + ") then tr._" + String.valueOf(i+1) + " else (");
        }
        code.pr("{ NULL, inf(), { " + initialStates + " }, { " + initialTriggers + " } }");
        code.pr(")".repeat(CT) + ";\n");

        // Define an event getter from the trace.
        code.pr(String.join("\n", 
            "define elem(i : step_t) : event_t",
            "= get(trace, i);",
            "",
            "// projection macros",
            "define rxn      (i : step_t) : rxn_t        = elem(i)._1;",
            "define g        (i : step_t) : tag_t        = elem(i)._2;",
            "define s        (i : step_t) : state_t      = elem(i)._3;",
            "define t        (i : step_t) : trigger_t    = elem(i)._4;",
            "define isNULL   (i : step_t) : boolean      = rxn(i) == NULL;",
            ""
        ));
    }

    /**
     * FIXME
     */
    protected void generateReactionIdsAndStateVars() {
        // Encode the components and the logical delays
        // present in a reactor system.
        code.pr(String.join("\n",
            "/**********************************",
            " * Reaction IDs & State Variables *",
            " *********************************/",
            "",
            "//////////////////////////",
            "// Application Specific"
        ));

        // Enumerate over all reactions.
        code.pr(String.join("\n",
            "// Reaction ids",
            "type rxn_t = enum {"
        ));
        code.indent();
        System.out.println(this.reactionInstances);
        for (var rxn : this.reactionInstances) {
            // Print a list of reaction IDs.
            // Add a comma if not last.
            code.pr(rxn.getReaction().getFullNameWithJoiner("_") + "_" + String.valueOf(rxn.id) + ",");
        }
        code.pr("NULL");
        code.unindent();
        code.pr("};\n\n");

        // State variables and triggers
        // FIXME: expand to data types other than integer
        code.pr("type state_t = {");
        code.indent();
        if (this.variableNames.size() > 0) {
            for (var i = 0 ; i < this.variableNames.size(); i++) {
                code.pr("integer" + ((i++ == this.variableNames.size() - 1) ? "" : ",") + "// " + this.variableNames.get(i));
            }
        } else {
            code.pr(String.join("\n", 
                "// There are no ports or state variables.",
                "// Insert a dummy integer to make the model compile.",
                "integer"
            ));            
        }
        code.unindent();
        code.pr("};");
        code.pr("// State variable projection macros");
        for (var i = 0; i < this.variableNames.size(); i++) {
            code.pr("define " + this.variableNames.get(i) + "(s : state_t) : integer = s._" + i + ";");
        }
        code.pr("\n"); // Newline

        // A boolean tuple indicating whether triggers are present.
        code.pr("type trigger_t = {");
        code.indent();
        if (this.triggerNames.size() > 0) {
            for (var i = 0 ; i < this.triggerNames.size(); i++) {
                code.pr("boolean" + ((i++ == this.triggerNames.size() - 1) ? "" : ",") + "// " + this.variableNames.get(i));
            }
        } else {
            code.pr(String.join("\n", 
                "// There are no triggers.",
                "// Insert a dummy boolean to make the model compile.",
                "boolean"
            ));            
        }
        code.unindent();
        code.pr("};");
    }

    /**
     * FIXME
     */
    protected void generateReactorSemantics() {

    }

    /**
     * FIXME
     */
    protected void generateConnectionsAndActions() {

    }

    /**
     * FIXME
     */
    protected void generateTopologicalMacros() {

    }

    /**
     * FIXME
     */
    protected void generateTopology() {

    }

    /**
     * FIXME
     */
    protected void generateInitialConditions() {

    }

    /**
     * FIXME
     */
    protected void generateReactorAbstractions() {

    }

    /**
     * FIXME
     */
    protected void generateReactionAbstractions() {

    }

    /**
     * FIXME
     */
    protected void generateControlBlock() {

    }

    ////////////////////////////////////////////////////////////
    //// Private methods

    /**
     * If a main or federated reactor has been declared, create a ReactorInstance
     * for this top level. This will also assign levels to reactions, then,
     * if the program is federated, perform an AST transformation to disconnect
     * connections between federates.
     */
    private void createMainReactorInstance() {
        if (this.mainDef != null) {
            if (this.main == null) {
                // Recursively build instances. This is done once because
                // it is the same for all federates.
                this.main = new ReactorInstance(toDefinition(mainDef.getReactorClass()), errorReporter,
                    this.unorderedReactions);
                var reactionInstanceGraph = this.main.assignLevels();
                if (reactionInstanceGraph.nodeCount() > 0) {
                    errorReporter.reportError("Main reactor has causality cycles. Skipping code generation.");
                    return;
                }
            }
            
            // FIXME: Is this needed?
            // Force reconstruction of dependence information.
            if (isFederated) {
                // Avoid compile errors by removing disconnected network ports.
                // This must be done after assigning levels.
                removeRemoteFederateConnectionPorts(main);
                // There will be AST transformations that invalidate some info
                // cached in ReactorInstance.
                this.main.clearCaches(false);
            }
        }
    }

    private void setUpDirectories() {
        // Make sure the target directory exists.
        var targetDir = this.fileConfig.getSrcGenPath();
        try {
            Files.createDirectories(targetDir);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }
        System.out.println("The models will be located in: " + targetDir);
    }

    /**
     * Populate the data structures.
     */
    private void populateDataStructures() {
        // Construct graphs
        this.reactionInstanceGraph = new ReactionInstanceGraph(this.main, false);
        
        // Collect reactions from the reaction graph.
        this.reactionInstances = this.reactionInstanceGraph.nodes();
    }

    /////////////////////////////////////////////////
    //// Functions from generatorBase
    
    @Override
    public Target getTarget() {
        return Target.C; // FIXME: How to make this target independent? Target.ALL does not work.
    }
     
    @Override
    public TargetTypes getTargetTypes() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub");
    }   

    @Override
    public String generateDelayBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub");
    }

    @Override
    public String generateForwardBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub");
    }

    @Override
    public String generateDelayGeneric() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub");
    }
}