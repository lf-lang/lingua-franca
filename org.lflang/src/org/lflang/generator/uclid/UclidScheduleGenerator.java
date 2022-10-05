// An SMT-based static schedule generator powered by Uclid5 and Z3

/*************
Copyright (c) 2019-2022, The University of California at Berkeley.

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

package org.lflang.generator.uclid;

import java.io.BufferedReader;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import java.util.regex.MatchResult;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactionInstance.Runtime;
import org.lflang.generator.ReactionInstanceGraph;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

/**
 * An SMT-based static schedule generator powered by Uclid5 and Z3
 * 
 * @author {Shaokai Lin <shaokai@berkeley.edu>}
 */
public class UclidScheduleGenerator {
    ////////////////////////////////////////////
    //// Protected fields

    /** The current file configuration. */
    protected FileConfig fileConfig;

    /** A error reporter for reporting any errors or warnings during the code generation */
    public ErrorReporter errorReporter;

    /** The main (top-level) reactor instance. */
    public ReactorInstance main;

    /** Reaction instance graph that contains the set of all reactions in the LF program */
    public ReactionInstanceGraph reactionInstanceGraph;

    /** The target properties of the LF program */
    public TargetConfig targetConfig;

    /** The main place to put generated code. */
    protected CodeBuilder code = new CodeBuilder();

    ////////////////////////////////////////////
    //// Private fields
    public UclidScheduleGenerator(FileConfig fileConfig, ErrorReporter errorReporter, ReactorInstance main, TargetConfig targetConfig) {
        this.fileConfig = fileConfig;
        this.errorReporter = errorReporter;
        this.main = main;
        this.reactionInstanceGraph = new ReactionInstanceGraph(this.main, false);
        this.targetConfig = targetConfig;
    }

    private CodeBuilder generateUclidEncoding() {
        var uclidCode = new CodeBuilder();

        // The module declaration
        uclidCode.pr("module main {");
        uclidCode.pr("");
        uclidCode.indent();

        /*****************************
         * Types, variables, getters *
         *****************************/
        uclidCode.pr(String.join("\n", 
            "/*****************************",
            " * Types, variables, getters *",
            " *****************************/"
        ));
        uclidCode.pr("");

        // Declare the set of reactions.
        uclidCode.pr("// Declare the set of reactions.");
        uclidCode.pr("type task_t = enum {");
        uclidCode.indent();
        for (var rxn : this.reactionInstanceGraph.nodes()) {
            // Replace "." and " " with "_".
            uclidCode.pr("rxn_" + rxn.getReactionID() + ", //" + rxn.getFullName());
        }
        uclidCode.pr("NULL");
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Declare worker schedule.
        uclidCode.pr("// Declare worker schedule.");
        uclidCode.pr("type schedule_t = {");
        uclidCode.indent();
        uclidCode.pr(String.join(", ", Collections.nCopies(this.reactionInstanceGraph.nodeCount(), "task_t")));
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Declare workers.
        uclidCode.pr("// Declare workers.");
        uclidCode.pr("type workers_t = {");
        uclidCode.indent();
        uclidCode.pr(String.join(", ", Collections.nCopies(this.targetConfig.workers, "schedule_t")));
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Define a group of task indices.
        uclidCode.pr("// Define a group of task indices.");
        uclidCode.pr("group task_indices : integer = {");
        uclidCode.indent();
        uclidCode.pr(String.join(", ",
            IntStream.range(0, this.reactionInstanceGraph.nodeCount())
            .boxed()
            .collect(Collectors.toList())
            .stream()
            .map(String::valueOf)
            .collect(Collectors.toList())
        ));
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Define a group of worker indices.
        uclidCode.pr("// Define a group of worker indices.");
        uclidCode.pr("group worker_indices : integer = {");
        uclidCode.indent();
        uclidCode.pr(String.join(", ",
            IntStream.range(0, this.targetConfig.workers)
            .boxed()
            .collect(Collectors.toList())
            .stream()
            .map(String::valueOf)
            .collect(Collectors.toList())
        ));
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Compute the most generic schedule.
        // Define the current task set assuming all reactions are triggered.
        uclidCode.pr("// Compute the most generic schedule.");
        uclidCode.pr("// Define the current task set assuming all reactions are triggered.");
        uclidCode.pr("group current_task_set : task_t = {");
        uclidCode.indent();
        for (var rxn : this.reactionInstanceGraph.nodes()) {
            // Replace "." and " " with "_".
            uclidCode.pr("rxn_" + rxn.getReactionID() + ", //" + rxn.getFullName());
        }
        uclidCode.pr("NULL");
        uclidCode.unindent();
        uclidCode.pr("};");
        uclidCode.pr("");

        // Declare workers.
        uclidCode.pr("// Declare workers.");
        uclidCode.pr("var workers : workers_t;");
        uclidCode.pr("");

        // Get a task from a worker schedule.
        uclidCode.pr("// Get a task from a worker schedule.");
        uclidCode.pr("define getT(s : schedule_t, i : integer) : task_t =");
        uclidCode.indent();
        for (var i = 0; i < this.reactionInstanceGraph.nodeCount(); i++) {
            uclidCode.pr("(if (i == " + i + ") then s._" + (i+1) + " else");
        }
        uclidCode.pr("NULL");
        uclidCode.unindent();
        uclidCode.pr(String.join("", Collections.nCopies(this.reactionInstanceGraph.nodeCount(), ")")) + ";");
        uclidCode.pr("");

        // Get a worker from workers.
        uclidCode.pr("// Get a worker from workers.");
        uclidCode.pr("define getW(workers : workers_t, w : integer) : schedule_t =");
        uclidCode.indent();
        for (var i = 0; i < this.targetConfig.workers-1; i++) {
            uclidCode.pr("(if (w == " + i + ") then workers._" + (i+1) + " else");
        }
        uclidCode.pr("workers._" + this.targetConfig.workers);
        uclidCode.unindent();
        uclidCode.pr(String.join("", Collections.nCopies(this.targetConfig.workers-1, ")")) + ";");
        uclidCode.pr("");

        // Condense the two getters above.
        uclidCode.pr("// Condense the two getters above.");
        uclidCode.pr(String.join("\n", 
            "define get(w, i : integer) : task_t",
            "    = getT(getW(workers, w), i);"
        ));
        uclidCode.pr("");

        /***************
         * Constraints *
         ***************/
        uclidCode.pr(String.join("\n", 
            "/***************",
            " * Constraints *",
            " ***************/"
        ));
        uclidCode.pr("");

        // The schedules cannot be all empty.
        uclidCode.pr("// The schedules cannot be all empty.");
        uclidCode.pr(String.join("\n", 
            "axiom(finite_exists (w : integer) in worker_indices ::",
            "    (finite_exists (i : integer) in task_indices ::",
            "        get(w, i) != NULL));"
        ));
        uclidCode.pr("");

        // Each reaction only appears once.
        uclidCode.pr("// Each reaction only appears once.");
        uclidCode.pr(String.join("\n", 
            "axiom(finite_forall (w1: integer) in worker_indices ::",
            "    (finite_forall (w2 : integer) in worker_indices ::",
            "    (finite_forall (i : integer) in task_indices ::",
            "    (finite_forall (j : integer) in task_indices ::",
            "        (get(w1, i) != NULL)",
            "        ==> ((w1 == w2 && i == j) <==>",
            "        (get(w1, i) == get(w2, j)))",
            "    ))));"
        ));
        uclidCode.pr("");

        // Each reaction appears at least once.
        uclidCode.pr("// Each reaction appears at least once.");
        uclidCode.pr(String.join("\n", 
            "axiom(finite_forall (task : task_t) in current_task_set ::",
            "    (finite_exists (w : integer) in worker_indices ::",
            "    (finite_exists (i : integer) in task_indices ::",
            "        get(w, i) == task",
            "    )));"
        ));
        uclidCode.pr("");

        // Dependency graph (DAG) with reactions as nodes and
        // precedence relations as edges.
        uclidCode.pr(String.join("\n", 
            "// Dependency graph (DAG) with reactions as nodes and",
            "// precedence relations as edges."
        ));
        uclidCode.pr("define precedes(t1, t2 : task_t) : boolean = false");
        uclidCode.indent();
        // Iterate through all the reactions and get their downstream reactions.
        for (var rxn : this.reactionInstanceGraph.nodes()) {
            var downstream = this.reactionInstanceGraph.getDownstreamAdjacentNodes(rxn);
            for (var ds : downstream) {
                uclidCode.pr("|| (t1 == " + "rxn_" + rxn.getReactionID()
                    + " && t2 == " + "rxn_" + ds.getReactionID() + ")"
                    + " // " + rxn.getFullName().replaceAll("(\\.| )", "_")
                    + " -> " + ds.getFullName().replaceAll("(\\.| )", "_"));
            }
        }
        uclidCode.unindent();
        uclidCode.pr(";");
        uclidCode.pr("");

        // The worker schedules need to respect the dependency graph.
        uclidCode.pr("// The worker schedules need to respect the dependency graph.");
        uclidCode.pr(String.join("\n", 
            "axiom(finite_forall (w1: integer) in worker_indices ::",
            "    (finite_forall (w2 : integer) in worker_indices ::",
            "    (finite_forall (i : integer) in task_indices ::",
            "    (finite_forall (j : integer) in task_indices ::",
            "        (precedes(get(w1, i), get(w2, j))) ==> (i < j)",
            "    ))));"
        ));
        uclidCode.pr("");

        /*****************
         * Optimizations *
         *****************/
        uclidCode.pr(String.join("\n", 
            "/*****************",
            " * Optimizations *",
            " *****************/"
        ));
        uclidCode.pr("");

        // Optimization 1: parallelization
        uclidCode.pr("// Optimization 1: maximize parallelization");
        uclidCode.pr(String.join("\n",
            "// A macro function that counts the number of reactions",
            "// that could run in parallel at an instant."));
        uclidCode.pr("define countP(step : integer) : integer = 0");
        uclidCode.indent();
        for (var i = 0; i < this.targetConfig.workers; i++) {
            uclidCode.pr("+ (if (get(" + i + ", step) != NULL) then 1 else 0)");
        }
        uclidCode.unindent();
        uclidCode.pr(";");
        uclidCode.pr("");

        // Use sum of square to compute the parallel metric.
        uclidCode.pr("// Use sum of square to compute the parallel metric.");
        uclidCode.pr("var parallel_metric : integer;");
        uclidCode.pr("axiom(parallel_metric == 1");
        uclidCode.indent();
        for (var i = 0; i < this.reactionInstanceGraph.nodeCount(); i++) {
            uclidCode.pr("+ (countP(" + i + ") * countP(" + i + "))");
        }
        uclidCode.unindent();
        uclidCode.pr(");");
        uclidCode.pr("");

        // Optimization 2: spatial locality
        uclidCode.pr("// TODO: Optimization 2: spatial locality");

        // Property
        uclidCode.pr("property prop : !(true);");
        uclidCode.pr("");

        // The control block
        uclidCode.pr(String.join("\n", 
            "control {",
            "    v = unroll(0);",
            "    check;",
            "    print_results;",
            "    v.print_cex;",
            "}"
        ));

        // End the module declaration
        uclidCode.unindent();
        uclidCode.pr("}");

        return uclidCode;
    }

    public String generateSmtScheduleModel() {
        // Create temp folder
        var tempFolder  = fileConfig.getSrcGenPath() + File.separator + "temp";
        try {
            FileUtil.deleteDirectory(Paths.get(tempFolder));
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Generate Uclid encoding
        var uclidFile   = tempFolder + File.separator + "schedule.ucl";
        var uclidCode   = this.generateUclidEncoding();
        try {
            uclidCode.writeToFile(uclidFile);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Compile Uclid encoding to a SMT file.
        LFCommand cmdCompileUclid = LFCommand.get(
            "uclid",
            List.of(uclidFile, "-g", "smt"),
            false,
            Paths.get(tempFolder)
        );
        cmdCompileUclid.run();

        // List .smt files in the directory.
        File dir = new File(tempFolder);
        File [] smtFiles = dir.listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File dir, String name) {
                return name.endsWith(".smt");
            }
        });
        // Assuming that there is only 1 .smt file in the directory.
        // Load the generated file into a string.
        String smtStr = "";
        try {
            smtStr = Files.readString(Paths.get(smtFiles[0].getAbsolutePath()), StandardCharsets.US_ASCII);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // Remove Uclid variable prefixes using regex.
        smtStr = smtStr.replaceAll("initial_([0-9]+)_", ""); // or "initial_\\d+_", \\ escapes \.
        smtStr = smtStr.replaceAll("\\(check-sat\\)", "");
        smtStr = smtStr.replaceAll("\\(get-info :all-statistics\\)", "");

        // Add optimization objectives for parallelization.
        smtStr += "(maximize parallel_metric)\n";

        // Add directives.
        smtStr += String.join("\n", 
            "(check-sat)",
            "(get-info :all-statistics)",
            "(get-model)",
            "(get-objectives)"
        );

        // Write SMT back to file (for debugging)
        var smtCode = new CodeBuilder();
        smtCode.pr(smtStr);
        try {
            smtCode.writeToFile(smtFiles[0].getAbsolutePath());
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }

        // FIXME: Factor this in LFCommand.java.
        // Run Z3 on the printed SMT file.
        StringBuilder sb = new StringBuilder();
        try {
            ProcessBuilder pb = new ProcessBuilder("z3", smtFiles[0].getAbsolutePath());
            final Process p=pb.start();
            BufferedReader br=new BufferedReader(new InputStreamReader(p.getInputStream()));
            String line;
            while((line=br.readLine())!=null) sb.append(line);
        } catch (IOException e) {
            Exceptions.sneakyThrow(e);
        }
        String model = sb.toString();

        return model;
    }

    public CodeBuilder generateScheduleCode() {
        CodeBuilder scheduleCode = new CodeBuilder();
        List<Integer> scheduleLengths = new ArrayList<Integer>();

        // This SMT model contains a set of worker schedules.
        String modelStr = generateSmtScheduleModel();

        // Use regex to parse generated schedules.
        String[] matches = Pattern.compile("\\(_tuple_0 ([A-Za-z0-9\\_\\s\\n]+)\\)")
                            .matcher(modelStr)
                            .results()
                            .map(MatchResult::group)
                            .toArray(String[]::new);

        // Process the matched strings.
        List<List<String>> schedules = new ArrayList<List<String>>();
        for (var i = 0; i < matches.length; i++) {
            System.out.println(matches[i]);
            String str = matches[i].replaceAll("(\\(|\\)|_tuple_0|NULL)", "").strip();
            if (str.isBlank()) {
                // Add an empty list to schedules
                schedules.add(new ArrayList());
            } else {
                schedules.add(Arrays.asList(str.split("\\s+")));
            }
        }     

        // Begin to generate static schedules.
        scheduleCode.pr("/* Auto-generated schedule */");

        // Generate executable worker schedules using the custom instruction set.
        int semaphore_count = 0;
        // Maps a pair of reaction IDs (i.e. <upstream ID, downstream ID>) to a semaphore id.
        Map<ReactionPair, Integer> semaphoreMap = new HashMap<ReactionPair, Integer>();
        // Calculate the semaphores needed and store them in a map
        // mapping from a pair of upstream-downstream reactions to the semaphore id.
        for (var w = 0; w < this.targetConfig.workers; w++) {
            for (var i = 0; i < schedules.get(w).size(); i++) {
                // Get the current reaction to be executed.
                long reactionID = Long.parseLong(schedules.get(w).get(i).split("_")[1]);
                var rxn = this.reactionInstanceGraph.getReactionByID(reactionID);
                
                // Check if this reaction has upstream reactions processed
                // by other threads. If so, add a "wait" instruction.
                var upstream = this.reactionInstanceGraph.getUpstreamAdjacentNodes(rxn);
                for (Runtime us : upstream) {
                    // Iterate over schedules other than the current schedule.
                    for (var w2 = 0; w2 < this.targetConfig.workers; w2++) {
                        // Skip if it is the current schedule.
                        if (w2 == w) continue;
                        // If another schedule contains the upstream reaction,
                        // add a semaphore.
                        if (schedules.get(w2).contains("rxn_" + us.getReactionID())) {
                            semaphoreMap.put(new ReactionPair(us.getReactionID(), // Upstream ID
                                                                reactionID),      // Downstream ID
                                            semaphore_count); // Semaphore index
                            semaphore_count++;
                        }
                    }
                }
            }
        }

        // Generate static schedules.
        scheduleCode.pr("// The static schedules");
        for (var w = 0; w < this.targetConfig.workers; w++) {
            int scheduleLength = 0;
            scheduleCode.pr("static const inst_t s1_w" + w + "[] = {");
            scheduleCode.indent();

            for (var i = 0; i < schedules.get(w).size(); i++) {
                String instructions = "";

                // Get the current reaction to be executed.
                var reactionID = Long.parseLong(schedules.get(w).get(i).split("_")[1]);

                // Add the "execute" instruction to the list.
                instructions += "{.inst='e', .op=" + reactionID + "}, "
                                + "// Execute "
                                + this.reactionInstanceGraph.getReactionByID(reactionID).getFullName()
                                + ".\n";
                scheduleLength++;

                // Generate "wait" and "notify" instructions.
                for (Map.Entry<ReactionPair,Integer> entry : semaphoreMap.entrySet()) {
                    ReactionPair reactionPair = entry.getKey();
                    int semaphoreID = entry.getValue();

                    // Check if reactionID is the upstream reaction.
                    // If so, generate a "notify."
                    if (reactionID == reactionPair.getUpstream()) {
                        instructions += "{.inst='n', .op=" + semaphoreID + "}, "
                                        + "// Notify semaphore " + semaphoreID + ".\n";
                        scheduleLength++;
                    }
                    // Check if reactionID is the downstream reaction.
                    // If so, generate a "wait."
                    else if (reactionID == reactionPair.getDownstream()) {
                        // Prepend "wait" instructions.
                        instructions = "{.inst='w', .op=" + semaphoreID + "}, "
                                        + "// Wait for semaphore " + semaphoreID + ".\n" 
                                        + instructions;
                        scheduleLength++;
                    }
                }

                // Finally, print all the instructions for this worker schedule.
                scheduleCode.pr(instructions);
            }

            // Generate "stop" instruction.
            scheduleCode.pr("{.inst='s', .op=0}");
            scheduleLength++;

            // Add the lengths to scheduleLengths
            scheduleLengths.add(scheduleLength);
            
            scheduleCode.unindent();
            scheduleCode.pr("};");
        }

        // Create an array to store the worker schedules.
        scheduleCode.pr("static const inst_t* s1[] = {");
        scheduleCode.indent();
        for (var w = 0; w < this.targetConfig.workers; w++) {
            scheduleCode.pr("s1_w" + w
                + (w == this.targetConfig.workers - 1 ? "" : ","));
        }
        scheduleCode.unindent();
        scheduleCode.pr("};");

        // Create a schedule array to store all the schedules
        // (currently only 1 generic schedule)
        // FIXME: Generalize to multiple schedules.
        scheduleCode.pr("static const inst_t** static_schedules[] = { s1 };");

        // Create an array to store the schedule lengths
        scheduleCode.pr("static const uint32_t s1_length[] = {");
        scheduleCode.indent();
        for (var w = 0; w < this.targetConfig.workers; w++) {
            scheduleCode.pr(scheduleLengths.get(w) 
                + (w == this.targetConfig.workers - 1 ? "" : ","));
        }
        scheduleCode.unindent();
        scheduleCode.pr("};");

        // Create an array to store the lengths of all schedules
        // (currently only the length of the generic schedule)
        // FIXME: Generalize to multiple schedules.
        scheduleCode.pr("static const uint32_t* schedule_lengths[] = { s1_length };");

        // Generate preambles (everything other than the schedule in the .h file).
        scheduleCode.pr("// The total number of reactions");
        scheduleCode.pr("static const int reaction_count = "
                        + this.reactionInstanceGraph.nodeCount() + ";");
        scheduleCode.pr("// The number of semaphores needed");
        scheduleCode.pr("static const int num_semaphores = " + semaphore_count + ";");

        return scheduleCode;
    }

    // Private helper class for storing a pair of reactions
    private class ReactionPair {
        private long upstreamID;
        private long downstreamID; 
        public ReactionPair(long upstream, long downstream) {
            this.upstreamID = upstream;
            this.downstreamID = downstream;
        }
        public long getUpstream() {
            return this.upstreamID;
        }
        public long getDownstream() {
            return this.downstreamID;
        }
    }
}
