/** 
 * Runner for Uclid models.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.generator.uclid;

import java.io.IOException;
import java.io.OutputStream;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.sim.Tag;
import org.lflang.util.LFCommand;

public class UclidRunner {

    /**
     * A list of paths to the generated files
     */
    List<Path> filePaths;

    /**
     * The directory where the generated files are placed
     */
    public Path outputDir;

    /**
     * A factory for compiler commands.
     */
    GeneratorCommandFactory commandFactory;

    /**
     * A UclidGenerator instance
     */
    UclidGenerator generator;

    // Constructor
    public UclidRunner(
        UclidGenerator generator,
        FileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        this.generator = generator;
        this.commandFactory =
            new GeneratorCommandFactory(errorReporter, fileConfig);
    }

    /**
     * Parse information from an SMT model
     * for a step in the trace.
     */
    public StateInfo parseStateInfo(String smtStr) {
        StateInfo info = new StateInfo();

        // Split the counterexample string by newline.
        String[] splitedSmtStr = smtStr.split("\\r?\\n");

        // Reactions
        Pattern p = Pattern.compile("\\(_tuple_\\d+ \\(_tuple_\\d+ ([^\\)]+)\\)");
        Matcher m = p.matcher(splitedSmtStr[0].strip());
        m.find();
        String[] reactions = m.group(1).strip().split("\\s+");
        for (int i = 0; i < reactions.length; i++) {
            if (reactions[i].equals("true"))
                info.reactions.add(generator.reactionInstances.get(i).getReaction().getFullName());
        }

        // Time tag
        p = Pattern.compile("\\(_tuple_\\d+([^\\)]+)\\)");
        m = p.matcher(splitedSmtStr[1].strip());
        m.find();
        String[] tag = m.group(1).strip().split("\\s+");
        info.tag = new Tag(
            Long.parseLong(tag[0]),
            Long.parseLong(tag[1]), false);

        // Variables
        m = p.matcher(splitedSmtStr[2].strip());
        m.find();
        String[] variables = m.group(1).strip().split("\\s+");
        for (int i = 0; i < variables.length; i++) {
            info.variables.put(generator.namedInstances.get(i).getFullName(), variables[i]);
        }

        // Triggers
        m = p.matcher(splitedSmtStr[3].strip());
        m.find();
        String[] triggers = m.group(1).strip().split("\\s+");
        for (int i = 0; i < triggers.length; i++) {
            info.triggers.put(generator.triggerInstances.get(i).getFullName(), triggers[i]);
        }

        return info;
    }

    /**
     * Run all the generated Uclid models, report outputs,
     * and generate counterexample trace diagrams.
     */
    public void run() {
        for (Path path : generator.generatedFiles) {
            // Execute uclid for each property.
            LFCommand command = commandFactory.createCommand(
                "uclid", List.of(
                    path.toString(),
                    // Any counterexample will be in <path.toString()>.json
                    "--json-cex", path.toString()),
                generator.outputDir);
            command.run();
            
            String output = command.getOutput().toString();
            boolean failed = output.contains("FAILED");
            if (failed) {
                System.out.println("Not valid!");
                try {
                    // Read from the JSON counterexample (cex).
                    String cexJSONStr = Files.readString(
                        Paths.get(path.toString() + ".json"),
                        StandardCharsets.UTF_8);
                    // System.out.println(cexJSONStr);
                    JSONObject cexJSON = new JSONObject(cexJSONStr);

                    //// Extract the counterexample trace from JSON.
                    // Get the first key "property_*"
                    Iterator<String> keys = cexJSON.keys();
                    String firstKey = keys.next();
                    JSONObject propertyObj = cexJSON.getJSONObject(firstKey);

                    // Get Uclid trace.
                    JSONArray uclidTrace = propertyObj.getJSONArray("trace");
                    
                    // Get the first step of the Uclid trace.
                    JSONObject uclidTraceStepOne = uclidTrace.getJSONObject(0);

                    // Get the actual trace defined in the verification model.
                    JSONObject trace = uclidTraceStepOne.getJSONArray("trace").getJSONObject(0);

                    String stepStr = "";
                    for (int i = 0; i <= generator.CT; i++) {
                        try {
                            stepStr = trace.getString(String.valueOf(i));
                        } catch(JSONException e) {
                            stepStr = trace.getString("-");
                        }
                        System.out.println("============ Step " + i + " ============");
                        StateInfo info = parseStateInfo(stepStr);
                        info.display();
                    }
                } catch (IOException e) {
                    System.out.println("ERROR: Not able to read from " + path.toString());
                }
            } else {
                System.out.println("Valid!");
            }
        }
    }
}