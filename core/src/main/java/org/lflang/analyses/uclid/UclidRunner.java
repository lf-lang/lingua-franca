package org.lflang.analyses.uclid;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.lflang.analyses.statespace.StateInfo;
import org.lflang.analyses.statespace.Tag;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.util.LFCommand;

/** (EXPERIMENTAL) Runner for Uclid5 models. */
public class UclidRunner {

  /** A list of paths to the generated files */
  List<Path> filePaths;

  /** The directory where the generated files are placed */
  public Path outputDir;

  /** A factory for compiler commands. */
  GeneratorCommandFactory commandFactory;

  /** A UclidGenerator instance */
  UclidGenerator generator;

  // Constructor
  public UclidRunner(UclidGenerator generator) {
    this.generator = generator;
    this.commandFactory =
        new GeneratorCommandFactory(
            generator.context.getErrorReporter(), generator.context.getFileConfig());
  }

  /** Parse information from an SMT model for a step in the trace. */
  public StateInfo parseStateInfo(String smtStr) {
    StateInfo info = new StateInfo();

    // Check for any let bindings.
    Pattern p =
        Pattern.compile(
            "\\(let \\(\\((a!\\d+) \\(_tuple_\\d+ ((.)+?)\\)\\)\\)(\\\\n"
                + "|\\s)+\\(_tuple_\\d+ ((.|\\n"
                + ")+)\\)");
    HashMap<String, String> symbolTable = new HashMap<>();
    Matcher m = p.matcher(smtStr.strip());
    String itemized = "";
    if (m.find()) {
      // FIXME: Handle multiple let bindings.
      String symbol = m.group(1).strip();
      String value = m.group(2).strip();
      symbolTable.put(symbol, value);
      itemized = m.group(5).strip();
    } else {
      // Remove the outer tuple layer.
      p = Pattern.compile("\\(_tuple_\\d+((.|\\n)*)\\)");
      m = p.matcher(smtStr.strip());
      m.find();
      itemized = m.group(1).strip();
    }

    // Process each sub-tuple by matching (_tuple_n ...)
    // or matching a!n, where n is an integer.
    p = Pattern.compile("(a!\\d+)|\\(_tuple_\\d+((\\s|\\\\n|\\d|\\(- \\d+\\)|true|false)+)\\)");
    m = p.matcher(itemized);

    // Reactions
    m.find();
    String reactionsStr = "";
    // Found a let binding.
    if (m.group(1) != null) {
      reactionsStr = symbolTable.get(m.group(1)).strip();
    }
    // The rest falls into group 2.
    else {
      reactionsStr = m.group(2).strip();
    }
    String[] reactions = reactionsStr.split("\\s+");
    // Iterating over generator lists avoids accounting for
    // the single dummy Uclid variable inserted earlier.
    for (int i = 0; i < generator.reactionInstances.size(); i++) {
      if (reactions[i].equals("true"))
        info.reactions.add(generator.reactionInstances.get(i).getReaction().getFullName());
    }

    // Time tag
    m.find();
    String tagStr = "";
    // Found a let binding.
    if (m.group(1) != null) tagStr = symbolTable.get(m.group(1)).strip();
    // The rest falls into group 2.
    else tagStr = m.group(2).strip();
    String[] tag = tagStr.split("\\s+");
    info.tag = new Tag(Long.parseLong(tag[0]), Long.parseLong(tag[1]), false);

    // Variables
    // Currently all integers.
    // Negative numbers could appear.
    m.find();
    String variablesStr = "";
    // Found a let binding.
    if (m.group(1) != null) variablesStr = symbolTable.get(m.group(1)).strip();
    // The rest falls into group 2.
    else variablesStr = m.group(2).strip();
    String[] variables = variablesStr.replaceAll("\\(-\\s(.*?)\\)", "-$1").split("\\s+");
    for (int i = 0; i < generator.namedInstances.size(); i++) {
      info.variables.put(generator.namedInstances.get(i).getFullName(), variables[i]);
    }

    // Triggers
    m.find();
    String triggersStr = "";
    // Found a let binding.
    if (m.group(1) != null) triggersStr = symbolTable.get(m.group(1)).strip();
    // The rest falls into group 2.
    else triggersStr = m.group(2).strip();
    String[] triggers = triggersStr.split("\\s+");
    for (int i = 0; i < generator.triggerInstances.size(); i++) {
      info.triggers.put(generator.triggerInstances.get(i).getFullName(), triggers[i]);
    }

    // Actions scheduled
    m.find();
    String scheduledStr = "";
    // Found a let binding.
    if (m.group(1) != null) scheduledStr = symbolTable.get(m.group(1)).strip();
    // The rest falls into group 2.
    else scheduledStr = m.group(2).strip();
    String[] scheduled = scheduledStr.split("\\s+");
    for (int i = 0; i < generator.actionInstances.size(); i++) {
      info.scheduled.put(generator.actionInstances.get(i).getFullName(), scheduled[i]);
    }

    // Scheduled payloads
    // Currently all integers.
    // Negative numbers could appear.
    m.find();
    String payloadsStr = "";
    // Found a let binding.
    if (m.group(1) != null) payloadsStr = symbolTable.get(m.group(1)).strip();
    // The rest falls into group 2.
    else payloadsStr = m.group(2).strip();
    String[] payloads = payloadsStr.replaceAll("\\(-\\s(.*?)\\)", "-$1").split("\\s+");
    for (int i = 0; i < generator.actionInstances.size(); i++) {
      info.payloads.put(generator.actionInstances.get(i).getFullName(), payloads[i]);
    }

    return info;
  }

  /**
   * Run all the generated Uclid models, report outputs, and generate counterexample trace diagrams.
   */
  public void run() {
    for (Path path : generator.generatedFiles) {
      // Execute uclid for each property.
      LFCommand command =
          commandFactory.createCommand(
              "uclid",
              List.of(
                  path.toString(),
                  // Any counterexample will be in <path.toString()>.json
                  "--json-cex",
                  path.toString()),
              generator.outputDir);
      command.run();

      String output = command.getOutput().toString();
      boolean valid = !output.contains("FAILED");
      if (valid) {
        System.out.println("Valid!");
      } else {
        System.out.println("Not valid!");
        try {
          // Read from the JSON counterexample (cex).
          String cexJSONStr =
              Files.readString(Paths.get(path.toString() + ".json"), StandardCharsets.UTF_8);
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
            } catch (JSONException e) {
              stepStr = trace.getString("-");
            }
            System.out.println("============ Step " + i + " ============");
            StateInfo info = parseStateInfo(stepStr);
            info.display();
          }
        } catch (IOException e) {
          System.out.println("ERROR: Not able to read from " + path.toString());
        }
      }

      // If "expect" is set, check if the result matches it.
      // If not, exit with error code 1.
      String expect = generator.expectations.get(path);
      if (expect != null) {
        boolean expectValid = Boolean.parseBoolean(expect);
        if (expectValid != valid) {
          System.out.println(
              "ERROR: The expected result does not match the actual result. Expected: "
                  + expectValid
                  + ", Result: "
                  + valid);
          System.exit(1);
        }
      }
    }
  }
}
