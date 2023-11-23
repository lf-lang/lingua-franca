package org.lflang.generator.c;

import java.util.List;
import java.util.Set;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CEnclaveGraph.EnclaveConnection;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TracingProperty;

/**
 * This class is in charge of code generating functions and global variables related to the enclaves
 * and environments. An environment is the context in which an enclave exists. Each enclave has its
 * own environment where queues, current tag, thread synchronization primitives etc. is stored.
 */
public class CEnclaveGenerator {

  /**
   * @param main The main reactor instance of the program
   * @param enclaveMap A mapping from reactor instances to enclave instances
   * @param ast The AST transformation which has info about the enclave topology
   * @param lfModuleName
   * @param messageReporter
   */
  public CEnclaveGenerator(
      ReactorInstance main,
      ReactorEnclaveMap enclaveMap,
      CEnclavedReactorTransformation ast,
      String lfModuleName,
      MessageReporter messageReporter) {
    this.enclaveMap = enclaveMap;
    this.connGraph = new CEnclaveGraph(ast);
    this.lfModuleName = lfModuleName;
    this.messageReporter = messageReporter;
    this.connGraph.build(main, enclaveMap);

    // Here we test for zero-delay cycles in the enclave graph.
    if (connGraph.hasZeroDelayCycle()) {
      messageReporter
          .nowhere()
          .error("Found zero delay cycle between enclaves: `" + connGraph.buildCycleString() + "`");
    }
  }

  /** Retrieve the number of enclaves in the program. */
  public int numEnclaves() {
    return this.enclaveMap.numEnclaves();
  }

  /** Generate declarations in the main C file associated with environments. */
  public String generateDeclarations() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateEnvironmentEnum());
    code.pr(generateEnvironmentArray());
    return code.toString();
  }

  /** Generate the definitions on the main C file associated with environments and enclaves. */
  public String generateDefinitions(TargetConfig targetConfig) {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateCreateEnvironments(targetConfig));
    code.pr(generateGetEnvironments());
    code.pr(generateConnectionTopologyInfo());
    return code.toString();
  }

  /** A map from reactor instance to enclave instance. */
  private ReactorEnclaveMap enclaveMap;
  /** A graph of the enclave instances of the program. */
  private CEnclaveGraph connGraph;

  private final String lfModuleName;
  private final MessageReporter messageReporter;

  /**
   * Generate a static array of environment structs whose length matches the number of enclaves in
   * the program.
   */
  private String generateEnvironmentArray() {
    return String.join(
        "\n",
        "// The global array of environments associated with each enclave",
        "static environment_t "
            + CUtil.ENVIRONMENT_VARIABLE_NAME
            + "["
            + CUtil.NUM_ENVIRONMENT_VARIABLE_NAME
            + "];");
  }

  /** Generate a function which returns a pointer to the first element of the environment array. */
  private String generateGetEnvironments() {
    return String.join(
        "\n",
        "// Update the pointer argument to point to the beginning of the environment array",
        "// and return the size of that array",
        "int _lf_get_environments(environment_t ** return_envs) {",
        "   (*return_envs) = (environment_t *) " + CUtil.ENVIRONMENT_VARIABLE_NAME + ";",
        "   return " + CUtil.NUM_ENVIRONMENT_VARIABLE_NAME + ";",
        "}");
  }

  /**
   * Generate an enum mapping enclave names to the index of their associated environment in the
   * environment array.
   */
  private String generateEnvironmentEnum() {
    CodeBuilder code = new CodeBuilder();
    code.pr("typedef enum {");
    code.indent();
    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      code.pr(enclave.getId() + ",");
    }
    code.pr(CUtil.NUM_ENVIRONMENT_VARIABLE_NAME);
    code.unindent();
    code.pr("} _enclave_id;");

    return code.toString();
  }

  /** Generate the function which initializes the environment struct for each enclave. */
  private String generateCreateEnvironments(TargetConfig targetConfig) {
    CodeBuilder code = new CodeBuilder();
    code.pr("// 'Create' and initialize the environments in the program");
    code.pr("void _lf_create_environments() {");
    code.indent();
    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      // Decide the number of workers to use. If this is the top-level
      // use the global variable _lf_number_of_workers which accounts for federation etc.
      String numWorkers = String.valueOf(enclave.numWorkers);
      if (enclave.getReactorInstance().isMainOrFederated()) {
        numWorkers = "_lf_number_of_workers";
      }

      // Figure out the name of the trace file.
      String traceFileName = "NULL";
      var tracing = targetConfig.get(TracingProperty.INSTANCE);
      if (tracing.isEnabled()) {
        if (tracing.traceFileName != null) {
          if (enclave.getReactorInstance().isMainOrFederated()) {
            traceFileName = "\"" + tracing.traceFileName + ".lft\"";
          } else {
            traceFileName = "\"" + tracing.traceFileName + enclave.getId() + ".lft\"";
          }
        } else {
          if (enclave.getReactorInstance().isMainOrFederated()) {
            traceFileName = "\"" + lfModuleName + ".lft\"";
          } else {
            traceFileName = "\"" + lfModuleName + enclave.getId() + ".lft\"";
          }
        }
      }

      code.pr(
          "environment_init(&"
              + CUtil.getEnvironmentStruct(enclave)
              + ","
              + "\""
              + enclave.getId()
              + "\""
              + ","
              + enclave.getId()
              + ","
              + numWorkers
              + ","
              + enclave.numTimerTriggers
              + ","
              + enclave.numStartupReactions
              + ","
              + enclave.numShutdownReactions
              + ","
              + enclave.numResetReactions
              + ","
              + enclave.numIsPresentFields
              + ","
              + enclave.numModalReactors
              + ","
              + enclave.numModalResetStates
              + ","
              + traceFileName
              + ");");
    }
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /** Generate arrays and functions for representing the topology of the enclaves. */
  private String generateConnectionTopologyInfo() {
    CodeBuilder code = new CodeBuilder();

    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      code.pr(generateConnectionArrays(enclave));
    }
    code.pr(generateConnectionGetFunctions());
    return code.toString();
  }

  /**
   * Generate the static arrays representing the connections and the delay between the enclaves
   *
   * @param enclave The enclave for which to generate the arrays.
   */
  private String generateConnectionArrays(CEnclaveInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateDownstreamsArray(enclave));
    code.pr(generateUpstreamsArray(enclave));
    code.pr(generateUpstreamDelaysArray(enclave));
    return code.toString();
  }

  /**
   * Generate the static array representing which enclaves are downstream of `enclave`.
   *
   * @param enclave The enclave for which to generate the array.
   */
  private String generateDownstreamsArray(CEnclaveInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    List<CEnclaveInstance> downstreams =
        connGraph.graph.getDownstreamOf(enclave).keySet().stream().toList();
    int numDownstream = downstreams.size();
    String encName = enclave.getId();
    String numDownstreamVar = (encName + "_num_downstream").toUpperCase();
    String downstreamVar = encName + "_downstream";

    code.prComment("Arrays representing the enclaves downstream of `" + downstreamVar + "`");
    code.pr("#define " + numDownstreamVar + " " + numDownstream);
    if (numDownstream == 0) {
      code.pr("// No downstreams, but 0-length arrays not allowed by MSVC (C2466)");
      code.pr("int " + downstreamVar + "[1] = {-1};");
    } else {
      code.pr("int " + downstreamVar + "[" + numDownstreamVar + "] = { ");
      code.indent();
      int idx = 0;
      for (CEnclaveInstance downstream : downstreams) {
        String element = downstream.getId();
        if (idx < numDownstream - 1) {
          element += ",";
        }
        code.pr(element);
        idx += 1;
      }
      code.unindent();
      code.pr("};");
    }

    return code.toString();
  }

  /**
   * Generate the static array representing which enclaves are upstream of `enclave`.
   *
   * @param enclave The enclave for which to generate the array.
   */
  private String generateUpstreamsArray(CEnclaveInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    List<CEnclaveInstance> upstreams =
        connGraph.graph.getUpstreamOf(enclave).keySet().stream().toList();
    int numUpstream = upstreams.size();
    String encName = enclave.getId();
    String numUpstreamVar = (encName + "_num_upstream").toUpperCase();
    String upstreamVar = encName + "_upstream";

    code.prComment("Arrays representing the enclaves upstream of `" + upstreamVar + "`");
    code.pr("#define " + numUpstreamVar + " " + numUpstream);
    if (numUpstream == 0) {
      code.pr("// No upstreams, but 0-length arrays not allowed by MSVC (C2466)");
      code.pr("int " + upstreamVar + "[1] = {-1};");
    } else {
      code.pr("int " + upstreamVar + "[" + numUpstreamVar + "] = { ");
      code.indent();
      int idx = 0;
      for (CEnclaveInstance upstream : upstreams) {
        String element = upstream.getId();
        if (idx < numUpstream - 1) {
          element += ",";
        }
        code.pr(element);
      }
      code.unindent();
      code.pr("};");
    }
    return code.toString();
  }

  /**
   * Generate the static array representing the delay on the connections between `enclave` and its
   * upstream enclaves.
   *
   * @param enclave The enclave.
   */
  private String generateUpstreamDelaysArray(CEnclaveInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    var upstreams = connGraph.graph.getUpstreamOf(enclave);
    int numUpstream = upstreams.size();
    String encName = enclave.getId();
    String numUpstreamVar = (encName + "_num_upstream").toUpperCase();
    String upstreamDelayVar = encName + "_upstream_delay";
    if (numUpstream == 0) {
      code.pr("// No upstreams, but 0-length arrays not allowed by MSVC (C2466)");
      code.pr("interval_t " + upstreamDelayVar + "[1] = {FOREVER};");
    } else {
      code.pr("interval_t " + upstreamDelayVar + "[" + numUpstreamVar + "] = { ");
      code.indent();
      int idx = 0;
      for (Set<EnclaveConnection> upstreamSet : upstreams.values()) {
        var element = "NEVER";
        // If the connection has no after delay then we use the special value "NEVER".
        // A upstream delay of 0 means that there is a microstep delay.
        TimeValue delay = TimeValue.NEVER;
        for (EnclaveConnection conn : upstreamSet) {
          if (conn.hasAfterDelay()) {
            if (delay.isEarlierThan(conn.delay())) {
              delay = conn.delay();
              element = String.valueOf(delay.toNanoSeconds());
            }
          }
        }
        if (idx < numUpstream - 1) {
          element += ",";
        }
        code.pr(element);
      }
      code.unindent();
      code.pr("};");
    }
    return code.toString();
  }

  /**
   * Generates the functions used to get the upstream, downstreams and upstream delays of an
   * enclave.
   */
  private String generateConnectionGetFunctions() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateGetUpstreamOf());
    code.pr(generateGetDownstreamOf());
    code.pr(generateGetUpstreamDelayOf());
    return code.toString();
  }

  /**
   * Generate the `_lf_get_downstream_of` function which points the `result` argument to the
   * beginning of an array of the id`s of the enclaves downstream of `enclave_id`.
   */
  private String generateGetDownstreamOf() {
    CodeBuilder code = new CodeBuilder();
    code.prComment(
        "Writes a pointer to the array of downstream enclaves into `result` and returns the"
            + " length");
    code.pr("int _lf_get_downstream_of(int enclave_id, int ** result) {");
    code.indent();
    code.pr("int num_downstream;");
    code.pr("int* downstream;");
    code.pr("switch(enclave_id) { ");
    code.indent();
    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      String enclaveNumDownstream = (enclave.getId() + "_num_downstream").toUpperCase();
      code.pr("case " + enclave.getId() + ":");
      code.indent();
      code.pr("num_downstream = " + enclaveNumDownstream + ";");
      code.pr("downstream = &" + enclave.getId() + "_downstream[0];");
      code.pr("break;");
      code.unindent();
    }
    code.pr("default:");
    code.indent();
    code.pr("lf_print_error_and_exit(\"Illegal enclave_id %u\", enclave_id);");
    code.pr("break;");
    code.unindent();
    code.pr("}");
    code.unindent();
    code.pr("(*result) = downstream;");
    code.pr("return num_downstream;");
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /**
   * Generates the function `_lf_get_upstream_of` which points `result` to an array of enclaves
   * upstream of `enclave_id`.
   */
  private String generateGetUpstreamOf() {
    CodeBuilder code = new CodeBuilder();
    code.prComment(
        "Writes a pointer to the array of upstream enclaves into `result` and returns the length");
    code.pr("int _lf_get_upstream_of(int enclave_id, int ** result) {");
    code.indent();
    code.pr("int num_upstream;");
    code.pr("int* upstream;");
    code.pr("switch(enclave_id) { ");
    code.indent();
    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      String enclaveNumUpstream = (enclave.getId() + "_num_upstream").toUpperCase();
      code.pr("case " + enclave.getId() + ":");
      code.indent();
      code.pr("num_upstream = " + enclaveNumUpstream + ";");
      code.pr("upstream = &" + enclave.getId() + "_upstream[0];");
      code.pr("break;");
      code.unindent();
    }
    code.pr("default:");
    code.indent();
    code.pr("lf_print_error_and_exit(\"Illegal enclave_id %u\", enclave_id);");
    code.pr("break;");
    code.unindent();
    code.pr("}");
    code.unindent();
    code.pr("(*result) = upstream;");
    code.pr("return num_upstream;");
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /**
   * Generates the `_lf_get_upstream_delay_of()` function which points `result` to an array of the
   * upstream delays for `enclave_id`.
   */
  private String generateGetUpstreamDelayOf() {
    CodeBuilder code = new CodeBuilder();
    code.prComment(
        "Writes a pointer to the array of upstream delays into `result` and returns the length");
    code.pr("int _lf_get_upstream_delay_of(int enclave_id, interval_t ** result) {");
    code.indent();
    code.pr("int num_upstream;");
    code.pr("interval_t* delay;");
    code.pr("switch(enclave_id) { ");
    code.indent();
    for (CEnclaveInstance enclave : enclaveMap.getEnclaves()) {
      String enclaveNumUpstream = (enclave.getId() + "_num_upstream").toUpperCase();
      code.pr("case " + enclave.getId() + ":");
      code.indent();
      code.pr("num_upstream = " + enclaveNumUpstream + ";");
      code.pr("delay = &" + enclave.getId() + "_upstream_delay[0];");
      code.pr("break;");
      code.unindent();
    }
    code.pr("default:");
    code.indent();
    code.pr("lf_print_error_and_exit(\"Illegal enclave_id %u\", enclave_id);");
    code.pr("break;");
    code.unindent();
    code.pr("}");
    code.unindent();
    code.pr("(*result) = delay;");
    code.pr("return num_upstream;");
    code.unindent();
    code.pr("}");
    return code.toString();
  }
}
