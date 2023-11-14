package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.EnclaveInfo;
import org.lflang.generator.EnclaveInfo.EnclaveConnection;
import org.lflang.generator.ReactorInstance;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TracingProperty;

/**
 * This class is in charge of code generating functions and global variables related to the enclaves
 * and environments. An environment is the context in which an enclave exists. Each enclave has its
 * own environment where queues, current tag, thread synchronization primitives etc. is stored.
 */
public class CEnclaveGenerator {

  /**
   * @param main The top-level reactor instance of the program.
   * @param lfModuleName The lfModuleName of the program.
   * @param messageReporter To report warnings and messages.
   */
  public CEnclaveGenerator(
      ReactorInstance main, String lfModuleName, MessageReporter messageReporter) {
    this.enclaves = CUtil.getEnclaves(main);
    this.lfModuleName = lfModuleName;
    this.messageReporter = messageReporter;

    // FIXME: Test for ZDC in the enclave graph
  }

  /** Retrieve the number of enclaves in the program. */
  public int numEnclaves() {
    return this.enclaves.size();
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

  private List<EnclaveInfo> enclaves = new ArrayList<>();
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
        "environment_t envs[_num_enclaves];");
  }

  /** Generate a function which returns a pointer to the first element of the environment array. */
  private String generateGetEnvironments() {
    return String.join(
        "\n",
        "// Update the pointer argument to point to the beginning of the environment array",
        "// and return the size of that array",
        "int _lf_get_environments(environment_t ** return_envs) {",
        "   (*return_envs) = (environment_t *) envs;",
        "   return _num_enclaves;",
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
    for (EnclaveInfo enclave : enclaves) {
      code.pr(enclave.getId() + ",");
    }
    code.pr("_num_enclaves");
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
    for (EnclaveInfo enclave : enclaves) {
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

    for (EnclaveInfo enclave : enclaves) {
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
  private String generateConnectionArrays(EnclaveInfo enclave) {
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
  private String generateDownstreamsArray(EnclaveInfo enclave) {
    CodeBuilder code = new CodeBuilder();

    List<EnclaveInfo> downstreams =
        enclave.downstreams.stream().map(EnclaveConnection::target).toList();
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
      for (EnclaveInfo downstream : downstreams) {
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
  private String generateUpstreamsArray(EnclaveInfo enclave) {
    CodeBuilder code = new CodeBuilder();
    List<EnclaveInfo> upstreams =
        enclave.upstreams.stream().map(EnclaveConnection::source).toList();
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
      for (EnclaveInfo upstream : upstreams) {
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
  private String generateUpstreamDelaysArray(EnclaveInfo enclave) {
    CodeBuilder code = new CodeBuilder();
    List<EnclaveConnection> upstreams = enclave.upstreams.stream().toList();
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
      for (EnclaveConnection upstream : upstreams) {
        String element;
        // If the connection has no after delay then we use the special value "NEVER".
        // A upstream delay of 0 means that there is a microstep delay.
        if (upstream.hasAfterDelay()) {
          element = String.valueOf(upstream.delay().toNanoSeconds());
        } else {
          element = "NEVER";
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
    for (EnclaveInfo enclave : enclaves) {
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
    for (EnclaveInfo enclave : enclaves) {
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
    for (EnclaveInfo enclave : enclaves) {
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
