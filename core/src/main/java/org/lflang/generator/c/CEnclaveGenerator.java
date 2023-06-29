package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.ErrorReporter;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CEnclaveGraph.EnclaveConnection;

// FIXME: This file should be renamed to CEnclaveGenerator or something.
/**
 * This class is in charge of code generating functions and global variables related to the
 * environments
 */
public class CEnclaveGenerator {

  /**
   * @param main The top-level reactor instance of the program
   * @param targetConfig The target config of the program
   * @param lfModuleName The lfModuleName of the program
   */
  public CEnclaveGenerator(
      ReactorInstance main,
      TargetConfig targetConfig,
      String lfModuleName,
      ErrorReporter errorReporter) {
    this.enclaves = CUtil.getEnclaves(main);
    this.targetConfig = targetConfig;
    this.lfModuleName = lfModuleName;
    this.errorReporter = errorReporter;
    this.enclaveGraph = new CEnclaveGraph(this.enclaves);

    // FIXME: This is not really the best place for the cycle detection. The problem is that the
    // EnclaveGraph
    //  assumes that is being run on the post AST transformation graph. So cannot be used in the
    // ValidatorCheck (which is pre-
    if (enclaveGraph.hasZeroDelayCycles()) {
      errorReporter.reportError(
          "Found zero delay cycle between enclaves: `" + enclaveGraph.buildCycleString() + "`");
    }
  }

  public int numEnclaves() {
    return this.enclaves.size();
  }

  public String generateDeclarations() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateEnvironmentEnum());
    code.pr(generateEnvironmentArray());
    return code.toString();
  }

  public String generateDefinitions() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateCreateEnvironments());
    code.pr(generateGetEnvironments());
    code.pr(generateConnectionTopologyInfo());
    return code.toString();
  }

  private List<ReactorInstance> enclaves = new ArrayList<>();
  private final TargetConfig targetConfig;
  private final String lfModuleName;

  private final ErrorReporter errorReporter;

  private final CEnclaveGraph enclaveGraph;

  private String generateEnvironmentArray() {
    return String.join(
        "\n",
        "// The global array of environments associated with each enclave",
        "environment_t envs[_num_enclaves];");
  }

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

  private String generateEnvironmentEnum() {
    CodeBuilder code = new CodeBuilder();
    code.pr("typedef enum {");
    code.indent();
    for (ReactorInstance enclave : enclaves) {
      code.pr(CUtil.getEnvironmentId(enclave) + ",");
    }
    code.pr("_num_enclaves");
    code.unindent();
    code.pr("} _enclave_id;");

    return code.toString();
  }

  private String generateCreateEnvironments() {
    CodeBuilder code = new CodeBuilder();
    code.pr("// 'Create' and initialize the environments in the program");
    code.pr("void _lf_create_environments() {");
    code.indent();
    for (ReactorInstance enclave : enclaves) {
      // Decide the number of workers to use. If this is the top-level
      // use the global variable _lf_number_of_workers which accounts for federation etc.
      String numWorkers = String.valueOf(enclave.enclaveInfo.numWorkers);
      if (enclave.isMainOrFederated()) {
        numWorkers = "_lf_number_of_workers";
      }

      // Figure out the name of the trace file
      String traceFileName = "NULL";
      if (targetConfig.tracing != null) {
        if (targetConfig.tracing.traceFileName != null) {
          if (enclave.isMainOrFederated()) {
            traceFileName = "\"" + targetConfig.tracing.traceFileName + ".lft\"";
          } else {
            traceFileName =
                "\"" + targetConfig.tracing.traceFileName + enclave.getName() + ".lft\"";
          }
        } else {
          if (enclave.isMainOrFederated()) {
            traceFileName = "\"" + lfModuleName + ".lft\"";
          } else {
            traceFileName = "\"" + lfModuleName + enclave.getName() + ".lft\"";
          }
        }
      }

      code.pr(
          "environment_init(&"
              + CUtil.getEnvironmentStruct(enclave)
              + ","
              + "\""
              + enclave.getName()
              + "\""
              + ","
              + CUtil.getEnvironmentId(enclave)
              + ","
              + numWorkers
              + ","
              + enclave.enclaveInfo.numTimerTriggers
              + ","
              + enclave.enclaveInfo.numStartupReactions
              + ","
              + enclave.enclaveInfo.numShutdownReactions
              + ","
              + enclave.enclaveInfo.numResetReactions
              + ","
              + enclave.enclaveInfo.numIsPresentFields
              + ","
              + enclave.enclaveInfo.numModalReactors
              + ","
              + enclave.enclaveInfo.numModalResetStates
              + ","
              + traceFileName
              + ");");
    }
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  private String generateConnectionTopologyInfo() {
    CodeBuilder code = new CodeBuilder();

    for (ReactorInstance enclave : enclaves) {
      code.pr(generateConnectionArrays(enclave, enclaveGraph));
    }
    code.pr(generateConnectionGetFunctions());
    return code.toString();
  }

  private String generateConnectionArrays(ReactorInstance enclave, CEnclaveGraph connectionGraph) {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateDownstreamsArray(enclave, connectionGraph));
    code.pr(generateUpstreamsArray(enclave, connectionGraph));
    code.pr(generateUpstreamDelaysArray(enclave, connectionGraph));
    return code.toString();
  }

  private String generateDownstreamsArray(ReactorInstance enclave, CEnclaveGraph connectionGraph) {
    CodeBuilder code = new CodeBuilder();

    Set<ReactorInstance> downstreams =
        connectionGraph.getDirectDownstreams(enclave).stream()
            .map(EnclaveConnection::getTarget)
            .collect(Collectors.toSet());
    int numDownstream = downstreams.size();
    String encName = CUtil.getEnvironmentId(enclave);
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
      for (ReactorInstance downstream : downstreams) {
        String element = CUtil.getEnvironmentId(downstream);
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

  private String generateUpstreamsArray(ReactorInstance enclave, CEnclaveGraph connectionGraph) {
    CodeBuilder code = new CodeBuilder();
    List<ReactorInstance> upstreams =
        connectionGraph.getDirectUpstreams(enclave).stream()
            .map(EnclaveConnection::getSource)
            .toList();
    int numUpstream = upstreams.size();
    String encName = CUtil.getEnvironmentId(enclave);
    String numUpstreamVar = (encName + "_num_upstream").toUpperCase();
    String upstreamVar = encName + "_upstream";
    String upstreamDelayVar = encName + "_upstream_delay";

    code.prComment("Arrays representing the enclaves upstream of `" + upstreamVar + "`");
    code.pr("#define " + numUpstreamVar + " " + numUpstream);
    if (numUpstream == 0) {
      code.pr("// No upstreams, but 0-length arrays not allowed by MSVC (C2466)");
      code.pr("int " + upstreamVar + "[1] = {-1};");
    } else {
      code.pr("int " + upstreamVar + "[" + numUpstreamVar + "] = { ");
      code.indent();
      int idx = 0;
      for (ReactorInstance upstream : upstreams) {
        String element = CUtil.getEnvironmentId(upstream);
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

  private String generateUpstreamDelaysArray(
      ReactorInstance enclave, CEnclaveGraph connectionGraph) {
    CodeBuilder code = new CodeBuilder();
    List<TimeValue> upstreamDelays =
        connectionGraph.getDirectUpstreams(enclave).stream()
            .map(EnclaveConnection::getDelay)
            .toList();
    int numUpstream = upstreamDelays.size();
    String encName = CUtil.getEnvironmentId(enclave);
    String numUpstreamVar = (encName + "_num_upstream").toUpperCase();
    String upstreamDelayVar = encName + "_upstream_delay";
    if (numUpstream == 0) {
      code.pr("// No upstreams, but 0-length arrays not allowed by MSVC (C2466)");
      code.pr("interval_t " + upstreamDelayVar + "[1] = {FOREVER};");
    } else {
      code.pr("interval_t " + upstreamDelayVar + "[" + numUpstreamVar + "] = { ");
      code.indent();
      int idx = 0;
      for (TimeValue delay : upstreamDelays) {
        String element;
        if (delay.toNanoSeconds() == 0) {
          element = "NEVER";
        } else {
          element = String.valueOf(delay.toNanoSeconds());
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

  private String generateConnectionGetFunctions() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateGetUpstreamOf());
    code.pr(generateGetDownstreamOf());
    code.pr(generateGetUpstreamDelayOf());
    return code.toString();
  }

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
    for (ReactorInstance enclave : enclaves) {
      String enclaveId = CUtil.getEnvironmentId(enclave);
      String enclaveNumDownstream = (enclaveId + "_num_downstream").toUpperCase();
      code.pr("case " + enclaveId + ":");
      code.indent();
      code.pr("num_downstream = " + enclaveNumDownstream + ";");
      code.pr("downstream = &" + enclaveId + "_downstream[0];");
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
    for (ReactorInstance enclave : enclaves) {
      String enclaveId = CUtil.getEnvironmentId(enclave);
      String enclaveNumUpstream = (enclaveId + "_num_upstream").toUpperCase();
      code.pr("case " + enclaveId + ":");
      code.indent();
      code.pr("num_upstream = " + enclaveNumUpstream + ";");
      code.pr("upstream = &" + enclaveId + "_upstream[0];");
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
    for (ReactorInstance enclave : enclaves) {
      String enclaveId = CUtil.getEnvironmentId(enclave);
      String enclaveNumUpstream = (enclaveId + "_num_upstream").toUpperCase();
      code.pr("case " + enclaveId + ":");
      code.indent();
      code.pr("num_upstream = " + enclaveNumUpstream + ";");
      code.pr("delay = &" + enclaveId + "_upstream_delay[0];");
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
