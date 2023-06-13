package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;

// FIXME: This file should be renamed to CEnclaveGenerator or something.
/**
 * This class is in charge of code generating functions and global variables related to the
 * environments
 */
public class CEnvironmentFunctionGenerator {

  /**
   * @param main The top-level reactor instance of the program
   * @param targetConfig The target config of the program
   * @param lfModuleName The lfModuleName of the program
   */
  public CEnvironmentFunctionGenerator(
      ReactorInstance main, TargetConfig targetConfig, String lfModuleName) {
    this.enclaves = CUtil.getEnclaves(main);
    this.targetConfig = targetConfig;
    this.lfModuleName = lfModuleName;
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
  private TargetConfig targetConfig;
  private String lfModuleName;

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
      code.pr(generateConnectionArrays(enclave));
    }
    code.pr(generateConnectionGetFunctions());
    return code.toString();
  }

  private String generateConnectionArrays(ReactorInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateUpstreamsArrays(enclave));

    return code.toString();
  }

  private String generateUpstreamsArrays(ReactorInstance enclave) {
    CodeBuilder code = new CodeBuilder();
    int numUpstream = enclave.inputs.size();
    int numDownstream = enclave.outputs.size();
    String encName = CUtil.getEnvironmentId(enclave);
    String numUpstreamVar = encName + "_num_upstream";
    String upstreamVar = encName + "_upstream";
    String upstreamDelayVar = encName + "_upstream_delay";

    code.pr("const int " + numUpstreamVar + " = " + numUpstream + ";");
    code.pr("int " + upstreamVar + "[" + numUpstreamVar + "] = { ");
    code.indent();
    for (int i = 0; i < numUpstream; i++) {
      ReactorInstance upstream = enclave.inputs.get(i).eventualSources().get(0).parentReactor();
      String element = CUtil.getEnvironmentId(upstream);
      if (i < numUpstream - 1) {
        element += ",";
      }
      code.pr(element);
    }
    code.unindent();
    code.pr("};");

    code.pr("int " + upstreamDelayVar + "[" + numUpstreamVar + "] = { ");
    code.indent();
    for (int i = 0; i < numUpstream; i++) {
      // FIXME: This is too ugly. Also consider all other connection topologies. I think we should
      // factor out some EnclaveTopology stuff
      ReactorInstance connection =
          enclave
              .inputs
              .get(i)
              .getDependentPorts()
              .get(0)
              .destinations
              .get(0)
              .instance
              .parents()
              .get(0);
      long delay = connection.actions.get(0).getMinDelay().toNanoSeconds();
      if (delay == 0) {
        delay = TimeValue.MAX_VALUE.toNanoSeconds();
      }
      String element = String.valueOf(delay);
      if (i < numUpstream - 1) {
        element += ",";
      }
      code.pr(element);
    }
    code.unindent();
    code.pr("};");
    return code.toString();
  }

  private String generateConnectionGetFunctions() {
    CodeBuilder code = new CodeBuilder();
    code.pr("int _lf_get_upstream(int enclave_id, int ** result) {");
    code.indent();
    code.pr("int num_upstreams[_num_enclaves] = { ");
    code.indent();
    var first = true;
    for (int i = 0; i < enclaves.size(); i++) {
      String element = CUtil.getEnvironmentId(enclaves.get(i));
      if (i < enclaves.size() - 1) {
        element += ",";
      }
      code.pr(element);
    }
    code.unindent();
    code.pr("};");

    code.unindent();
    return code.toString();
  }
}
