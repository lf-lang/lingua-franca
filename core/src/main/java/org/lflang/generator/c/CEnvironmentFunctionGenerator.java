package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;

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
}
