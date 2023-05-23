package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;

import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;

public class CEnvironmentFunctionGenerator {


    public String generateCode(ReactorInstance main) {
        this.enclaves = CUtil.getEnclaves(main);

        CodeBuilder code = new CodeBuilder();
        code.pr(generateEnvironmentInclude());
        code.pr(generateEnvironmentEnum());
        code.pr(generateEnvironmentArray());
        code.pr(generateCreateEnvironments());
        code.pr(generateGetEnvironments());
        return code.toString();
    }

    private List<ReactorInstance> enclaves = new ArrayList<>();

    private String generateEnvironmentInclude() {
        return "#include \"environment.h\"";
    }

    private String generateEnvironmentArray() {
        return String.join("\n",
            "// The global array of environments associated with each enclave",
            "environment_t envs[_num_enclaves];");
    }
    private String generateGetEnvironments() {
        return String.join("\n",
            "// Update the pointer argument to point to the beginning of the environment array",
            "// and return the size of that array",
            "int _lf_get_environments(environment_t ** return_envs) {",
            "   (*return_envs) = (environment_t *) envs;",
            "   return _num_enclaves;",
            "}"
            );
    }

    private String generateEnvironmentEnum() {
        CodeBuilder code = new CodeBuilder();
        code.pr("typedef enum {");
        code.indent();
        for (ReactorInstance enclave: enclaves) {
            code.pr(CUtil.getEnvironmentId(enclave) +",");
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
        for (ReactorInstance enclave: enclaves) {
            code.pr(
                "environment_init(&"+CUtil.getEnvironmentStruct(enclave) +
                    ","+CUtil.getEnvironmentId(enclave) +
                    ","+CUtil.numWorkersInEnclave(enclave) +
                    ","+CUtil.numTimerTriggersInEnclave(enclave) +
                    ","+CUtil.numStartupReactionsInEnclave(enclave) +
                    ","+CUtil.numShutdownReactionsInEnclave(enclave) +
                    ","+CUtil.numResetReactionsInEnclave(enclave)+
                    ","+CUtil.numIsPresentFieldsInEnclave(enclave)+
                    ");"
            );
        }
        code.unindent();
        code.pr("}");
        return code.toString();
    }
}
