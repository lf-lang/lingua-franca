package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.TargetConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.util.StringUtil;

public class CMainGenerator {
    private TargetConfig targetConfig;
    /** The command to run the generated code if specified in the target directive. */
    private List<String> runCommand;

    public CMainGenerator(TargetConfig targetConfig) {
        this.targetConfig = targetConfig;
        runCommand = new ArrayList<>();
        parseTargetParameters();
    }

    /**
     * Generate the code that is the entry point
     * of the program.
     *
     * Ideally, this code would belong to its own `main.c`
     * file, but it currently lives in the same file
     * as all the code generated for reactors.
     */
    public String generateCode() {
        CodeBuilder code = new CodeBuilder();
        code.pr(generateMainFunction());
        code.pr(generateSetDefaultCliOption());
        return code.toString();
    }

    /**
     * Generate the `main` function.
     */
    private String generateMainFunction() {
        return String.join("\n",
            "int main(int argc, const char* argv[]) {",
            "    return lf_reactor_c_main(argc, argv);",
            "}"
        );
    }

    /**
     * Generate code that is used to override the
     * command line options to the `main` function
     */
    private String generateSetDefaultCliOption() {
        // Generate function to set default command-line options.
        // A literal array needs to be given outside any function definition,
        // so start with that.
        return runCommand.size() > 0 ?
            String.join("\n",
                "const char* _lf_default_argv[] = { " +
                        StringUtil.addDoubleQuotes(
                            StringUtil.joinObjects(runCommand,
                                StringUtil.addDoubleQuotes(", ")))+" };",
                "void _lf_set_default_command_line_options() {",
                "        default_argc = "+runCommand.size()+";",
                "        default_argv = _lf_default_argv;",
                "}")
                : "void _lf_set_default_command_line_options() {}";
    }

    /**
     * Parse the target parameters and set flags to the runCommand
     * accordingly.
     */
    private void parseTargetParameters() {
        if (targetConfig.fastMode) {
            runCommand.add("-f");
            runCommand.add("true");
        }
        if (targetConfig.keepalive) {
            runCommand.add("-k");
            runCommand.add("true");
        }
        if (targetConfig.timeout != null) {
            runCommand.add("-o");
            runCommand.add(targetConfig.timeout.getMagnitude() + "");
            runCommand.add(targetConfig.timeout.unit.getCanonicalName());
        }
        // The runCommand has a first entry that is ignored but needed.
        if (runCommand.size() > 0) {
            runCommand.add(0, "dummy");
        }
    }
}
