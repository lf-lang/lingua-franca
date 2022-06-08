package org.lflang.generator.python;

import java.io.File;
import org.lflang.FileConfig;

public class PythonInfoGenerator {
    /**
     * Print information about necessary steps to install the supporting
     * Python C extension for the generated program.
     *
     * @note Only needed if no-compile is set to true
     */
    public static String generateSetupInfo(FileConfig fileConfig) {
        return String.join("\n",
            "",
            "#####################################",
            "To compile and install the generated code, do:",
            "    ",
            "    cd "+fileConfig.getSrcGenPath()+File.separator,
            "    python3 -m pip install --force-reinstall .",
            ""
        );
    }

    /**
     * Print information on how to execute the generated program.
     */
    public static String generateRunInfo(FileConfig fileConfig, String lfModuleName) {
        return String.join("\n",
            "",
            "#####################################",
            "To run the generated program, use:",
            "    ",
            "    python3 "+fileConfig.getSrcGenPath()+File.separator+lfModuleName+".py",
            "",
            "#####################################",
            ""
        );
    }

    /**
     * Print information on how to execute the generated federation.
     */
    public static String generateFedRunInfo(FileConfig fileConfig) {
        return String.join("\n",
            "",
            "#####################################",
            "To run the generated program, run:",
            "    ",
            "    bash "+fileConfig.binPath+"/"+fileConfig.name,
            "",
            "#####################################",
            ""
        );
    }
}
