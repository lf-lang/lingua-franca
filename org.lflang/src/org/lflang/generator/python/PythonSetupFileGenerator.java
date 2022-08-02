package org.lflang.generator.python;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.c.CCompiler;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;
import org.lflang.util.StringUtil;

class PythonSetupFileGenerator {

    /**
     * Generate the setup.py required to compile and install the module.
     * Currently, the package name is based on filename which does not support sharing the setup.py for multiple .lf files.
     * TODO: use an alternative package name (possibly based on folder name)
     *
     * If the LF program itself is threaded or if tracing is enabled, NUMBER_OF_WORKERS is added as a macro
     * so that platform-specific C files will contain the appropriate functions.
     */
    static String generatePythonSetupFile(
        String lfModuleName,
        String pyModuleName,
        TargetConfig targetConfig,
        List<String> pythonRequiredModules,
        FileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        List<String> sources = new ArrayList<>(targetConfig.compileAdditionalSources);
        sources.add(lfModuleName + ".c");
        sources = sources.stream()
                .map(Paths::get)
                .map(FileUtil::toUnixString)
                .map(StringUtil::addDoubleQuotes)
                .collect(Collectors.toList());

        List<String> macros = new ArrayList<>();
        macros.add(generateMacroEntry("MODULE_NAME", pyModuleName));

        for (var entry : targetConfig.compileDefinitions.entrySet()) {
            macros.add(generateMacroEntry(entry.getKey(), entry.getValue()));
        }

        if (targetConfig.threading || targetConfig.tracing != null) {
            macros.add(generateMacroEntry("NUMBER_OF_WORKERS", String.valueOf(targetConfig.workers)));
        }

        List<String> installRequires = new ArrayList<>(pythonRequiredModules);
        installRequires.add("LinguaFrancaBase");
        installRequires.replaceAll(StringUtil::addDoubleQuotes);

        return String.join("\n",
            """
            import sys
            assert (sys.version_info.major >= 3 and sys.version_info.minor >= 6), \
                "The Python target requires Python version >= 3.6."

            from skbuild import setup
            from setuptools import Extension
            """,
            "linguafranca"+lfModuleName+"module = Extension("+StringUtil.addDoubleQuotes(pyModuleName)+",",
            "                                            sources = ["+String.join(", ", sources)+"],",
            "                                            define_macros=["+String.join(", ", macros)+"])",
            "",
            "setup(name="+StringUtil.addDoubleQuotes(pyModuleName)+", version=\"1.0\",",
            "        ext_modules = [linguafranca"+lfModuleName+"module],",
            "        install_requires=["+String.join(", ", installRequires)+"])"
        );
    }

    private static String argListOfCommand(LFCommand command) {
        return command.command().stream()
            .map(StringUtil::addDoubleQuotes)
            .skip(1)
            .collect(Collectors.joining(", "));
    }

    /**
     * Generate a (`key`, `val`) tuple pair for the `define_macros` field
     * of the Extension class constructor from setuptools.
     *
     * @param key The key of the macro entry
     * @param val The value of the macro entry
     * @return A (`key`, `val`) tuple pair as String
     */
    private static String generateMacroEntry(String key, String val) {
        return "(" + StringUtil.addDoubleQuotes(key) + ", " + StringUtil.addDoubleQuotes(val) + ")";
    }
}
