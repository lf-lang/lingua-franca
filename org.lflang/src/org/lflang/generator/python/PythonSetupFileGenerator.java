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
//        List<String> sources = new ArrayList<>(targetConfig.compileAdditionalSources);
//        sources.add(lfModuleName + ".c");
//        sources = sources.stream()
//            .map(Paths::get)
//            .map(FileUtil::toUnixString)
//            .map(StringUtil::addDoubleQuotes)
//            .collect(Collectors.toList());

        List<String> macros = new ArrayList<>();
        macros.add(generateMacroEntry("MODULE_NAME", pyModuleName));
//
//        for (var entry : targetConfig.compileDefinitions.entrySet()) {
//            macros.add(generateMacroEntry(entry.getKey(), entry.getValue()));
//        }
//
//        if (targetConfig.threading || targetConfig.tracing != null) {
//            macros.add(generateMacroEntry("NUMBER_OF_WORKERS", String.valueOf(targetConfig.workers)));
//        }

        List<String> installRequires = new ArrayList<>(pythonRequiredModules);
        installRequires.add("LinguaFrancaBase");
        installRequires.replaceAll(StringUtil::addDoubleQuotes);

        var compilerRunner = new CCompiler(targetConfig, fileConfig, errorReporter, false);
        String cmakeArgs = argListOfCommand(compilerRunner.compileCmakeCommand());
        String cmakeBuildArgs = argListOfCommand(compilerRunner.buildCmakeCommand());

        return String.join("\n",
            """
            import sys
            import os
            import shutil
            import pathlib
            assert (sys.version_info.major >= 3 and sys.version_info.minor >= 6), \
                "The Python target requires Python version >= 3.6."

            from setuptools import setup, Extension
            from setuptools.command.build_ext import build_ext as original_build_ext
            from setuptools.command.install_lib import install_lib
            
            """,
            "CMAKE_ARGS = [" + cmakeArgs + "]",
            "CMAKE_BUILD_ARGS = [" + cmakeBuildArgs + "]",
            """
            class CMakeExtension(Extension):
                def __init__(self, name):
                    super().__init__(name, sources=[])

            class BuildExt(original_build_ext):

                def run(self):
                    for ext in self.extensions:
                        self.build_cmake(ext)
                    super().run()

                def build_cmake(self, ext):
                    cwd = pathlib.Path().absolute()
                    build_temp = pathlib.Path(self.build_temp)
                    build_temp.mkdir(parents=True, exist_ok=True)
                    extension_dir = pathlib.Path(self.get_ext_fullpath(ext.name))
                    extension_dir.mkdir(parents=True, exist_ok=True)
                    os.chdir(str(build_temp))
                    self.spawn(['cmake', str(cwd)] + CMAKE_ARGS)
                    self.spawn(['cmake'] + CMAKE_BUILD_ARGS)

            class InstallCMakeLibs(install_lib):
                ""\"
                Get the libraries from the parent distribution, use those as the outfiles
                        
                Skip building anything; everything is already built, forward libraries to
                the installation step
                ""\"

                def run(self):
                    ""\"
                    Copy libraries from the bin directory and place them as appropriate
                    ""\"
                    self.announce("Moving library files", level=3)
                    self.skip_build = True
                    bin_dir = \"""" + fileConfig.getSrcGenPath().resolve("build") + "\""
            + """
                    # Depending on the files that are generated from your cmake
                    # build chain, you may need to change the below code, such that
                    # your files are moved to the appropriate location when the installation
                    # is run
                    libs = [
                        os.path.join(dir, file)
                        for dir, subdirs, files in os.walk(bin_dir)
                        for file in files
                        if os.path.isfile(os.path.join(bin_dir, file))
                        and os.path.splitext(_lib)[1] in [".dll", ".so"]
                        and not (_lib.startswith("python") or _lib.startswith(PACKAGE_NAME))
                    ]
                        
                    for lib in libs:
                        shutil.move(lib, os.path.join(self.build_dir, os.path.basename(lib)))
                    self.distribution.data_files = [
                        os.path.join(self.install_dir, os.path.basename(lib))
                        for lib in libs
                    ]
                    self.distribution.run_command("install_data")
                    super().run()
            """,
//            "linguafranca"+lfModuleName+"module = Extension("+StringUtil.addDoubleQuotes(pyModuleName)+",",
//            "                                            sources = ["+String.join(", ", sources)+"],",
//            "                                            define_macros=["+String.join(", ", macros)+"])",
            "",
            "setup(name="+StringUtil.addDoubleQuotes(pyModuleName)+", version=\"1.0\",",
            "        ext_modules = [CMakeExtension(\"linguafranca"+lfModuleName+"module\")],",
            "        cmdclass={",
            "            \"build_ext\": BuildExt,",
            "            \"install_lib\": InstallCMakeLibs",
            "        }",
            ")"
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
