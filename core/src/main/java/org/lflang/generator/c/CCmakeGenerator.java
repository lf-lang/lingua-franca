/*************
 * Copyright (c) 2019-2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.c;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.lflang.FileConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.CompilerFlagsProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.PlatformType.Platform;
import org.lflang.util.FileUtil;

/**
 * A helper class that generates a CMakefile that can be used to compile the generated C code.
 *
 * <p>Adapted from @see org.lflang.generator.CppCmakeGenerator.kt
 *
 * @author Soroush Bateni
 * @author Peter Donovan
 */
public class CCmakeGenerator {
  private static final String DEFAULT_INSTALL_CODE =
      """
        install(
            TARGETS ${LF_MAIN_TARGET}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    """;

  public static final String MIN_CMAKE_VERSION = "3.19";

  private final FileConfig fileConfig;
  private final List<String> additionalSources;
  private final SetUpMainTarget setUpMainTarget;
  private final String installCode;

  public CCmakeGenerator(FileConfig fileConfig, List<String> additionalSources) {
    this.fileConfig = fileConfig;
    this.additionalSources = additionalSources;
    this.setUpMainTarget = CCmakeGenerator::setUpMainTarget;
    this.installCode = DEFAULT_INSTALL_CODE;
  }

  public CCmakeGenerator(
      FileConfig fileConfig,
      List<String> additionalSources,
      SetUpMainTarget setUpMainTarget,
      String installCode) {
    this.fileConfig = fileConfig;
    this.additionalSources = additionalSources;
    this.setUpMainTarget = setUpMainTarget;
    this.installCode = installCode;
  }

  /**
   * Generate the contents of a CMakeLists.txt that builds the provided LF C 'sources'. Any error
   * will be reported in the 'errorReporter'.
   *
   * @param sources A list of .c files to build.
   * @param CppMode Indicate if the compilation should happen in C++ mode
   * @param hasMain Indicate if the .lf file has a main reactor or not. If not, a library target
   *     will be created instead of an executable.
   * @param cMakeExtras CMake-specific code that should be appended to the CMakeLists.txt.
   * @param context The context of the code generator.
   * @return The content of the CMakeLists.txt.
   */
  CodeBuilder generateCMakeCode(
      List<String> sources,
      boolean CppMode,
      boolean hasMain,
      String cMakeExtras,
      LFGeneratorContext context) {

    CodeBuilder cMakeCode = new CodeBuilder();
    var executableName = context.getFileConfig().name;
    var targetConfig = context.getTargetConfig();
    var messageReporter = context.getErrorReporter();

    List<String> additionalSources = new ArrayList<>();
    for (String file : targetConfig.compileAdditionalSources) {
      var relativePath =
          fileConfig
              .getSrcGenPath()
              .relativize(fileConfig.getSrcGenPath().resolve(Paths.get(file)));
      additionalSources.add(FileUtil.toUnixString(relativePath));
    }
    // Parse board option of the platform target property
    // Specified as a series of colon spaced options
    // Board syntax
    //  rp2040 <board_name> : <stdio_opt>
    //  arduino
    String[] boardProperties = {};
    var platformOptions = targetConfig.getOrDefault(PlatformProperty.INSTANCE);
    if (platformOptions.board() != null) {
      boardProperties = platformOptions.board().trim().split(":");
      // Ignore whitespace
      for (int i = 0; i < boardProperties.length; i++) {
        boardProperties[i] = boardProperties[i].trim();
      }
    }

    additionalSources.addAll(this.additionalSources);
    cMakeCode.newLine();

    cMakeCode.pr("cmake_minimum_required(VERSION " + MIN_CMAKE_VERSION + ")");

    // Setup the project header for different platforms
    switch (platformOptions.platform()) {
      case ZEPHYR:
        cMakeCode.pr("# Include default lf conf-file.");
        cMakeCode.pr("set(CONF_FILE prj_lf.conf)");
        cMakeCode.pr("# Include user-provided conf-file, if it exists");
        cMakeCode.pr("if(EXISTS prj.conf)");
        cMakeCode.pr("  set(OVERLAY_CONFIG prj.conf)");
        cMakeCode.pr("endif()");
        if (platformOptions.board() != null) {
          cMakeCode.pr("# Selecting board specified in target property");
          cMakeCode.pr("set(BOARD " + platformOptions.board() + ")");
        } else {
          cMakeCode.pr("# Selecting default board");
          cMakeCode.pr("set(BOARD qemu_cortex_m3)");
        }
        cMakeCode.pr("# We recommend Zephyr v3.4.0 but we are compatible with older versions also");
        cMakeCode.pr("find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE} 3.4.0)");
        cMakeCode.newLine();
        cMakeCode.pr("project(" + executableName + " LANGUAGES C)");
        cMakeCode.newLine();
        break;
      case RP2040:
        // Attempt to set PICO_SDK_PATH if it is not already set.
        if (System.getenv("PICO_SDK_PATH") == null) {
          Path picoSDKPath = fileConfig.srcPkgPath.resolve("pico-sdk");
          if (Files.isDirectory(picoSDKPath)) {
            messageReporter
                .nowhere()
                .info(
                    "pico-sdk library found at "
                        + picoSDKPath.toString()
                        + ". You can override this by setting PICO_SDK_PATH.");
            cMakeCode.pr("# Define the root of the pico-sdk library.");
            cMakeCode.pr("set(PICO_SDK_PATH " + picoSDKPath + ")");
          } else {
            messageReporter
                .nowhere()
                .warning(
                    "No PICO_SDK_PATH environment variable and no pico-sdk directory "
                        + "at the package root directory. Pico SDK will not be found.");
          }
        }
        cMakeCode.pr("include(pico_sdk_import.cmake)");
        cMakeCode.pr("project(" + executableName + " LANGUAGES C CXX ASM)");
        cMakeCode.newLine();
        // board type for rp2040 based boards
        if (platformOptions.board() != null) {
          if (boardProperties.length < 1 || boardProperties[0].equals("")) {
            cMakeCode.pr("set(PICO_BOARD pico)");
          } else {
            cMakeCode.pr("set(PICO_BOARD \"" + boardProperties[0] + "\")");
          }
        }
        // remove warnings for rp2040 only to make debug easier
        cMakeCode.pr("set(CMAKE_C_FLAGS \"${CMAKE_C_FLAGS} -w\")");
        break;
      default:
        cMakeCode.pr("project(" + executableName + " LANGUAGES C)");
        cMakeCode.newLine();
    }

    // The Test build type is the Debug type plus coverage generation
    cMakeCode.pr("if(CMAKE_BUILD_TYPE STREQUAL \"Test\")");
    cMakeCode.pr("  set(CMAKE_BUILD_TYPE \"Debug\")");
    cMakeCode.pr("  if(CMAKE_C_COMPILER_ID STREQUAL \"GNU\")");
    cMakeCode.pr("    find_program(LCOV_BIN lcov)");
    cMakeCode.pr("    if(LCOV_BIN MATCHES \"lcov$\")");
    cMakeCode.pr(
        "      set(CMAKE_C_FLAGS \"${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage\")");
    cMakeCode.pr("    else()");
    cMakeCode.pr(
        "      message(\"Not producing code coverage information since lcov was not found\")");
    cMakeCode.pr("    endif()");
    cMakeCode.pr("  else()");
    cMakeCode.pr(
        "    message(\"Not producing code coverage information since the selected compiler is no"
            + " gcc\")");
    cMakeCode.pr("  endif()");
    cMakeCode.pr("endif()");

    cMakeCode.pr("# Require C11");
    cMakeCode.pr("set(CMAKE_C_STANDARD 11)");
    cMakeCode.pr("set(CMAKE_C_STANDARD_REQUIRED ON)");
    cMakeCode.newLine();

    cMakeCode.pr("# Require C++17");
    cMakeCode.pr("set(CMAKE_CXX_STANDARD 17)");
    cMakeCode.pr("set(CMAKE_CXX_STANDARD_REQUIRED ON)");
    cMakeCode.newLine();

    // Set the build type
    cMakeCode.pr("set(DEFAULT_BUILD_TYPE " + targetConfig.get(BuildTypeProperty.INSTANCE) + ")\n");
    cMakeCode.pr("if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)\n");
    cMakeCode.pr(
        "    set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE} CACHE STRING \"Choose the type of build.\""
            + " FORCE)\n");
    cMakeCode.pr("endif()\n");
    cMakeCode.newLine();

    cMakeCode.pr("# Do not print install messages\n");
    cMakeCode.pr("set(CMAKE_INSTALL_MESSAGE NEVER)\n");

    cMakeCode.pr("# Colorize compilation output\n");
    cMakeCode.pr("set(CMAKE_COLOR_DIAGNOSTICS ON)\n");
    cMakeCode.newLine();

    if (CppMode) {
      // Suppress warnings about const char*.
      cMakeCode.pr("set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -Wno-write-strings\")");
      cMakeCode.newLine();
    }

    if (platformOptions.platform() != Platform.AUTO) {
      cMakeCode.pr("set(CMAKE_SYSTEM_NAME " + platformOptions.platform().getcMakeName() + ")");
    }
    cMakeCode.newLine();
    cMakeCode.pr("# Set default values for build parameters\n");
    targetConfig
        .get(CompileDefinitionsProperty.INSTANCE)
        .forEach(
            (key, value) -> {
              cMakeCode.pr("if (NOT DEFINED " + key + ")\n");
              cMakeCode.indent();
              var v = "TRUE";
              if (value != null && !value.isEmpty()) {
                v = value;
              }
              cMakeCode.pr("set(" + key + " " + v + ")\n");
              cMakeCode.unindent();
              cMakeCode.pr("endif()\n");
            });

    // Setup main target for different platforms
    switch (platformOptions.platform()) {
      case ZEPHYR:
        cMakeCode.pr(
            setUpMainTargetZephyr(
                hasMain,
                executableName,
                Stream.concat(additionalSources.stream(), sources.stream())));
        break;
      case RP2040:
        cMakeCode.pr(
            setUpMainTargetRp2040(
                hasMain,
                executableName,
                Stream.concat(additionalSources.stream(), sources.stream())));
        break;
      default:
        cMakeCode.pr(
            setUpMainTarget.getCmakeCode(
                hasMain,
                executableName,
                Stream.concat(additionalSources.stream(), sources.stream())));
    }

    // Ensure that the math library is linked
    cMakeCode.pr("find_library(MATH_LIBRARY m)");
    cMakeCode.pr("if(MATH_LIBRARY)");
    cMakeCode.pr("  target_link_libraries(${LF_MAIN_TARGET} PUBLIC ${MATH_LIBRARY})");
    cMakeCode.pr("endif()");

    cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE reactor-c)");

    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC .)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/api)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/platform)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/modal_models)");
    cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/utils)");

    // post target definition board configurations
    switch (platformOptions.platform()) {
      case RP2040:
        // set stdio output
        boolean usb = true;
        boolean uart = true;
        if (platformOptions.board() != null && boardProperties.length > 1) {
          uart = !boardProperties[1].equals("usb");
          usb = !boardProperties[1].equals("uart");
        }
        cMakeCode.pr("pico_enable_stdio_usb(${LF_MAIN_TARGET} " + (usb ? 1 : 0) + ")");
        cMakeCode.pr("pico_enable_stdio_uart(${LF_MAIN_TARGET} " + (uart ? 1 : 0) + ")");
        break;
    }

    if (targetConfig.get(AuthProperty.INSTANCE)) {
      // If security is requested, add the auth option.
      var osName = System.getProperty("os.name").toLowerCase();
      // if platform target was set, use given platform instead
      if (platformOptions.platform() != Platform.AUTO) {
        osName = platformOptions.platform().toString();
      }
      if (osName.contains("mac")) {
        cMakeCode.pr("set(OPENSSL_ROOT_DIR /usr/local/opt/openssl)");
      }
      cMakeCode.pr("# Find OpenSSL and link to it");
      cMakeCode.pr("find_package(OpenSSL REQUIRED)");
      cMakeCode.pr("target_link_libraries( ${LF_MAIN_TARGET} PRIVATE OpenSSL::SSL)");
      cMakeCode.newLine();
    }

    if (!targetConfig.get(SingleThreadedProperty.INSTANCE)
        && platformOptions.platform() != Platform.ZEPHYR
        && platformOptions.platform() != Platform.RP2040) {
      // If threaded computation is requested, add the threads option.
      cMakeCode.pr("# Find threads and link to it");
      cMakeCode.pr("find_package(Threads REQUIRED)");
      cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE Threads::Threads)");
      cMakeCode.newLine();
    }

    // Add additional flags so runtime can distinguish between multi-threaded and single-threaded
    // mode
    if (!targetConfig.get(SingleThreadedProperty.INSTANCE)) {
      cMakeCode.pr("# Set the number of workers to enable threading/tracing");
      cMakeCode.pr(
          "target_compile_definitions(${LF_MAIN_TARGET} PUBLIC NUMBER_OF_WORKERS="
              + targetConfig.get(WorkersProperty.INSTANCE)
              + ")");
      cMakeCode.newLine();
    } else {
      cMakeCode.pr("# Set flag to indicate a single-threaded runtime");
      cMakeCode.pr("target_compile_definitions( ${LF_MAIN_TARGET} PUBLIC LF_SINGLE_THREADED=1)");
    }
    cMakeCode.newLine();

    if (CppMode) cMakeCode.pr("enable_language(CXX)");

    if (targetConfig.isSet(CompilerProperty.INSTANCE)) {
      if (CppMode) {
        // Set the CXX compiler to what the user has requested.
        cMakeCode.pr("set(CMAKE_CXX_COMPILER " + targetConfig.get(CompilerProperty.INSTANCE) + ")");
      } else {
        cMakeCode.pr("set(CMAKE_C_COMPILER " + targetConfig.get(CompilerProperty.INSTANCE) + ")");
      }
      cMakeCode.newLine();
    }

    // link protobuf
    if (!targetConfig.get(ProtobufsProperty.INSTANCE).isEmpty()) {
      cMakeCode.pr("include(FindPackageHandleStandardArgs)");
      cMakeCode.pr("FIND_PATH( PROTOBUF_INCLUDE_DIR protobuf-c/protobuf-c.h)");
      cMakeCode.pr(
          """
                         find_library(PROTOBUF_LIBRARY\s
                         NAMES libprotobuf-c.a libprotobuf-c.so libprotobuf-c.dylib protobuf-c.lib protobuf-c.dll
                         )""");
      cMakeCode.pr(
          "find_package_handle_standard_args(libprotobuf-c DEFAULT_MSG PROTOBUF_INCLUDE_DIR"
              + " PROTOBUF_LIBRARY)");
      cMakeCode.pr(
          "target_include_directories( ${LF_MAIN_TARGET} PUBLIC ${PROTOBUF_INCLUDE_DIR} )");
      cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE ${PROTOBUF_LIBRARY})");
      cMakeCode.newLine();
    }

    // Set the compiler flags
    // We can detect a few common libraries and use the proper target_link_libraries to find them
    for (String compilerFlag : targetConfig.get(CompilerFlagsProperty.INSTANCE)) {
      messageReporter
          .nowhere()
          .warning(
              "Using the flags target property with cmake is dangerous.\n"
                  + " Use cmake-include instead.");
      cMakeCode.pr("add_compile_options( " + compilerFlag + " )");
      cMakeCode.pr("add_link_options( " + compilerFlag + ")");
    }
    cMakeCode.newLine();

    // Add the install option
    cMakeCode.pr(installCode);
    cMakeCode.newLine();

    // Add the include file
    for (String includeFile : targetConfig.getOrDefault(CmakeIncludeProperty.INSTANCE)) {
      cMakeCode.pr("include(\"" + Path.of(includeFile).getFileName() + "\")");
    }
    cMakeCode.newLine();

    // Add definition of directory where the main CMakeLists.txt file resides because this is where
    // any files specified by the `file` target directive will be put.
    cMakeCode.pr(
        "# Define directory in which files from the 'files' target directive will be put.");
    cMakeCode.pr(
        "target_compile_definitions(${LF_MAIN_TARGET} PUBLIC"
            + " LF_TARGET_FILES_DIRECTORY=\"${CMAKE_CURRENT_LIST_DIR}\")");

    cMakeCode.pr(cMakeExtras);
    cMakeCode.newLine();

    return cMakeCode;
  }

  /** Provide a strategy for configuring the main target of the CMake build. */
  public interface SetUpMainTarget {
    // Implementation note: This indirection is necessary because the Python
    // target produces a shared object file, not an executable.
    String getCmakeCode(boolean hasMain, String executableName, Stream<String> cSources);
  }

  /** Generate the C-target-specific code for configuring the executable produced by the build. */
  private static String setUpMainTarget(
      boolean hasMain, String executableName, Stream<String> cSources) {
    var code = new CodeBuilder();
    code.pr("add_subdirectory(core)");
    code.newLine();
    code.pr("set(LF_MAIN_TARGET " + executableName + ")");
    code.newLine();

    if (hasMain) {
      code.pr("# Declare a new executable target and list all its sources");
      code.pr("add_executable(");
    } else {
      code.pr("# Declare a new library target and list all its sources");
      code.pr("add_library(");
    }
    code.indent();
    code.pr("${LF_MAIN_TARGET}");
    cSources.forEach(code::pr);
    code.unindent();
    code.pr(")");
    code.newLine();
    return code.toString();
  }

  private static String setUpMainTargetZephyr(
      boolean hasMain, String executableName, Stream<String> cSources) {
    var code = new CodeBuilder();
    code.pr("add_subdirectory(core)");
    code.newLine();

    if (hasMain) {
      code.pr("# Declare a new executable target and list all its sources");
      code.pr("set(LF_MAIN_TARGET app)");
      code.pr("target_sources(");
    } else {
      code.pr("# Declare a new library target and list all its sources");
      code.pr("set(LF_MAIN_TARGET" + executableName + ")");
      code.pr("add_library(");
    }
    code.indent();
    code.pr("${LF_MAIN_TARGET}");

    if (hasMain) {
      code.pr("PRIVATE");
    }

    cSources.forEach(code::pr);
    code.unindent();
    code.pr(")");
    code.newLine();
    return code.toString();
  }

  private static String setUpMainTargetRp2040(
      boolean hasMain, String executableName, Stream<String> cSources) {
    var code = new CodeBuilder();
    // initialize sdk
    code.pr("pico_sdk_init()");
    code.newLine();
    code.pr("add_subdirectory(core)");
    code.pr("target_link_libraries(reactor-c PUBLIC pico_stdlib)");
    code.pr("target_link_libraries(reactor-c PUBLIC pico_multicore)");
    code.pr("target_link_libraries(reactor-c PUBLIC pico_sync)");
    code.newLine();
    code.pr("set(LF_MAIN_TARGET " + executableName + ")");

    if (hasMain) {
      code.pr("# Declare a new executable target and list all its sources");
      code.pr("add_executable(");
    } else {
      code.pr("# Declare a new library target and list all its sources");
      code.pr("add_library(");
    }
    code.indent();
    code.pr("${LF_MAIN_TARGET}");
    cSources.forEach(code::pr);
    code.unindent();
    code.pr(")");
    code.newLine();
    code.pr("pico_add_extra_outputs(${LF_MAIN_TARGET})");
    code.newLine();
    return code.toString();
  }
}
