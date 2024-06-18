package org.lflang.generator.docker;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.cpp.CppPlatformGenerator;
import org.lflang.generator.cpp.CppStandaloneGenerator;
import org.lflang.target.property.BuildTypeProperty;

/**
 * Generates the docker file related code for the C++ target.
 *
 * @author Marten Lohstroh
 */
public class CppDockerGenerator extends DockerGenerator {

  private CppPlatformGenerator platformGenerator;

  /** Construct a new Docker generator. */
  public CppDockerGenerator(LFGeneratorContext context, CppPlatformGenerator platformGenerator) {
    super(context);
    this.platformGenerator = platformGenerator;
  }

  @Override
  protected String generateCopyForSources() {
    return "COPY src-gen src-gen";
  }

  public static final String DEFAULT_BASE_IMAGE = "alpine:latest";

  @Override
  public String defaultImage() {
    return DEFAULT_BASE_IMAGE;
  }

  @Override
  protected String generateRunForInstallingDeps() {
    if (builderBase().equals(defaultImage())) {
      return "RUN set -ex && apk add --no-cache g++ musl-dev cmake make && apk add --no-cache"
          + " --update --repository=https://dl-cdn.alpinelinux.org/alpine/v3.16/main/"
          + " libexecinfo-dev";
    } else {
      return "# (Skipping installation of build dependencies; custom base image.)";
    }
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("./bin/" + context.getFileConfig().name);
  }

  @Override
  protected List<String> defaultBuildCommands() {
    var mkdirCommand = List.of("mkdir", "-p", "build", "&&", "mkdir", "-p", "bin");
    var cmakeCommand = new ArrayList<String>();
    cmakeCommand.add("cmake");
    cmakeCommand.addAll(platformGenerator.getCmakeArgs());
    cmakeCommand.addAll(
        List.of(
            "-DCMAKE_INSTALL_BINDIR=bin",
            "-DCMAKE_INSTALL_PREFIX=.",
            "-DREACTOR_CPP_LINK_EXECINFO=ON",
            "-S",
            "src-gen",
            "-B",
            "build"));
    var makeCommand =
        List.of(
            "cmake",
            "--build",
            "build",
            "--target",
            context.getFileConfig().name,
            "--config",
            CppStandaloneGenerator.Companion.buildTypeToCmakeConfig(
                context.getTargetConfig().get(BuildTypeProperty.INSTANCE)));
    var installCommand =
        List.of(
            "cmake",
            "--build",
            "build",
            "--target",
            "install",
            "--config",
            CppStandaloneGenerator.Companion.buildTypeToCmakeConfig(
                context.getTargetConfig().get(BuildTypeProperty.INSTANCE)));
    return Stream.of(mkdirCommand, cmakeCommand, makeCommand, installCommand)
        .map(CppDockerGenerator::argListToCommand)
        .toList();
  }

  static String argListToCommand(List<String> args) {
    return args.stream().map(it -> "\"" + it + "\"").collect(Collectors.joining(" "));
  }
}
