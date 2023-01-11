/** Stand-alone version of the Lingua Franca formatter (lff). */
package org.lflang.cli;

import java.io.IOException;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.FileVisitResult;
import java.nio.file.FileVisitor;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.eclipse.emf.ecore.resource.Resource;

import org.lflang.ast.FormattingUtils;
import org.lflang.util.FileUtil;

/**
 * Standalone version of the Lingua Franca formatter (lff). Based on lfc.
 *
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Billy Bao
 */
public class Lff extends CliBase {

  public Lff() {
    super("lff");
  }

  /**
   * Supported CLI options.
   *
   * <p>Stores an Apache Commons CLI Option for each entry, sets it to be required if so
   * specified,
   * and stores whether to pass the option to the code generator.
   *
   * @author Marten Lohstroh
   * @author Billy Bao
   */
  enum CLIOption {
    DRY_RUN(
        "d",
        "dry-run",
        false,
        false,
        "Send the formatted file contents to stdout without writing to the file system."),
    LINE_WRAP(
        "w",
        "wrap",
        true,
        false,
        "Causes the formatter to line wrap the files to a specified length."),
    NO_RECURSE(
        null,
        "no-recurse",
        false,
        false,
        "Do not format files in subdirectories of the specified paths."),
    OUTPUT_PATH(
        "o",
        "output-path",
        true,
        false,
        "If specified, outputs all formatted files into this directory instead of"
            + " overwriting the original files. Subdirectory structure will be preserved."),
    VERBOSE("v", "verbose", false, false, "Print more details on files affected."),
    ;

    /** The corresponding Apache CLI Option object. */
    public final Option option;

    /**
     * Construct a new CLIOption.
     *
     * @param opt The short option name. E.g.: "f" denotes a flag "-f".
     * @param longOpt The long option name. E.g.: "foo" denotes a flag "--foo".
     * @param hasArg Whether or not this option has an argument. E.g.: "--foo bar" where "bar" is
     *     the argument value.
     * @param isReq Whether or not this option is required. If it is required but not specified a
     *     menu is shown.
     * @param description The description used in the menu.
     */
    CLIOption(String opt, String longOpt, boolean hasArg, boolean isReq, String description) {
      this.option = new Option(opt, longOpt, hasArg, description);
      option.setRequired(isReq);
    }

    /**
     * Create an Apache Commons CLI Options object and add all the options.
     *
     * @return Options object that includes all the options in this enum.
     */
    public static Options getOptions() {
      Options opts = new Options();
      Arrays.asList(CLIOption.values()).forEach(o -> opts.addOption(o.option));
      return opts;
    }
  }


  /**
   * Entry point of the formatter.
   * Caution: this will invoke System.exit.
   *
   * @param args CLI arguments
   */
  public static void main(final String[] args) {
    main(Io.SYSTEM, args);
  }

  /**
   * Programmatic entry point, with a custom IO.
   *
   * @param io IO streams.
   * @param args Command-line arguments.
   */
  public static void main(Io io, final String... args) {
    cliMain("lff", Lff.class, io, args);
  }


  @Override
  protected Options getOptions() {
    return CLIOption.getOptions();
  }

  /**
   * Check all given input paths and the output path, then invokes the formatter on all files
   * given.
   */
  @Override
  protected void runTool(CommandLine cmd, List<Path> files) {
    String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
    final Path outputRoot;
    if (cmd.hasOption(pathOption)) {
      outputRoot = io.getWd().resolve(cmd.getOptionValue(pathOption)).toAbsolutePath().normalize();
      if (!Files.exists(outputRoot)) {
        reporter.printFatalErrorAndExit("Output location '" + outputRoot + "' does not exist.");
      }
      if (!Files.isDirectory(outputRoot)) {
        reporter.printFatalErrorAndExit("Output location '" + outputRoot + "' is not a directory.");
      }
    } else {
      outputRoot = null;
    }

    for (Path path : files) {
      if (!Files.exists(path)) {
        reporter.printFatalErrorAndExit(path + ": No such file or directory");
      }
    }

    final int lineLength =
        !cmd.hasOption(CLIOption.LINE_WRAP.option.getOpt())
            ? FormattingUtils.DEFAULT_LINE_LENGTH
            : Integer.parseInt(cmd.getOptionValue(CLIOption.LINE_WRAP.option.getOpt()));

    final boolean dryRun = cmd.hasOption(CLIOption.DRY_RUN.option.getOpt());

    boolean verbose = cmd.hasOption(CLIOption.VERBOSE.option.getOpt());
    for (Path relPath : files) {
      if (verbose) {
        reporter.printInfo("Formatting " + io.getWd().relativize(relPath) + ":");
      }
      Path path = toAbsolutePath(relPath);
      if (Files.isDirectory(path) && !cmd.hasOption(CLIOption.NO_RECURSE.option.getLongOpt())) {
        // this is a directory, walk its contents.
        try {
          Files.walkFileTree(path, new SimpleFileVisitor<>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) {
              formatSingleFile(file, path, outputRoot, lineLength, dryRun, verbose);
              return FileVisitResult.CONTINUE;
            }
          });
        } catch (IOException e) {
          reporter.printError("IO error: " + e);
        }

      } else {
        // Simple file
        formatSingleFile(path, path.getParent(), outputRoot, lineLength, dryRun, verbose);
      }
    }

    exitIfCollectedErrors();
    if (!dryRun || verbose) {
      reporter.printInfo("Done formatting.");
    }
  }

  /** Load and validate a single file, then format it and output to the given outputPath. */
  private void formatSingleFile(Path file, Path inputRoot, Path outputRoot, int lineLength, boolean dryRun, boolean verbose) {
    file = file.normalize();
    Path outputPath = outputRoot == null
        ? file // format in place
        : outputRoot.resolve(inputRoot.relativize(file)).normalize();
    final Resource resource = getResource(file);
    if (resource == null) {
      if (verbose) {
        reporter.printInfo("Skipped " + file + ": not an LF file");
      }
      return; // not an LF file, nothing to do here
    }
    validateResource(resource);

    // todo don't abort whole run if one file has errors
    exitIfCollectedErrors();
    final String formattedFileContents =
        FormattingUtils.render(resource.getContents().get(0), lineLength);

    if (dryRun) {
      io.getOut().print(formattedFileContents);
    } else {
      try {
        FileUtil.writeToFile(formattedFileContents, outputPath, true);
      } catch (IOException e) {
        if (e instanceof FileAlreadyExistsException) {
          // only happens if a subdirectory is named with ".lf" at the end
          reporter.printFatalErrorAndExit(
              "Error writing to "
                  + outputPath
                  + ": file already exists. Make sure that no file or directory"
                  + " within provided input paths have the same relative paths.");
        }
      }
    }

    exitIfCollectedErrors();
    issueCollector.getAllIssues().forEach(reporter::printIssue);
    if (verbose) {
      String msg = "Formatted " + io.getWd().relativize(file);
      if (file != outputPath) msg += " -> " + io.getWd().relativize(outputPath);
      reporter.printInfo(msg);
    }
  }

}
