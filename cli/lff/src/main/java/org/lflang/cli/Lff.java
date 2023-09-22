package org.lflang.cli;

import java.io.IOException;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.List;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.ast.FormattingUtil;
import org.lflang.ast.IsEqual;
import org.lflang.ast.LfParsingHelper;
import org.lflang.lf.Model;
import org.lflang.util.FileUtil;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;

/**
 * Standalone version of the Lingua Franca formatter (lff). Based on lfc.
 *
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Billy Bao
 * @author Atharva Patil
 */
@Command(
    name = "lff",
    // Enable usageHelp (--help) and versionHelp (--version) options.
    mixinStandardHelpOptions = true,
    versionProvider = VersionProvider.class)
public class Lff extends CliBase {

  /** Supported CLI options for Lff. */
  @Option(
      names = {"-c", "--check"},
      description =
          "Check mode. Exit with an error code if any files would change when applying formatting.")
  private boolean check = false;

  @Option(
      names = {"-d", "--dry-run"},
      description =
          "Send the formatted file contents to stdout" + " without writing to the file system.")
  private boolean dryRun = false;

  @Option(
      names = {"-w", "--wrap"},
      description = "Causes the formatter to line wrap the files to a" + " specified length.",
      defaultValue = "" + FormattingUtil.DEFAULT_LINE_LENGTH,
      fallbackValue = "" + FormattingUtil.DEFAULT_LINE_LENGTH)
  private int lineLength;

  @Option(
      names = "--no-recurse",
      description = "Do not format files in subdirectories of the" + " specified paths.")
  private boolean noRecurse = false;

  @Option(
      names = {"-v", "--verbose"},
      description = "Print more details on files affected.")
  private boolean verbose = false;

  @Option(
      names = {"--ignore-errors"},
      description = "Ignore validation errors in files and format them anyway.")
  private boolean ignoreErrors = false;

  /**
   * Main function of the formatter. Caution: this will invoke System.exit.
   *
   * @param args CLI arguments
   */
  public static void main(String[] args) {
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

  /** Validates all paths and invokes the formatter on the input paths. */
  @Override
  public void doRun() {
    if (check && dryRun) {
      reporter.printFatalErrorAndExit(
          "The options --check (-c) and --dry-run (-d) are mutually exclusive. Please use only one"
              + " at a time.");
    }

    List<Path> paths;
    do {
      paths = getInputPaths();
      final Path outputRoot = getOutputRoot();

      try {
        // Format all files defined by the list of paths.
        formatAllFiles(paths, outputRoot);

        exitIfCollectedErrors();
        if (!dryRun || verbose) {
          reporter.printInfo("Done formatting.");
        }
      } catch (RuntimeException e) {
        reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
      }
    } while (stdinMode() && !paths.isEmpty());

    // return an error code if any errors were reported
    reporter.exit();
  }

  /*
   * Invokes the formatter on all files defined by the list of paths.
   */
  private void formatAllFiles(List<Path> paths, Path outputRoot) {
    for (Path relativePath : paths) {
      if (verbose) {
        reporter.printInfo("Formatting " + io.getWd().relativize(relativePath) + ":");
      }

      Path path = toAbsolutePath(relativePath);
      if (Files.isDirectory(path) && !noRecurse) {
        // Walk the contents of this directory.
        try {
          Files.walkFileTree(
              path,
              new SimpleFileVisitor<>() {
                @Override
                public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) {
                  formatSingleFile(file, path, outputRoot);
                  return FileVisitResult.CONTINUE;
                }
              });
        } catch (IOException e) {
          reporter.printFatalErrorAndExit("An unknown I/O exception occurred.", e);
        }
      } else {
        // Simple file.
        formatSingleFile(path, path.getParent(), outputRoot);
      }
    }
  }

  /*
   * Invokes the formatter on a single file defined by the given path.
   */
  private void formatSingleFile(Path path, Path inputRoot, Path outputRoot) {
    path = path.normalize();
    Path outputPath =
        outputRoot == null
            ? path // Format in place.
            : outputRoot.resolve(inputRoot.relativize(path)).normalize();

    Path relativePath = io.getWd().relativize(path);

    final Resource resource = getResource(path);
    // Skip file if not an LF file.
    if (resource == null) {
      if (verbose) {
        reporter.printInfo("Skipped " + relativePath + ": not an LF file");
      }
      return;
    }
    validateResource(resource);

    if (!ignoreErrors) {
      exitIfCollectedErrors();
    }

    final String formattedFileContents =
        FormattingUtil.render((Model) resource.getContents().get(0), lineLength);
    if (!new IsEqual(resource.getContents().get(0))
        .doSwitch(
            new LfParsingHelper()
                .parseSourceAsIfInDirectory(path.getParent(), formattedFileContents))) {
      reporter.printFatalErrorAndExit(
          "The formatter failed to produce output that is semantically equivalent to its input when"
              + " executed on the file "
              + path
              + ". Please file a bug report with Lingua Franca.");
    }

    try {
      if (check) {
        if (!FileUtil.isSame(formattedFileContents, outputPath)) {
          reporter.printError("Would reformat " + outputPath);
        }
      } else if (dryRun) {
        io.getOut().println(formattedFileContents);
        io.getOut().println("\0");
      } else {
        FileUtil.writeToFile(formattedFileContents, outputPath, true);
      }
    } catch (FileAlreadyExistsException e) {
      // Only happens if a subdirectory is named with
      // ".lf" at the end.
      reporter.printFatalErrorAndExit(
          "Error writing to "
              + outputPath
              + ": file already exists. Make sure that no file or"
              + " directory within provided input paths have the"
              + " same relative paths.");
    } catch (IOException e) {
      reporter.printFatalErrorAndExit(
          "An unknown I/O exception occurred while processing " + outputPath, e);
    }

    if (!ignoreErrors) {
      exitIfCollectedErrors();
    }
    // Only errors are printed. Warnings are not helpful for LFF
    // and since they don't prevent the file from being formatted,
    // the position of the issue may be wrong in the formatted file.
    // issueCollector.getAllIssues().forEach(reporter::printIssue);
    if (verbose) {
      String msg = "Formatted " + relativePath;
      if (path != outputPath) {
        msg += " -> " + io.getWd().relativize(outputPath);
      }
      reporter.printInfo(msg);
    }
  }
}
