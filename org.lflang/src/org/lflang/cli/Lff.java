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
import java.util.stream.Collectors;
import java.util.stream.Stream;

import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;
import picocli.CommandLine.Parameters;
import org.eclipse.emf.ecore.resource.Resource;

import org.lflang.ast.FormattingUtils;
import org.lflang.LocalStrings;
import org.lflang.util.FileUtil;

@Command(
    name = "lff",
    // Automatically add usageHelp and versionHelp options.
    mixinStandardHelpOptions = true,
    version = "0.3.1-SNAPSHOT")
public class Lff extends CliBaseNew {
    /**
     * Supported CLI options for LFF.
     */
    @Option(
        names = {"-d", "--dry-run"},
        description = "Send the formatted file contents to stdout"
                        + " without writing to the file system.")
    private boolean dryRun = false;

    @Option(
        names = {"-w", "--wrap"},
        description = "Causes the formatter to line wrap the files to a"
                        + " specified length.",
        defaultValue = "" + FormattingUtils.DEFAULT_LINE_LENGTH,
        fallbackValue = "" + FormattingUtils.DEFAULT_LINE_LENGTH)
    private int lineLength;

    @Option(
        names = "--no-recurse",
        description = "Do not format files in subdirectories of the specified paths.")
    private boolean noRecurse = false;

    @Option(
        names = {"-o", "--output-path"},
        defaultValue = "",
        fallbackValue = "",
        description = "If specified, outputs all formatted files into this" 
                        + " directory instead of overwriting the original"
                        + " files. Subdirectory structure will be preserved.")
    private String outputPath;

    @Option(
        names = {"-v", "--verbose"},
        description = "Print more details on files affected.")
    private boolean verbose = false;

    // public Lff() {
    //     super("lff");
    // }

    /**
     * Programmatic entry point, with a custom IO.
     *
     * @param io IO streams.
     * @param args Command-line arguments.
     */
    public static void main(Io io, final String... args) {
        cliMain("lff", Lff.class, io, args);
    }

    public static void main(String[] args) {
        main(Io.SYSTEM, args);
    }

    /*
     * The first method in LFF that is invoked when the parent CliBase Runnable
     * class is instantiated, i.e. the first method to run after the arguments
     * are parsed.
     */
    @Override
    public void run() {
        // TODO: Directly load paths instead of converting strings to paths.
        try {
            List<Path> paths = files.stream().map(
                    io.getWd()::resolve).collect(Collectors.toList());
            runTool(paths);
        } catch (RuntimeException e) {
            reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
        }
        io.callSystemExit(0);
    }

    /**
     * Check all given input paths and the output path, then invokes the formatter on all files
     * given.
     */
    @Override
    protected void runTool(List<Path> files) {
        final Path outputRoot;
        if (!outputPath.isEmpty()){
            outputRoot = 
                io.getWd().resolve(outputPath).toAbsolutePath().normalize();
            if (!Files.exists(outputRoot)) {
                reporter.printFatalErrorAndExit(
                        "Output location '" + outputRoot + "' does not exist.");
            }
            if (!Files.isDirectory(outputRoot)) {
                reporter.printFatalErrorAndExit(
                        "Output location '"
                        + outputRoot + "' is not a directory.");
            }
        } else {
            outputRoot = null;
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(
                        path + ": No such file or directory");
            }
        }

        for (Path relativePath : files) {
            if (verbose) {
                reporter.printInfo("Formatting "
                        + io.getWd().relativize(relativePath) + ":");
            }
            Path path = toAbsolutePath(relativePath);
            if (Files.isDirectory(path) && !noRecurse) {
                // this is a directory, walk its contents.
                try {
                    Files.walkFileTree(path, new SimpleFileVisitor<>() {
                        @Override
                        public FileVisitResult visitFile(
                                Path file, BasicFileAttributes attrs) {
                            formatSingleFile(
                                file, path, outputRoot,
                                lineLength, dryRun, verbose);
                            return FileVisitResult.CONTINUE;
                        }
                    });
                } catch (IOException e) {
                    reporter.printError("IO error: " + e);
                }

            } else {
                // Simple file
                formatSingleFile(
                    path, path.getParent(), outputRoot,
                    lineLength, dryRun, verbose);
            }
        }

        exitIfCollectedErrors();
        if (!dryRun || verbose) {
            reporter.printInfo("Done formatting.");
        }
    }

    private void formatSingleFile(
            Path file, Path inputRoot, Path outputRoot,
            int lineLength, boolean dryRun, boolean verbose) {
        file = file.normalize();
        Path outputPath = outputRoot == null
            ? file // format in place
            : outputRoot.resolve(inputRoot.relativize(file)).normalize();

        final Resource resource = getResource(file);
        // Skip file if not an LF file.
        if (resource == null) {
            if (verbose) {
                reporter.printInfo("Skipped " + file + ": not an LF file");
            }
            return; 
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
                    // Only happens if a subdirectory is named with 
                    // ".lf" at the end.
                    reporter.printFatalErrorAndExit(
                        "Error writing to "
                        + outputPath
                        + ": file already exists. Make sure that no file or" 
                        + " directory within provided input paths have the"
                        + " same relative paths.");
                }
            }
        }

        exitIfCollectedErrors();
        issueCollector.getAllIssues().forEach(reporter::printIssue);
        if (verbose) {
            String msg = "Formatted " + io.getWd().relativize(file);
            if (file != outputPath) msg += 
                " -> " + io.getWd().relativize(outputPath);
            reporter.printInfo(msg);
        }
    }
    
}
