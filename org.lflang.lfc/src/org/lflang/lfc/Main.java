/**
 * Stand-alone version of the Lingua Franca compiler (lfc).
 */
package org.lflang.lfc;

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Properties;
import java.util.stream.Collectors;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;
import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.generator.StandaloneContext;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

/**
 * Standalone version of the Lingua Franca compiler (lfc).
 *
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
public class Main {

    /**
     * The location of the class file of this class inside of the jar.
     */
    private static String MAIN_PATH_IN_JAR = String.join("/",
                                                         new String[] {"!", "org", "lflang", "lfc", "Main.class"});

    
    /**
     * Object for interpreting command line arguments.
     */
    protected CommandLine cmd;

    /**
     * Path to the jar.
     */
    protected Path jarPath;

    /**
     * Path to the root of the org.lflang source tree.
     */
    protected Path srcPath;

    /**
     * Path to the project root.
     */
    protected Path rootPath;

    /**
     * Injected resource provider.
     */
    @Inject
    private Provider<ResourceSet> resourceSetProvider;
    
    /**
     * Injected resource validator.
     */
    @Inject
    private IResourceValidator validator;
    
    /**
     * Injected code generator.
     */
    @Inject
    private GeneratorDelegate generator;

    /**
     * Injected file access object.
     */
    @Inject
    private JavaIoFileSystemAccess fileAccess;

    /**
     * Used to collect all errors that happen during validation/generation.
     */
    @Inject
    private IssueCollector issueCollector;

    /**
     * Used to report error messages at the end.
     */
    @Inject
    private ReportingBackend reporter;


    /**
     * Supported CLI options.
     * <p>
     * Stores an Apache Commons CLI Option for each entry, sets it to be
     * if required if so specified, and stores whether or not to pass the
     * option to the code generator.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     */
    enum CLIOption {
        COMPILER(null, "target-compiler", true, false, "Target compiler to invoke.", true),
        CLEAN("c", "clean", false, false, "Clean before building.", true),
        HELP("h", "help", false, false, "Display this information.", true),
        NO_COMPILE("n", "no-compile", false, false, "Do not invoke target compiler.", true),
        REBUILD("r", "rebuild", false, false, "Rebuild the LF compiler first.", false),
        UPDATE("u", "update-deps", false, false, "Update dependencies and rebuild the LF compiler (requires Internet connection).", false),
        FEDERATED("f", "federated", false, false, "Treat main reactor as federated.", false),
        THREADS("t", "threads", false, false, "Specify the default number of threads.", true),
        OUTPUT_PATH("o", "output-path", true, false, "Specify the root output directory.", false),
        RUNTIME_VERSION(null, "runtime-version", true, false, "Specify the version of the runtime library used for compiling LF programs.", true),
        EXTERNAL_RUNTIME_PATH(null, "external-runtime-path", true, false, "Specify an external runtime library to be used by the compiled binary.", true);
        
        /**
         * The corresponding Apache CLI Option object.
         */
        public final Option option;
        
        /**
         * Whether or not to pass this option to the code generator.
         */
        public final boolean passOn;
        
        /**
         * Construct a new CLIOption.
         * 
         * @param opt         The short option name. E.g.: "f" denotes a flag
         *                    "-f".
         * @param longOpt     The long option name. E.g.: "foo" denotes a flag
         *                    "--foo".
         * @param hasArg      Whether or not this option has an argument. E.g.:
         *                    "--foo bar" where "bar" is the argument value.
         * @param isReq       Whether or not this option is required. If it is
         *                    required but not specified a menu is shown.
         * @param description The description used in the menu.
         * @param passOn      Whether or not to pass this option as a property
         *                    to the code generator.
         */
        CLIOption(String opt, String longOpt, boolean hasArg, boolean isReq, String description, boolean passOn) {
            this.option = new Option(opt, longOpt, hasArg, description);
            option.setRequired(isReq);
            this.passOn = passOn;
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
        
        /**
         * Return a list of options that are to be passed on to the code
         * generator.
         * 
         * @return List of options that must be passed on to the code gen stage.
         */
        public static List<Option> getPassedOptions() {
            return Arrays.stream(CLIOption.values())
                         .filter(opt -> opt.passOn).map(opt -> opt.option)
                         .collect(Collectors.toList());
        }
        
    }

    /**
     * Indicate whether or not a rebuild and library update must occur.
     * @return whether or not the update flag is present.
     */
    private boolean mustUpdate() {
        return cmd.hasOption(CLIOption.UPDATE.option.getOpt());
    }
    
    /**
     * Indicate whether or not a rebuild must occur.
     * @return whether or not the rebuild or update flag is present.
     */
    private boolean mustRebuild() {
        return mustUpdate() || cmd.hasOption(CLIOption.REBUILD.option.getOpt());
    }
    
    /**
     * Main function of the stand-alone compiler.
     * @param args CLI arguments
     */
    public static void main(final String[] args) {
        final ReportingBackend reporter = new ReportingBackend(new Io());

        // Injector used to obtain Main instance.
        final Injector injector = new LFStandaloneSetup(new LFRuntimeModule(), new LFStandaloneModule(reporter))
            .createInjectorAndDoEMFRegistration();
        // Main instance.
        final Main main = injector.getInstance(Main.class);
        // Apache Commons Options object configured to according to available CLI arguments.
        Options options = CLIOption.getOptions();
        // CLI arguments parser.
        CommandLineParser parser = new DefaultParser();
        // Helper object for printing "help" menu.
        HelpFormatter formatter = new HelpFormatter();
        // Object used to print messages.
        // We could easily support eg a "quiet" mode that
        // disables warnings, or a "no colors" mode for CI environments.

        try {
            String mainClassUrl = Main.class.getResource("Main.class").toString();
            String jarUrl = mainClassUrl.replace("jar:", "").replace(MAIN_PATH_IN_JAR, "");

            main.jarPath = Paths.get(new URL(jarUrl).toURI());
            main.srcPath = main.jarPath.getParent().getParent().getParent().resolve("src").normalize();
            main.rootPath = main.jarPath.getParent().getParent().getParent().getParent().normalize();
        } catch (MalformedURLException | URISyntaxException e) {
            reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
        }

        try {
            main.cmd = parser.parse(options, args, true);
            
            if (main.cmd.hasOption(CLIOption.HELP.option.getOpt())) {
                formatter.printHelp("lfc", options);
                System.exit(0);
            }
            
            // If the rebuild flag is not used, or if it is used but the jar
            // is not out of date, continue with the normal flow of execution.
            if (!main.mustRebuild() || (main.mustRebuild() && !main.rebuildAndFork())) {
                List<String> files = main.cmd.getArgList();
                
                if (files.size() < 1) {
                    reporter.printFatalErrorAndExit("No input files.");
                }
                try {
                    List<Path> paths = files.stream().map(Paths::get).collect(Collectors.toList());
                    main.runGenerator(paths, injector);
                } catch (RuntimeException e) {
                    reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
                }
            }
        } catch (ParseException e) {
            reporter.printFatalError("Unable to parse commandline arguments. Reason: " + e.getMessage());
            formatter.printHelp("lfc", options);
            System.exit(1);
        }
    }
    
    /**
     * Fork off a new process (that is an execution of a freshly rebuilt jar)
     * and wait for it to return.
     * 
     * @param cmd The CommandLine object that has stored in it the CLI 
     * arguments of the parent process, to be passed on to the child process.
     */
    private void forkAndWait(CommandLine cmd) {
        List<String> cmdList = new ArrayList<>();
        cmdList.add("java");
        cmdList.add("-jar");
        cmdList.add(jarPath.toString());
        for (Option o : cmd.getOptions()) {
            // Drop -r and -u flags to prevent an infinite loop in case the
            // source fails to compile.
            if (!CLIOption.REBUILD.option.equals(o)
                    && !CLIOption.UPDATE.option.equals(o)) {
                // pass all other options on to the new command
                cmdList.add("--" + o.getLongOpt());
                if (o.hasArg()) {
                    cmdList.add(o.getValue());
                }
            }
        }
        cmdList.addAll(cmd.getArgList());
        Process p;
        try {
            p = new ProcessBuilder(cmdList).inheritIO().start();
            p.waitFor();
        } catch (IOException e) {
            reporter.printFatalError("Unable to fork off child process. Reason: " + e);
        } catch (InterruptedException e) {
            reporter.printError("Child process was interupted. Exiting.");
        }
        
    }
    
    private boolean modifiedFilesExist(Path start, long mod) throws IOException {
        return ((Files.find(start, Integer.MAX_VALUE,
                (path, attr) -> (attr.lastModifiedTime()
                        .compareTo(FileTime.fromMillis(mod)) > 0))).count() > 0);
    }
    
    /**
     * Indicate whether or not there is any work to do.
     * 
     * @return True if a rebuild is necessary, false otherwise.
     */
    private boolean needsUpdate() {
        File jar = jarPath.toFile();
        boolean outOfDate = false;
        try {
            outOfDate = (!jar.exists() || modifiedFilesExist(
                    srcPath, jar.lastModified()));

        } catch (IOException e) {
            reporter.printFatalErrorAndExit("Rebuild unsuccessful. Reason: " + e);
        }
        return outOfDate;
    }
    
    /**
     * Rebuild and return. If the rebuilding failed, exit.
     */
    private void rebuildOrExit() {
        LinkedList<String> cmdList = new LinkedList<String>();
        if (System.getProperty("os.name").startsWith("Windows")) {
            cmdList.add(".\\gradlew.bat");
        } else {
            cmdList.add("./gradlew");
        }
        cmdList.add("buildLfc");
        if (!this.mustUpdate()) {
            cmdList.add("--offline");
        }
        ProcessBuilder build = new ProcessBuilder(cmdList);
        build.directory(rootPath.toFile());
    
        try {
            Process p = build.start();
            // Read the output from the build.
            String result = new String(p.getInputStream().readAllBytes());
            
            p.waitFor();
            if (p.exitValue() == 0) {
                reporter.printInfo("Rebuild successful; forking off updated version of lfc.");
            } else {
                reporter.printFatalErrorAndExit("Rebuild failed. Reason: " + result);
            }
        } catch (Exception e) {
            reporter.printFatalErrorAndExit("Rebuild failed. Reason: " + e.getMessage());
        }
    }
    
    /**
     * Rebuild and fork if an update is needed. If the rebuild was successful,
     * return true. If no update was needed, return false. If the rebuild was
     * needed but failed, exit.
     * @return
     */
    private boolean rebuildAndFork() {
        // jar:file:<root>org.lflang.lfc/build/libs/org.lflang.lfc-<semver>-SNAPSHOT-all.jar!/org/lflang/lfc/Main.class
        if (needsUpdate()) {
            // Only rebuild if the jar is out-of-date.
            reporter.printInfo("Jar file is missing or out-of-date; running Gradle.");
            rebuildOrExit();
            forkAndWait(cmd);
        } else {
            reporter.printInfo("Not rebuilding; already up-to-date.");
            return false;
        }
        return true;
    }
  
    /**
     * Store arguments as properties, to be passed on to the generator.
     */
    protected Properties getProps(CommandLine cmd) {
        Properties props = new Properties();
        List<Option> passOn = CLIOption.getPassedOptions();
        for (Option o : cmd.getOptions()) {
            if (passOn.contains(o)) {
                String value = "";
                if (o.hasArg()) {
                    value = o.getValue();
                }
                props.setProperty(o.getLongOpt(), value);
            }
        }
        return props;
    }


    /**
     * Find the package root by looking for an 'src' directory. Print a warning
     * if none can be found and return the current working directory instead.
     *
     * @param input The *.lf file to find the package root for.
     * @return The package root, or the current working directory if none
     * exists.
     */
    private Path findPackageRoot(final Path input) {
        Path p = input;
        do {
            p = p.getParent();
            if (p == null) {
                reporter.printWarning("File '" + input.getFileName() + "' is not located in an 'src' directory.");
                reporter.printWarning("Adopting the current working directory as the package root.");
                return Paths.get(".").toAbsolutePath();
            }
        } while (!p.toFile().getName().equals("src"));
        return p.getParent();
    }


    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    private void runGenerator(List<Path> files, Injector injector) {
        Properties properties = this.getProps(cmd);
        String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
        Path root = null;
        if (cmd.hasOption(pathOption)) {
            root = Paths.get(cmd.getOptionValue(pathOption)).normalize();
            if (!Files.exists(root)) { // FIXME: Create it instead?
                reporter.printFatalErrorAndExit("Output location '" + root + "' does not exist.");
            }
            if (!Files.isDirectory(root)) {
                reporter.printFatalErrorAndExit("Output location '" + root + "' is not a directory.");
            }
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(path + ": No such file or directory");
            }
        }
        for (Path path : files) {
            path = path.toAbsolutePath();
            Path pkgRoot = findPackageRoot(path);
            String resolved;
            if (root != null) {
                resolved = root.resolve("src-gen").toString();
            } else {
                resolved = pkgRoot.resolve("src-gen").toString();
            }
            this.fileAccess.setOutputPath(resolved);

            final Resource resource = getValidatedResource(path);
            
            exitIfCollectedErrors();

            StandaloneContext context = new StandaloneContext();
            context.setArgs(properties);
            context.setCancelIndicator(CancelIndicator.NullImpl);
            context.setPackageRoot(pkgRoot);
            context.setReporter(injector.getInstance(ErrorReporter.class));

            this.generator.generate(resource, this.fileAccess, context);

            exitIfCollectedErrors();
            issueCollector.getAllIssues().forEach(reporter::printIssue);

            System.out.println("Code generation finished.");
        }
    }


    /**
     * If some errors were collected, print them and abort execution. Otherwise return.
     */
    private void exitIfCollectedErrors() {
        if (issueCollector.getErrorsOccurred()) {
            // if there are errors, don't print warnings.
            List<LfIssue> errors = issueCollector.getErrors();
            errors.forEach(reporter::printIssue);
            String cause = errors.size() == 1 ? "previous error"
                                              : errors.size() + " previous errors";
            reporter.printFatalErrorAndExit("Aborting due to " + cause);
        }
    }
    
    /**
     * Given a path, obtain a resource and validate it. If issues arise during validation,
     * these are recorded using the issue collector.
     * 
     * @param path Path to the resource to validate.
     * @return A validated resource
     */
    private Resource getValidatedResource(Path path) {
        final ResourceSet set = this.resourceSetProvider.get();
        final Resource resource =
            set.getResource(URI.createFileURI(path.toString()), true);

        if (cmd.hasOption(CLIOption.FEDERATED.option.getOpt())) {
            if (!ASTUtils.makeFederated(resource)) {
                reporter.printError("Unable to change main reactor to federated reactor.");
            }
        }

        List<Issue> issues = this.validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl);
        
        for (Issue issue : issues) {
            URI uri = issue.getUriToProblem(); // Issues may also relate to imported resources.
            try {
                issueCollector.accept(new LfIssue(issue.getMessage(),
                    issue.getSeverity(), issue.getLineNumber(),
                    issue.getColumn(), issue.getLength(), FileConfig.toPath(uri)));
            } catch (IOException e) {
                reporter.printError("Unable to convert '" + uri + "' to path.");
            }
        }
        return resource;
    }
}
