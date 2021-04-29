/**
 * Stand-alone version of the Lingua Franca compiler (lfc).
 */
package org.lflang.generator;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileTime;
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
import org.lflang.LFStandaloneSetup;

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
            new String[] { "!", "org", "lflang", "generator", "Main.class" });
    
    /**
     * ANSI sequence color escape sequence for red bold font.
     */
    private final static String RED_BOLD = "\033[1;31m";

    /**
     * ANSI sequence color escape sequence for ending red bold font.
     */
    private final static String END_RED_BOLD = "\033[0m";
    
    /**
     * ANSI sequence color escape sequence for bold font.
     */
    private final static String BOLD = "\u001b[1m";
    
    /**
     * ANSI sequence color escape sequence for ending bold font.
     */
    private final static String END_BOLD = "\u001b[0m";
    
    /**
     * Header used when printing messages.
     */
    private final static String HEADER = bold("lfc: ");
    
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
     * Print a fatal error message prefixed with a header that indicates the
     * source and severity.
     * 
     * @param message The message to print.
     */
    public static void printFatalError(String message) {
        System.err.println(HEADER + redAndBold("fatal error: ") + message);
    }
    
    /**
     * Print an error message prefixed with a header that indicates the source 
     * and severity.
     * 
     * @param message The message to print.
     */
    public static void printError(String message) {
        System.err.println(HEADER + redAndBold("error: ") + message);
    }
    
    /**
     * Print an informational message prefixed with a header that indicates the
     * source and severity.
     * 
     * @param message The message to print.
     */
    public static void printInfo(String message) {
        System.out.println(HEADER + bold("info: ") + message);
    }
    
    /**
     * Print a warning message prefixed with a header that indicates the
     * source and severity.
     * 
     * @param message The message to print.
     */
    public static void printWarning(String message) {
        System.out.println(HEADER + bold("warning: ") + message);
    }
    
    /**
     * Return the given string in bold face.
     * 
     * @param s String to type set.
     * @return a bold face version of the given string.
     */
    public static String bold(String s) {
        return BOLD + s + END_BOLD;
    }
    
    /**
     * Return the given string in red color and bold face.
     * 
     * @param s String to type set.
     * @return a red and bold face version of the given string.
     */
    public static String redAndBold(String s) {
        return RED_BOLD + s + END_RED_BOLD;
    }
    
    /**
     * Supported CLI options.
     * 
     * Stores an Apache Commons CLI Option for each entry, sets it to be
     * if required if so specified, and stores whether or not to pass the
     * option to the code generator.
     * 
     * @author Marten Lohstroh <marten@berkeley.edu>
     */
    enum CLIOption {
        COMPILER("c", "target-compiler", true, false, "Target compiler to invoke.", true),
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
            return Arrays.asList(CLIOption.values()).stream()
                    .filter(opt -> opt.passOn).map(opt -> opt.option)
                    .collect(Collectors.toList());
        }
        
    }

    /**
     * Indicate whether or not a rebuild and library update must occur.
     * @return whether or not the update flag is present.
     */
    private boolean mustUpdate() {
        if (cmd.hasOption(CLIOption.UPDATE.option.getOpt())) {
            return true;
        }
        return false;
    }
    
    /**
     * Indicate whether or not a rebuild must occur.
     * @return whether or not the rebuild or update flag is present.
     */
    private boolean mustRebuild() {
        if (mustUpdate() || cmd.hasOption(CLIOption.REBUILD.option.getOpt())) {
            return true;
        }
        return false;
    }
    
    /**
     * Main function of the stand-alone compiler.
     * @param args CLI arguments
     */
    public static void main(final String[] args) {
        
        /**
         * Injector used to obtain Main instance.
         */
        final Injector injector = new LFStandaloneSetup()
                .createInjectorAndDoEMFRegistration();
        
        /**
         * Main instance.
         */
        final Main main = injector.<Main>getInstance(Main.class);

        /**
         * Apache Commons Options object configured to according to available 
         * CLI arguments. 
         */
        Options options = CLIOption.getOptions();

        /**
         * CLI arguments parser.
         */
        CommandLineParser parser = new DefaultParser();
        
        /**
         * Helper object for printing "help" menu.
         */
        HelpFormatter formatter = new HelpFormatter();

        try {
            String mainClassUrl = Main.class.getResource("Main.class").toString();
            String jarUrl = mainClassUrl.replace("jar:", "").replace(MAIN_PATH_IN_JAR, "");

            main.jarPath = Paths.get(new URL(jarUrl).toURI());
            main.srcPath = main.jarPath.getParent().resolve(Paths.get("..", "..", "src")).normalize();
            main.rootPath = main.jarPath.getParent().resolve(Paths.get("..", "..", "..")).normalize();
        } catch (MalformedURLException | URISyntaxException e) {
            printFatalError("An unexpected error occurred:");
            e.printStackTrace();
            System.exit(1);
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
                    printFatalError("No input files.");
                    System.exit(1);
                }
                try {
                    main.runGenerator(files);
                } catch (RuntimeException e) {
                    printFatalError("An unexpected error occurred:");
                    e.printStackTrace();
                    System.exit(1);
                }
            }
        } catch (ParseException e) {
            printFatalError("Unable to parse commandline arguments. Reason:");
            System.err.println(e.getMessage());
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
        LinkedList<String> cmdList = new LinkedList<String>();
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
            printFatalError("Unable to fork off child process. Reason:");
            System.err.println(e.getMessage());
        } catch (InterruptedException e) {
            printError("Child process was interupted. Exiting.");
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
            printFatalError("Rebuild unsuccessful. Reason:");
            System.err.println(e.getMessage());
            System.exit(1);
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
        cmdList.add("generateStandaloneCompiler");
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
                printInfo("Rebuild successful; forking off updated version of lfc.");
            } else {
                printFatalError("Rebuild failed. Reason:");
                System.err.print(result);
                System.exit(1);
            }
        } catch (Exception e) {
            printFatalError("Rebuild failed. Reason:");
            System.err.println(e.getMessage());
            System.exit(1);
        }
    }
    
    /**
     * Rebuild and fork if an update is needed. If the rebuild was successful,
     * return true. If no update was needed, return false. If the rebuild was
     * needed but failed, exit.
     * @return
     */
    private boolean rebuildAndFork() {
        // jar:file:<root>org.lflang.linguafranca/build/libs/org.lflang.linguafranca-1.0.0-SNAPSHOT-all.jar!/org/icyphy/generator/Main.class
        if (needsUpdate()) {
            // Only rebuild if the jar is out-of-date.
            printInfo("Jar file is missing or out-of-date; running Gradle.");
            rebuildOrExit();
            forkAndWait(cmd);
        } else {
            printInfo("Not rebuilding; already up-to-date.");
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
     * @param f The *.lf file to find the package root for.
     * @return The package root, or the current working directory if none
     *         exists.
     */
    private static Path findPackageRoot(File f) {
        Path p = f.toPath();
        do {
            p = p.getParent();
            if (p == null) {
                printWarning("File '" + f.getName() + "' is not located in an 'src' directory.");
                printWarning("Adopting the current working directory as the package root.");
                return Paths.get(new File("").getAbsolutePath());
            }
        } while (!p.toFile().getName().equals("src"));
        return p.getParent();
    }
    
    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    protected void runGenerator(List<String> files) {
        Properties properties = this.getProps(cmd);
        String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
        File root = null;
        if (cmd.hasOption(pathOption)) {
            root = new File(cmd.getOptionValue(pathOption));
            if (!root.exists()) { // FIXME: Create it instead?
                printFatalError("Output location '" + root + "' does not exist.");
                System.exit(1);
            }
            if (!root.isDirectory()) {
                printFatalError("Output location '" + root + "' is not a directory.");
                System.exit(1);
            }
        }
        
        for (String file : files) {
            final File f = new File(file);
            if (!f.exists()) {
                printFatalError(
                        file.toString() + ": No such file or directory");
                System.exit(1);
            }
        }
        for (String file : files) {
            Path pkgRoot = findPackageRoot(new File(file).getAbsoluteFile());
            final File f = new File(file);
            String resolved = "";
            try {
                if (root != null) {
                    resolved = new File(root, "src-gen").getCanonicalPath();
                } else {
                    resolved = new File(pkgRoot.toFile(), "src-gen").getCanonicalPath();
                }
                this.fileAccess.setOutputPath(resolved);
            } catch (IOException e) {
              printFatalError("Could not access '" + resolved + "'.");
            }
            
            final ResourceSet set = this.resourceSetProvider.get();
            final Resource resource = set
                    .getResource(URI.createFileURI(f.getAbsolutePath()), true);
            
            if (cmd.hasOption(CLIOption.FEDERATED.option.getOpt())) {
                if (!ASTUtils.makeFederated(resource)) {
                    printError("Unable to change main reactor to federated reactor.");
                }
            }
            
            final List<Issue> issues = this.validator.validate(resource,
                    CheckMode.ALL, CancelIndicator.NullImpl);
            if (!issues.isEmpty()) {
                printFatalError("Unable to validate resource. Reason:");
                issues.forEach(issue -> System.err.println(issue));
                System.exit(1);
            }
            
            StandaloneContext context = new StandaloneContext();
            context.setArgs(properties);
            context.setCancelIndicator(CancelIndicator.NullImpl);
            context.setPackageRoot(pkgRoot);

            this.generator.generate(resource, this.fileAccess, context);
            System.out.println("Code generation finished.");
        }
    }
    
    public ResourceSet getResourceSet() {
        return this.resourceSetProvider.get();
    }
}
