package org.lflang;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.RuntimeIOException;

import org.lflang.TargetConfig.Mode;
import org.lflang.generator.StandaloneContext;
import org.lflang.lf.Reactor;

/**
 * Base class that governs the interactions between code generators and the file system.
 *  
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public class FileConfig {

    // Public static fields.
    
    public final static String DEFAULT_SRC_DIR = "src";
    
    /**
     * Default name of the directory to store binaries in.
     */
    public final static String DEFAULT_BIN_DIR = "bin";
    
    /**
     * Default name of the directory to store generated sources in.
     */
    public final static String DEFAULT_SRC_GEN_DIR = "src-gen";

    /**
     * Suffix that when appended to the name of a federated reactor yields 
     * the name of its corresponding RTI executable.
     */
    public final static String RTI_BIN_SUFFIX = "_RTI";
    
    /**
     * Suffix that when appended to the name of a federated reactor yields 
     * the name of its corresponding distribution script.
     */
    public final static String RTI_DISTRIBUTION_SCRIPT_SUFFIX = "_distribute.sh";


    // Public fields.

    /**
     * The directory in which to put binaries, if the code generator produces any.
     */
    public final Path binPath;

    /**
     * Object for abstract file system access.
     */
    public final IFileSystemAccess2 fsa;

    /**
     * Object used for communication between the IDE or stand-alone compiler
     * and the code generator.
     */
    public final IGeneratorContext context;

    /**
     * The name of the main reactor, which has to match the file name (without
     * the '.lf' extension).
     */
    public final String name;

    /**
     * The directory that is the root of the package in which the .lf source file resides. This path is determined
     * differently depending on whether the compiler is invoked through the IDE or from the command line. In the former
     * case, the package is the project root that the source resides in. In the latter case, it is the parent directory
     * of the nearest `src` directory up the hierarchy, if there is one, or just the `outPath` if there is none. It is
     * recommended to always keep the sources in a `src` directory regardless of the workflow, in which case the
     * output behavior will be identical irrespective of the way the compiler is invoked.
     */
    public final Path srcPkgPath;

    /**
     * The file containing the main source code.
     * This is the Eclipse eCore view of the file, which is distinct
     * from the XText view of the file and the OS view of the file.
     */
    public final Resource resource;

    /**
     * If running in an Eclipse IDE, the iResource refers to the
     * IFile representing the Lingua Franca program.
     * This is the XText view of the file, which is distinct
     * from the Eclipse eCore view of the file and the OS view of the file.
     */
    public final IResource iResource;

    /**
     * The full path to the file containing the .lf file including the
     * full filename with the .lf extension.
     */
    public final Path srcFile;

    /**
     * The directory in which the source .lf file was found.
     */
    public final Path srcPath;

    // Protected fields.

    /**
     * Path representation of srcGenRoot, the root directory for generated
     * sources.
     */
    protected Path srcGenBasePath;

    /**
     * The directory in which to put the generated sources.
     * This takes into account the location of the source file relative to the
     * package root. Specifically, if the source file is x/y/Z.lf relative
     * to the package root, then the generated sources will be put in x/y/Z
     * relative to srcGenBasePath.
     */
    private Path srcGenPath;

    // private fields

    /**
     * The parent of the directory designated for placing generated sources into (`./src-gen` by default). Additional
     * directories (such as `bin` or `build`) should be created as siblings of the directory for generated sources,
     * which means that such directories should be created relative to the path assigned to this class variable.
     *
     * The generated source directory is specified in the IDE (Project Properties->LF->Compiler->Output Folder). When
     * invoking the standalone compiler, the output path is specified directly using the `-o` or `--output-path` option.
     */
    private final Path outPath;

    /**
     * The directory that denotes the root of the package to which the
     * generated sources belong. Even if the target language does not have a
     * notion of packages, this directory groups all files associated with a
     * single main reactor.
     * of packages.
     */
    private final Path srcGenPkgPath;


    public FileConfig(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) throws IOException {
        this.resource = resource;
        this.fsa = fsa;
        this.context = context;

        this.srcFile = toPath(this.resource);

        this.srcPath = srcFile.getParent();
        this.srcPkgPath = getPkgPath(resource, context);

        this.srcGenBasePath = toPath(getSrcGenRoot(fsa));
        this.name = nameWithoutExtension(this.srcFile);
        this.srcGenPath = getSrcGenPath(this.srcGenBasePath, this.srcPkgPath,
                this.srcPath, name);
        this.srcGenPkgPath = this.srcGenPath;
        this.outPath = srcGenBasePath.getParent();
        this.binPath = getBinPath(this.srcPkgPath, this.srcPath, this.outPath, context);
        this.iResource = getIResource(resource);
    }
/**
     * A copy constructor for FileConfig objects. Children of this class can
     * use this constructor to obtain a copy of a parent object.
     *
     * @param fileConfig An object of FileConfig
     * @throws IOException
     */
    protected FileConfig(FileConfig fileConfig) throws IOException {
        this.resource = fileConfig.resource;
        this.fsa = fileConfig.fsa;
        this.context = fileConfig.context;

        this.srcFile = fileConfig.srcFile;

        this.srcPath = srcFile.getParent();
        this.srcPkgPath = fileConfig.srcPkgPath;

        this.srcGenBasePath = fileConfig.srcGenBasePath;
        this.name = nameWithoutExtension(this.srcFile);
        this.srcGenPath = getSrcGenPath(this.srcGenBasePath, this.srcPkgPath,
                this.srcPath, name);
        this.srcGenPkgPath = this.srcGenPath;
        this.outPath = srcGenBasePath.getParent();
        this.binPath = getBinPath(this.srcPkgPath, this.srcPath, this.outPath, context);
        this.iResource = getIResource(resource);
    }

    // Getters to be overridden in derived classes.

    protected void setSrcGenPath(Path srcGenPath) {
        this.srcGenPath = srcGenPath;
    }


    /**
     * Get the iResource corresponding to the provided resource if it can be
     * found.
     * @throws IOException
     */
    public IResource getIResource(Resource r) throws IOException {
        IResource iResource = null;
        java.net.URI uri = toPath(r).toFile().toURI();
        if (r.getURI().isPlatform()) {
            IWorkspaceRoot workspaceRoot = ResourcesPlugin.getWorkspace().getRoot();
            if (uri != null) {
                 IFile[] files = workspaceRoot.findFilesForLocationURI(uri);
                 if (files != null && files.length > 0 && files[0] != null) {
                     iResource = files[0];
                 }
            }
        } else {
            // FIXME: find the iResource outside Eclipse
        }
        return iResource;
    }
    
    /**
     * Get the specified path as an Eclipse IResource or, if it is not found, then
     * return the iResource for the main file.
     * 
     */
    public IResource getIResource(Path path) {
        return getIResource(path.toUri());
    }
    
    /**
     * Get the specified path as an Eclipse IResource or, if it is not found, then
     * return the iResource for the main file.
     * 
     */
    public IResource getIResource(File file) {
        return getIResource(file.toURI());
    }
    
    /**
     * Get the specified uri as an Eclipse IResource or, if it is not found, then
     * return the iResource for the main file.
     * For some inexplicable reason, Eclipse uses a mysterious parallel to the file
     * system, and when running in INTEGRATED mode, for some things, you cannot access
     * files by referring to their file system location. Instead, you have to refer
     * to them relative the workspace root. This is required, for example, when marking
     * the file with errors or warnings or when deleting those marks. 
     * 
     * @param uri A java.net.uri of the form "file://path".
     */
    public IResource getIResource(java.net.URI uri) {
        IResource resource = iResource; // Default resource.
        // For some peculiar reason known only to Eclipse developers,
        // the resource cannot be used directly but has to be converted
        // a resource relative to the workspace root.
        IWorkspaceRoot workspaceRoot = ResourcesPlugin.getWorkspace().getRoot();
         
        IFile[] files = workspaceRoot.findFilesForLocationURI(uri);
        if (files != null && files.length > 0 && files[0] != null) {
            resource = files[0];
        }
        return resource;
    }

    /** 
     * Get the file name of a resource without file extension
     */
    public static String getName(Resource r) throws IOException {
        return nameWithoutExtension(toPath(r));
    }
    
    /**
     * Get the directory a resource is located in relative to the root package
     */
    public Path getDirectory(Resource r) throws IOException {
        return getSubPkgPath(this.srcPkgPath, toPath(r).getParent());
    }

    /**
     * The parent of the directory designated for placing generated sources into (`./src-gen` by default). Additional
     * directories (such as `bin` or `build`) should be created as siblings of the directory for generated sources,
     * which means that such directories should be created relative to the path assigned to this class variable.
     *
     * The generated source directory is specified in the IDE (Project Properties->LF->Compiler->Output Folder). When
     * invoking the standalone compiler, the output path is specified directly using the `-o` or `--output-path` option.
     */
    public Path getOutPath() {
        return outPath;
    }

    /**
     * The directory in which to put the generated sources.
     * This takes into account the location of the source file relative to the
     * package root. Specifically, if the source file is x/y/Z.lf relative
     * to the package root, then the generated sources will be put in x/y/Z
     * relative to srcGenBasePath.
     */
    public Path getSrcGenPath() {
        return srcGenPath;
    }


    /**
     * Path representation of srcGenRoot, the root directory for generated
     * sources. This is the root, meaning that if the source file is x/y/Z.lf
     * relative to the package root, then the generated sources will be put in x/y/Z
     * relative to this URI.
     */
    public Path getSrcGenBasePath() {
        return srcGenBasePath;
    }

    /**
     * The directory that denotes the root of the package to which the
     * generated sources belong. Even if the target language does not have a
     * notion of packages, this directory groups all files associated with a
     * single main reactor.
     */
    public Path getSrcGenPkgPath() {
        return srcGenPkgPath;
    }
    
    /**
     * Return the directory in which to put the generated sources for the 
     * RTI. By default, this is the same as the regular src-gen directory.
     */
    public Path getRTISrcPath() {
        return this.srcGenPath;
    }

    /**
     * Return the directory in which to put the generated binaries for the
     * RTI. By default, this is the same as the regular src-gen directory.
     */
    public Path getRTIBinPath() {
        return this.binPath;
    }

    /**
     * Return the output directory for generated binary files.
     */
    private static Path getBinPath(Path pkgPath, Path srcPath, Path outPath, IGeneratorContext context) {
        Path root = outPath.resolve(DEFAULT_BIN_DIR);
        // The context might have a directive to structure the binary directory
        // hierarchically (just like we do with the generated sources).
        if (context instanceof StandaloneContext) {
           if (((StandaloneContext) context).isHierarchicalBin()) {
               return root.resolve(getSubPkgPath(pkgPath, srcPath));
           }
        }
        return root;
    }
    
    /**
     * FIXME: Forget why the trailing slash mattered. It probably shouldn't.
     * 
     */
    private static URI getSrcGenRoot(IFileSystemAccess2 fsa) {
        URI srcGenURI = fsa.getURI("");
        if (srcGenURI.hasTrailingPathSeparator()) {
            srcGenURI = srcGenURI.trimSegments(1);
        }
        return srcGenURI;
    }
    
    protected static Path getSrcGenPath(Path srcGenRootPath, Path pkgPath,
            Path srcPath, String name) throws IOException {
        return srcGenRootPath.resolve(getSubPkgPath(pkgPath, srcPath)).resolve(name);
    }
    
    /**
     * Given a path that denotes the root of the package and a path
     * that denotes the full path to a source file (not including the
     * file itself), return the relative path from the root of the 'src'
     * directory, or, if there is no 'src' directory, the relative path 
     * from the root of the package. 
     * @param pkgPath The root of the package.
     * @param srcPath The path to the source.
     * @return
     */
    protected static Path getSubPkgPath(Path pkgPath, Path srcPath) {
        Path relSrcPath = pkgPath.relativize(srcPath);
        if (relSrcPath.startsWith(DEFAULT_SRC_DIR)) {
            int segments = relSrcPath.getNameCount(); 
            if (segments == 1) {
                return Paths.get("");
            } else {
                relSrcPath = relSrcPath.subpath(1, segments);
            }
        }
        return relSrcPath;
    }

    /**
     * Copy a given directory from 'src' to 'dest'.
     *
     * @param src The source directory path.
     * @param dest The destination directory path.
     * @throws IOException if copy fails.
     */
    public static void copyDirectory(Path src, Path dest) throws IOException {
        try (Stream<Path> stream = Files.walk(src)) {
            stream.forEach(source -> {
                // Handling checked exceptions in lambda expressions is
                // hard. See
                // https://www.baeldung.com/java-lambda-exceptions#handling-checked-exceptions.
                // An alternative would be to create a custom Consumer interface and use that
                // here.
                try {
                    copyFile(source, dest.resolve(src.relativize(source)));
                } catch (IOException e) {
                    throw new RuntimeIOException(e);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            });
        }
    }

    /**
     * Copy a given file from 'source' to 'destination'.
     *
     * @param source The source file path string.
     * @param destination The destination file path string.
     * @throws IOException if copy fails.
     */
    public static void copyFile(String source, String destination)  throws IOException {
        try {
            copyFile(Paths.get(source), Paths.get(destination));
        } catch (IOException e) {
            throw e;
        }
    }

    /**
     * Copy a given file from 'source' to 'destination'.
     *
     * @param source The source file path.
     * @param destination The destination file path.
     * @throws IOException if copy fails.
     */
    public static void copyFile(Path source, Path destination)  throws IOException {
        try {
            Files.copy(source, destination, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException e) {
            throw e;
        }
    }
    
    /**
     *  Lookup a file in the classpath and copy its contents to a destination path 
     *  in the filesystem.
     * 
     *  This also creates new directories for any directories on the destination
     *  path that do not yet exist.
     * 
     *  @param source The source file as a path relative to the classpath.
     *  @param destination The file system path that the source file is copied to.
     * @throws IOException 
     */
    public void copyFileFromClassPath(String source, String destination) throws IOException {
        InputStream sourceStream = this.getClass().getResourceAsStream(source);

        if (sourceStream == null) {
            throw new IOException(
                "A required target resource could not be found: " + source + "\n" +
                    "Perhaps a git submodule is missing or not up to date.\n" +
                    "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.\n" +
                    "Also try to refresh and clean the project explorer if working from eclipse.");
        }

        // Copy the file.
        try {
            // Make sure the directory exists
            File destFile = new File(destination);
            destFile.getParentFile().mkdirs();

            Files.copy(sourceStream, Paths.get(destination), StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException ex) {
            throw new IOException(
                "A required target resource could not be copied: " + source + "\n" +
                    "Perhaps a git submodule is missing or not up to date.\n" +
                    "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.",
                ex);
        } finally {
            sourceStream.close();
        }
    }

    /**
     * Copy a list of files from a given source directory to a given destination directory.
     * @param srcDir The directory to copy files from.
     * @param dstDir The directory to copy files to.
     * @param files The list of files to copy.
     * @throws IOException 
     */
    public void copyFilesFromClassPath(String srcDir, String dstDir, List<String> files) throws IOException {
        for (String file : files) {
            copyFileFromClassPath(srcDir + '/' + file, dstDir + File.separator + file);
        }
    }
    
   /**
    * Copy the 'fileName' from the 'srcDirectory' to the 'destinationDirectory'.
    * This function has a fallback search mechanism, where if `fileName` is not
    * found in the `srcDirectory`, it will try to find `fileName` via the following procedure:
    * 1- Search in LF_CLASSPATH. @see findFile()
    * 2- Search in CLASSPATH. @see findFile()
    * 3- Search for 'fileName' as a resource.
    *  That means the `fileName` can be '/path/to/class/resource'. @see java.lang.Class.getResourceAsStream()
    * 
    * @param fileName Name of the file
    * @param srcDir Where the file is currently located
    * @param dstDir Where the file should be placed
    * @return The name of the file in destinationDirectory
    */
   public String copyFileOrResource(String fileName, Path srcDir, Path dstDir) {
       // Try to copy the file from the file system.
       Path file = findFile(fileName, srcDir);
       if (file != null) {
           Path target = dstDir.resolve(file.getFileName());
           try {
               Files.copy(file, target, StandardCopyOption.REPLACE_EXISTING);
               return file.getFileName().toString();
           } catch (IOException e) {
               // Files has failed to copy the file, possibly since
               // it doesn't exist. Will try to find the file as a 
               // resource before giving up.
           }
       } 
       
       // Try to copy the file as a resource.
       // If this is missing, it should have been previously reported as an error.
       try {
           String filenameWithoutPath = fileName;
           int lastSeparator = fileName.lastIndexOf(File.separator);
           if (lastSeparator > 0) {
               filenameWithoutPath = fileName.substring(lastSeparator + 1); // FIXME: brittle. What if the file is in a subdirectory?
           }
           copyFileFromClassPath(fileName, dstDir + File.separator + filenameWithoutPath);
           return filenameWithoutPath;
       } catch (IOException ex) {
           // Ignore. Previously reported as a warning.
           System.err.println("WARNING: Failed to find file " + fileName);
       }
       
       return "";
   }

    /**
     * Check if a clean was requested from the standalone compiler and perform
     * the clean step.
     */
    public void cleanIfNeeded() {
        if (context instanceof StandaloneContext) {
            if (((StandaloneContext) context).getArgs().containsKey("clean")) {
                try {
                    doClean();
                } catch (IOException e) {
                    System.err.println("WARNING: IO Error during clean");
                }
            }
        }
    }

    /**
     * Recursively delete a directory if it exists.
     * 
     * @throws IOException
     */
    public void deleteDirectory(Path dir) throws IOException {
        if (Files.isDirectory(dir)) {
            System.out.println("Cleaning " + dir.toString());
            List<Path> pathsToDelete = Files.walk(dir)
                    .sorted(Comparator.reverseOrder())
                    .collect(Collectors.toList());
            for (Path path : pathsToDelete) {
                Files.deleteIfExists(path);
            }
        }
    }

    /**
     * Clean any artifacts produced by the code generator and target compilers.
     * 
     * The base implementation deletes the bin and src-gen directories. If the
     * target code generator creates additional files or directories, the
     * corresponding generator should override this method.
     * 
     * @throws IOException
     */
    public void doClean() throws IOException {
        deleteDirectory(binPath);
        deleteDirectory(srcGenBasePath);
    }
 
    /**
     * Remove files in the bin directory that may have been created.
     * Call this if a compilation occurs so that files from a previous
     * version do not accidentally get executed.
     */
    public void deleteBinFiles() {
        String name = nameWithoutExtension(this.srcFile);
        String[] files = this.binPath.toFile().list();
        List<String> federateNames = new LinkedList<String>(); // FIXME: put this in ASTUtils?
        resource.getAllContents().forEachRemaining(node -> {
            if (node instanceof Reactor) {
                Reactor r = (Reactor) node;
                if (r.isFederated()) {
                    r.getInstantiations().forEach(inst -> federateNames
                            .add(inst.getName()));
                }
            }
        });
        for (String f : files) {
            // Delete executable file or launcher script, if any.
            // Delete distribution file, if any.
            // Delete RTI file, if any.
            if (f.equals(name) || f.equals(getRTIBinName())
                    || f.equals(getRTIDistributionScriptName())) {
                this.binPath.resolve(f).toFile().delete();
            }
            // Delete federate executable files, if any.
            for (String federateName : federateNames) {
                if (f.equals(name + "_" + federateName)) {
                    this.binPath.resolve(f).toFile().delete();
                }
            }
        }
    }
    
    public static String nameWithoutExtension(Path file) {
        String name = file.getFileName().toString();
        int idx = name.lastIndexOf('.');
        return idx < 0 ? name : name.substring(0, idx);
    }
    
    private static Path getPkgPath(Resource resource, IGeneratorContext context) throws IOException {
        if (resource.getURI().isPlatform()) {
            File srcFile = toPath(resource).toFile();
            for (IProject r : ResourcesPlugin.getWorkspace().getRoot().getProjects()) {
                Path p = Paths.get(r.getLocation().toFile().getAbsolutePath());
                Path f = Paths.get(srcFile.getAbsolutePath());
                if (f.startsWith(p)) {
                    return p;
                }
            }
        } else if (context instanceof StandaloneContext) {
            return ((StandaloneContext)context).getPackageRoot();
        }
        
        throw new IOException("Unable to determine the package root.");
    }
    
    /**
     * Return a java.nio.Path object corresponding to the given URI.
     * @throws IOException 
     */
    public static Path toPath(URI uri) throws IOException {
        return Paths.get(toIPath(uri).toFile().getAbsolutePath());
    }

    /**
     * Return a java.nio.Path object corresponding to the given Resource.
     * @throws IOException 
     */
    public static Path toPath(Resource resource) throws IOException {
        return FileConfig.toPath(resource.getURI());
    }
    
    /**
     * Return an org.eclipse.core.runtime.Path object corresponding to the
     * given URI.
     * @throws IOException 
     */
    public static IPath toIPath(URI uri) throws IOException {
        if (uri.isPlatform()) {
            IPath path = new org.eclipse.core.runtime.Path(uri.toPlatformString(true));
            if (path.segmentCount() == 1) {
                return ResourcesPlugin.getWorkspace().getRoot().getProject(path.lastSegment()).getLocation();
            } else {
                return ResourcesPlugin.getWorkspace().getRoot().getFile(path).getLocation();
            }
        } else if (uri.isFile()) {
            return new org.eclipse.core.runtime.Path(uri.toFileString());
        } else {
            throw new IOException("Unrecognized file protocol in URI " + uri.toString());
        }
    }

    /**
     * Convert a given path to a unix-style string.
     * 
     * This ensures that '/' is used instead of '\' as file separator.
     */
    public static String toUnixString(Path path) {
        return path.toString().replace('\\', '/');
    }
    
    /**
     * Check whether a given file (i.e., a relative path) exists in the given
     *directory.
     * @param filename String representation of the filename to search for.
     * @param directory String representation of the director to search in.
     */
    public static boolean fileExists(String filename, Path directory) {
        // Make sure the file exists and issue a warning if not.
        Path file = findFile(filename, directory);
        if (file == null) {
            // See if it can be found as a resource.
            InputStream stream = FileConfig.class.getResourceAsStream(filename);
            if (stream == null) {
                return false;
            } else {
                // Sadly, even with this not null, the file may not exist.
                try {
                    stream.read();
                    stream.close();
                } catch (IOException ex) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Search for a given file name in the given directory.
     * If not found, search in directories in LF_CLASSPATH.
     * If there is no LF_CLASSPATH environment variable, use CLASSPATH,
     * if it is defined.
     * The first file found will be returned.
     * 
     * @param fileName The file name or relative path + file name
     * in plain string format
     * @param directory String representation of the director to search in.
     * @return A Java file or null if not found
     */
     public static Path findFile(String fileName, Path directory) {
        Path foundFile;

        // Check in local directory

        foundFile = directory.resolve(fileName);
        if (Files.isRegularFile(foundFile)) {
            return foundFile;
        }

        // Check in LF_CLASSPATH
        // Load all the resources in LF_CLASSPATH if it is set.
        String classpathLF = System.getenv("LF_CLASSPATH");
        if (classpathLF == null) {
            classpathLF = System.getenv("CLASSPATH");
        }
        if (classpathLF != null) {
            String[] paths = classpathLF.split(System.getProperty("path.separator"));
            for (String path : paths) {
                foundFile = Paths.get(path).resolve(fileName);
                if (Files.isRegularFile(foundFile)) {
                    return foundFile;
                }
            }
        }
        // Not found.
        return null;
    }
     
     /**
      * Return the name of the RTI executable.
      * @return
      */
     public String getRTIBinName() {
         return nameWithoutExtension(srcFile) + RTI_BIN_SUFFIX;
     }
     
     public File getRTIBinFile() {
         return this.binPath.resolve(getRTIBinName()).toFile();
     }
     
     /**
      * Return the name of the RTI distribution script.
      * @return
      */
     public String getRTIDistributionScriptName() {
         return nameWithoutExtension(srcFile) + RTI_DISTRIBUTION_SCRIPT_SUFFIX;
     }
     
     public File getRTIDistributionScriptFile() {
         return this.binPath.resolve(getRTIDistributionScriptName()).toFile();
     }
     
     public static String nameWithoutExtension(Resource r) throws IOException {
         return nameWithoutExtension(toPath(r));
     }
     
     /**
      * Determine which mode the compiler is running in.
      * Integrated mode means that it is running within an Eclipse IDE.
      * Standalone mode means that it is running on the command line.
      * 
      * FIXME: Not sure if that us the right place for this function. But
      *  the decision which mode we are in depends on a file (the resource),
      *  thus it seems to fit here.
      */
     public Mode getCompilerMode() {
         if (resource.getURI().isPlatform()) {
             return Mode.INTEGRATED;
         } else if (resource.getURI().isFile()) {
             return Mode.STANDALONE;
         } else {
             System.err.println("ERROR: Source file protocol is not recognized: " + resource.getURI());
             return Mode.UNDEFINED;
         }
     }
}
