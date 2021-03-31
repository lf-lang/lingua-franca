package org.icyphy;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;

import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.icyphy.generator.StandaloneContext;
import org.icyphy.linguaFranca.Reactor;

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
     * The directory that is the root of the package in which the source
     * file resides.
     */
    public final Path srcPkgPath;

    /**
     * The file containing the main source code.
     * This is the Eclipse eCore view of the file, which is distinct
     * from the XText view of the file and the OS view of the file.
     */
    public final Resource resource;
    
    /**
     * The full path to the file containing the .lf file including the
     * full filename with the .lf extension.
     */
    public final File srcFile;

    /**
     * The directory in which the source file was found.
     */
    public final Path srcPath;
    
    // Protected fields.
    
    /**
     * The parent of the specified directory for generated sources. Additional
     * directories created during the build process should be created relative
     * to this path.
     */
    protected Path outPath;
   
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
    protected Path srcGenPath;
    
    /**
     * The directory that denotes the root of the package to which the
     * generated sources belong. Even if the target language does not have a
     * notion of packages, this directory groups all files associated with a 
     * single main reactor.
     * of packages.
     */
    protected Path srcGenPkgPath;
    
    // Protected fields.
    
    /**
     * URI representation of the directory that is the parent of the specified
     * directory in which to store generated sources.
     */
    protected final URI outputRoot;

    /**
     * URI representation of the directory in which to store generated sources.
     * This is the root, meaning that if the source file is x/y/Z.lf relative
     * to the package root, then the generated sources will be put in x/y/Z
     * relative to this URI.
     */
    protected final URI srcGenRoot;
    
    public FileConfig(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) throws IOException {
        this.resource = resource;
        this.fsa = fsa;
        this.context = context;
        
        this.srcFile = toPath(this.resource.getURI()).toFile();
        
        this.srcPath = srcFile.toPath().getParent();
        this.srcPkgPath = getPkgPath(resource, context);
        
        this.srcGenRoot = getSrcGenRoot(fsa);
        this.srcGenBasePath = toPath(this.srcGenRoot);
        this.outputRoot = getOutputRoot(this.srcGenRoot);
        this.name = nameWithoutExtension(this.srcFile);
        this.srcGenPath = getSrcGenPath(this.srcGenBasePath, this.srcPkgPath,
                this.srcPath, name);
        this.srcGenPkgPath = this.srcGenPath;
        this.outPath = toPath(this.outputRoot);
        this.binPath = getBinPath(this.srcPkgPath, this.srcPath, this.outPath, context);
    }
    
    // Getters to be overridden in derived classes.
    
    public static String getName(Resource r) throws IOException {
        return nameWithoutExtension(toPath(r.getURI()).toFile());
    }
    
    public Path getOutPath() {
        return outPath;
    }
 
    public Path getSrcGenPath() {
        return srcGenPath;
    }

    
    public Path getSrcGenBasePath() {
        return srcGenBasePath;
    }

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

    private static URI getOutputRoot(URI srcGenRoot) {
        return URI.createURI(".").resolve(srcGenRoot);
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
    
    private static Path getSrcGenPath(Path srcGenRootPath, Path pkgPath,
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
    private static Path getSubPkgPath(Path pkgPath, Path srcPath) {
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
    
    public void createDirectories() {
        // FIXME
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
    
    public static String nameWithoutExtension(File file) {
        String name = file.getName();
        String[] tokens = name.split("\\.");
        if (tokens.length < 3) {
            return tokens[0];
        } else {
            StringBuffer s = new StringBuffer();
            for (int i=0; i < tokens.length -1; i++) {
                s.append(tokens[i]);
            }
            return s.toString();
        }
    }
    
    private static Path getPkgPath(Resource resource, IGeneratorContext context) throws IOException {
        if (resource.getURI().isPlatform()) {
            File srcFile = toPath(resource.getURI()).toFile();
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
    
    public static String toPathString(URI uri) throws IOException { // FIXME: maybe not even have this?
        return toIPath(uri).toOSString();
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
     * 
     * @param path
     * @return
     */
    public static String toUnixPath(Path path) {
        return path.toString().replace('\\', '/');
    }
    
    public static String toFileURI(Path path) {
        return "file:/" + FileConfig.toUnixPath(path);
    }
    
    public static String toFileURI(File file) {
        return "file:/" + FileConfig.toUnixPath(file.toPath());
    }
    
    /**
     * Check whether a given file (i.e., a relative path) exists in the given
     *directory.
     * @param filename String representation of the filename to search for.
     * @param directory String representation of the director to search in.
     */
    public static boolean fileExists(String filename, String directory) {
        // Make sure the file exists and issue a warning if not.
        File file = findFile(filename, directory);
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
     * Search for a given file name in the current directory.
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
     public static File findFile(String fileName, String directory) {

        File foundFile;

        // Check in local directory
        foundFile = new File(directory, fileName);
        if (foundFile.exists() && foundFile.isFile()) {
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
                foundFile = new File(path, fileName);
                if (foundFile.exists() && foundFile.isFile()) {
                    return foundFile;
                }
            }
        }
        // Not found.
        return null;
    }

     /**
      * Create a string representing the absolute file path of a resource.
      * @throws IOException
      * @deprecated 
      */
     public static String toPathString(Resource resource) throws IOException {
         return toPathString(resource.getURI());
     }

     /**
      * Create a string representing the absolute file path of a file relative to a file system access object.
     * @throws IOException
     * @deprecated 
      */
     public static String getAbsolutePath(IFileSystemAccess2 fsa, String file) throws IOException {
         return toPathString(fsa.getURI(file));
     }

     /**
      * Extract the name of a file from a path represented as a string.
      * If the file ends with '.lf', the extension is removed.
      * @deprecated
      */
     public static String getFilename(String path) {
         File f = new File(path);
         String name = f.getName();
         if (name.endsWith(".lf")) {
             name = name.substring(0, name.length() - 3); // FIXME: code duplication (see analuzeResource in GeneratorBase)
         }
         return name;
     }

     /**
      * Extract the directory from a path represented as a string.
      * @deprecated
      */
     public static String getDirectory(String path) {
         File f = new File(path);
         return f.getParent();
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
         return nameWithoutExtension(toPath(r.getURI()).toFile());
     }
}
