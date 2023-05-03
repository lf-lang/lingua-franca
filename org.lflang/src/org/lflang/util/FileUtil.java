package org.lflang.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.JarURLConnection;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.FileLocator;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.util.RuntimeIOException;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;

public class FileUtil {

    /**
     * Return the name of the file excluding its file extension.
     * @param file A Path object
     * @return The name of the file excluding its file extension.
     */
    public static String nameWithoutExtension(Path file) {
        String name = file.getFileName().toString();
        int idx = name.lastIndexOf('.');
        return idx < 0 ? name : name.substring(0, idx);
    }

    /**
     * Return the name of the file associated with the given resource,
     * excluding its file extension.
     * @param r Any {@code Resource}.
     * @return The name of the file associated with the given resource,
     * excluding its file extension.
     * @throws IOException If the resource has an invalid URI.
     */
    public static String nameWithoutExtension(Resource r) throws IOException {
        return nameWithoutExtension(toPath(r));
    }

    /**
     * Return a java.nio.Path object corresponding to the given URI.
     * @throws IOException If the given URI is invalid.
     */
    public static Path toPath(URI uri) throws IOException {
        return Paths.get(toIPath(uri).toFile().getAbsolutePath());
    }

    /**
     * Return a java.nio.Path object corresponding to the given Resource.
     * @throws IOException If the given resource has an invalid URI.
     */
    public static Path toPath(Resource resource) throws IOException {
        return toPath(resource.getURI());
    }

    /**
     * Return an org.eclipse.core.runtime.Path object corresponding to the
     * given URI.
     * @throws IOException If the given URI is invalid.
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
            throw new IOException("Unrecognized file protocol in URI " + uri);
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
     * Parse the string as file location and return it as URI.
     * Supports URIs, plain file paths, and paths relative to a model.
     * 
     * @param path the file location as string.
     * @param resource the model resource this file should be resolved relatively. May be null.
     * @return the (Java) URI or null if no file can be located.
     */
    public static java.net.URI locateFile(String path, Resource resource) {
        // Check if path is URL
        try {
            var uri = new java.net.URI(path);
            if(uri.getScheme() != null) { // check if path was meant to be a URI
                return uri;
            }
        } catch (Exception e) {
            // nothing
        }
        // Check if path exists as it is
        File file = new File(path);
        if (file.exists()) {
            try {
                return file.toURI();
            } catch (Exception e) {
                // nothing
            }
        }
        // Check if path is relative to LF file
        if (resource != null) {
            URI eURI = resource.getURI();
            if (eURI != null) {
                java.net.URI sourceURI = null;
                try {
                    if (eURI.isFile()) {
                        sourceURI = new java.net.URI(eURI.toString());
                        sourceURI = new java.net.URI(sourceURI.getScheme(), null,
                                sourceURI.getPath().substring(0, sourceURI.getPath().lastIndexOf("/")), null);
                    } else if (eURI.isPlatformResource()) {
                        IResource iFile = ResourcesPlugin.getWorkspace().getRoot().findMember(eURI.toPlatformString(true));
                        sourceURI = iFile != null ? iFile.getRawLocation().toFile().getParentFile().toURI() : null; 
                    }
                    if (sourceURI != null) {
                        return sourceURI.resolve(path);
                    }
                } catch (Exception e) {
                    // nothing
                }
            }
        }
        // fail
        return null;
    }

    /**
     * Recursively copies the contents of the given 'src'
     * directory to 'dest'. Existing files of the destination
     * may be overwritten.
     *
     * @param src The source directory path.
     * @param dest The destination directory path.
     * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not be changed
     * @throws IOException if copy fails.
     */
    public static void copyDirectory(final Path src, final Path dest, final boolean skipIfUnchanged) throws IOException {
        try (Stream<Path> stream = Files.walk(src)) {
            stream.forEach(source -> {
                // Handling checked exceptions in lambda expressions is
                // hard. See
                // https://www.baeldung.com/java-lambda-exceptions#handling-checked-exceptions.
                // An alternative would be to create a custom Consumer interface and use that
                // here.
                if (Files.isRegularFile(source)) { // do not copy directories
                    try {
                        Path target = dest.resolve(src.relativize(source));
                        Files.createDirectories(target.getParent());
                        copyFile(source, target, skipIfUnchanged);
                    } catch (IOException e) {
                        throw new RuntimeIOException(e);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            });
        }
    }

    /**
     * Recursively copies the contents of the given 'src'
     * directory to 'dest'. Existing files of the destination
     * may be overwritten.
     *
     * @param src The source directory path.
     * @param dest The destination directory path.
     * @throws IOException if copy fails.
     */
    public static void copyDirectory(final Path src, final Path dest) throws IOException {
        copyDirectory(src, dest, false);
    }

    /**
     * Copy a given file from 'source' to 'destination'.
     *
     * This also creates new directories for any directories on the destination
     * path that do not yet exist.
     *
     * @param source The source file path.
     * @param destination The destination file path.
     * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not be changed
     * @throws IOException if copy fails.
     */
    public static void copyFile(Path source, Path destination, boolean skipIfUnchanged)  throws IOException {
        BufferedInputStream stream = new BufferedInputStream(new FileInputStream(source.toFile()));
        try (stream) {
            copyInputStream(stream, destination, skipIfUnchanged);
        }
    }

    /**
     * Copy a given file from 'source' to 'destination'.
     *
     * This also creates new directories for any directories on the destination
     * path that do not yet exist.
     *
     * @param source The source file path.
     * @param destination The destination file path.
     * @throws IOException if copy fails.
     */
    public static void copyFile(Path source, Path destination)  throws IOException {
        copyFile(source, destination, false);
    }

    /**
     * Given a list of files or directories, attempt to find them based on the given generator
     * context, and copy then to the destination. Files are searched for in the file system first.
     * Files that cannot be found in the file system are looked for on the class path.
     *
     * @param filesOrDirectories The files or directories to copy.
     * @param destination The location to copy them to.
     * @param fileConfig The file configuration that specifies where the files must be found.
     * @param errorReporter An error reporter to report problems.
     */
    public static void copyFiles(
        List<String> filesOrDirectories,
        Path destination,
        FileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        for (String fileOrDirectory : filesOrDirectories) {
            var path = Paths.get(fileOrDirectory);
            var found = FileUtil.findInPackage(path, fileConfig);
            if (found != null) {
                try {
                    FileUtil.copyFromFileSystem(found, destination);
                    System.out.println("Copied '" + fileOrDirectory + "' from the file system.");
                } catch (IOException e) {
                    errorReporter.reportError(
                        "Unable to copy '" + fileOrDirectory + "' from the file system."
                    );
                }
            } else {
                try {
                    FileUtil.copyFromClassPath(
                        fileOrDirectory,
                        destination,
                        false
                    );
                } catch(IOException e) {
                    errorReporter.reportError(
                        "Unable to copy '" + fileOrDirectory + "' from the class path."
                    );
                }
                System.out.println("Copied '" + fileOrDirectory + "' from the class path.");
            }
        }
    }

    /**
     * If the source is a directory, then copy the contents of the directory to the destination.
     * If the source is a file, then copy the file to the destination.
     * @param source A file or directory to copy to the destination.
     * @param destination A directory to copy the file(s) at the source to.
     * @throws IOException
     */
    public static void copyFromFileSystem(Path source, Path destination) throws IOException {
        if (Files.isDirectory(source)) {
            copyDirectory(source, destination);
        } else if (Files.isRegularFile(source)) {
            copyFile(source, destination.resolve(source.getFileName())); // FIXME: should copyFile have the same API and have a directory as the second argument?
        } else {
            throw new IllegalArgumentException("Source is neither a directory nor a regular file.");
        }
    }
    /**
     * Copy a given input stream to a destination file.
     *
     * This also creates new directories for any directories on the destination
     * path that do not yet exist.
     *
     * @param source The source input stream.
     * @param destination The destination file path.
     * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not be changed
     * @throws IOException if copy fails.
     */
    private static void copyInputStream(InputStream source, Path destination, boolean skipIfUnchanged) throws IOException {
        // Read the stream once and keep a copy of all bytes. This is required as a stream cannot be read twice.
        final var bytes = source.readAllBytes();
        final var parent = destination.getParent();
        if (Files.isRegularFile(destination)) {
            if (skipIfUnchanged) {
                if (Arrays.equals(bytes, Files.readAllBytes(destination))) {
                    // Abort if the file contents are the same.
                    return;
                }
            } else {
                // Delete the file exists but the contents don't match.
                Files.delete(destination);
            }
        } else if (!Files.exists(parent)) {
            Files.createDirectories(parent);
        }

        Files.write(destination, bytes);
    }

    /**
     *  Lookup a directory in the classpath and copy its contents to a destination path
     *  in the filesystem.
     *
     *  This also creates new directories for any directories on the destination
     *  path that do not yet exist.
     *
     *  @param source The source directory as a path relative to the classpath.
     *  @param destination The file system path that the source directory is copied to.
     *  @param skipIfUnchanged If true, don't overwrite the file if its content would not be changed
     *  @throws IOException If the given source cannot be copied.
     */
    public static void copyFromClassPath(final String source, final Path destination, final boolean skipIfUnchanged) throws IOException {
        final URL resource = FileConfig.class.getResource(source);

        if (resource == null) {
            throw new TargetResourceNotFoundException(source);
        }

        final URLConnection connection = resource.openConnection();
        if (connection instanceof JarURLConnection) {
            boolean copiedFiles = copyFromJar((JarURLConnection) connection, destination, skipIfUnchanged);
            if (!copiedFiles) {
                throw new TargetResourceNotFoundException(source);
            }
        } else {
            try {
                Path path = Paths.get(FileLocator.toFileURL(resource).toURI());
                if (path.toFile().isDirectory()) {
                    copyDirectory(path, destination, skipIfUnchanged);
                } else {
                    copyFile(path, destination.resolve(path.getFileName()), skipIfUnchanged);
                }
            } catch(URISyntaxException e) {
                // This should never happen as toFileURL should always return a valid URL
                throw new IOException("Unexpected error while resolving " + source + " on the classpath");
            }
        }
    }

    /**
     * Return true if the given connection points to a file.
     * @param connection A connection to a JAR file.
     * @throws IOException If the connection is faulty.
     */
    private static boolean isFileInJar(JarURLConnection connection) throws IOException {
        return connection.getJarFile().stream().filter(
            it -> it.getName().equals(connection.getEntryName())
        ).findFirst().isPresent();
    }

    private static void copyFileFromJar(JarFile jar, String source, Path destination, boolean skipIfUnchanged) throws IOException {
        var entry = jar.getJarEntry(source);
        var filename = Paths.get(entry.getName()).getFileName();
        InputStream is = jar.getInputStream(entry);
        try (is) {
            copyInputStream(is, destination.resolve(filename), skipIfUnchanged);
        }
    }

    /**
     * Copy a directory from a jar to a destination path in the filesystem.
     *
     * This method should only be used in standalone mode (lfc).
     *
     * This also creates new directories for any directories on the destination
     * path that do not yet exist
     *
     * @param connection a URLConnection to the source directory within the jar
     * @param destination The file system path that the source directory is copied to.
     * @param skipIfUnchanged If true, don't overwrite the file if its content would not be changed
     * @return true if any files were copied
     * @throws IOException If the given source cannot be copied.
     */
    private static boolean copyFromJar(JarURLConnection connection, final Path destination, final boolean skipIfUnchanged) throws IOException {
        final JarFile jar = connection.getJarFile();
        final String source = connection.getEntryName();

        if (isFileInJar(connection)) {
            copyFileFromJar(jar, source, destination, skipIfUnchanged);
            return true;
        }

        boolean copiedFiles = false;

        // Iterate all entries in the jar file.
        for (Enumeration<JarEntry> e = jar.entries(); e.hasMoreElements(); ) {
            final JarEntry entry = e.nextElement();
            final String entryName = entry.getName();
            if (entryName.startsWith(source)) {
                String filename = entry.getName().substring(source.length() + 1);
                Path currentFile = destination.resolve(filename);
                if (entry.isDirectory()) {
                    Files.createDirectories(currentFile);
                } else {
                    InputStream is = jar.getInputStream(entry);
                    try (is) {
                        copyInputStream(is, currentFile, skipIfUnchanged);
                        copiedFiles = true;
                    }
                }
            }
        }
        return copiedFiles;
    }

    /**
     * Delete unused Files from Arduino-CLI based compilation.
     *
     * Arduino-CLI (the build system) uses lazy compilation (i.e. compiles every file recursively from 
     * a source directory). This does the work of CMake by explicitly deleting files that 
     * shouldn't get compiled by the CLI. Generally, we delete all CMake artifacts and multithreaded
     * support files (including semaphores and thread folders)
     *
     * @param dir The folder to search for folders and files to delete. 
     * @throws IOException If the given folder and unneeded files cannot be deleted.
     */
    public static void arduinoDeleteHelper(Path dir, boolean threadingOn) throws IOException {
        deleteDirectory(dir.resolve("core/federated")); // TODO: Add Federated Support to Arduino
        deleteDirectory(dir.resolve("include/core/federated")); // TODO: Add Federated Support to Arduino
        
        if (!threadingOn) {
            deleteDirectory(dir.resolve("core/threaded")); // No Threaded Support for Arduino
            deleteDirectory(dir.resolve("include/core/threaded")); // No Threaded Support for Arduino
            deleteDirectory(dir.resolve("core/platform/arduino_mbed")); // No Threaded Support for Arduino
        } 

        List<Path> allPaths = Files.walk(dir)
                    .sorted(Comparator.reverseOrder())
                    .collect(Collectors.toList());
        for (Path path : allPaths) {
            String toCheck = path.toString().toLowerCase();
            if (toCheck.contains("cmake")) {
                Files.delete(path);
            }
        }
    }

    /**
     * Helper function for getting the string representation of the relative path 
     * to take to get from one file (currPath) to get to the other (fileName).
     *
     * Generally, this is useful for converting includes to have relative pathing when 
     * you lack access to adding additional include paths when compiling.
     * 
     * @param fileName File to search for.
     * @param currPath The current path to the file whose include statements we are modifying.
     * @param fileStringToFilePath Mapping of File Names to their paths.
     */
    private static String fileNameMatchConverter(String fileName, Path currPath, Map<String, Path> fileStringToFilePath) 
        throws NullPointerException {
        // First get the child file
        int lastPath = fileName.lastIndexOf(File.separator);
        if (lastPath != -1){
            fileName = fileName.substring(lastPath+1);
        }
        Path p = fileStringToFilePath.get(fileName);
        if(p == null) {
            return "#include \"" + fileName + "\"";
        }
        String relativePath = currPath.getParent().relativize(p).toString();
        return "#include \"" + relativePath + "\"";
    }

    /**
     * Return true if the given path points to a C file, false otherwise.
     */
    public static boolean isCFile(Path path) {
        String fileName = path.getFileName().toString();
        return fileName.endsWith(".c") || fileName.endsWith(".cpp") || fileName.endsWith(".h");
    }

    /**
     * Convert all includes recursively inside files within a specified folder to relative links
     *
     * @param dir The folder to search for includes to change. 
     * @throws IOException If the given set of files cannot be relativized.
     */
    public static void relativeIncludeHelper(Path dir, Path includePath) throws IOException {
        System.out.println("Relativizing all includes in " + dir.toString());
        List<Path> includePaths = Files.walk(includePath)
            .filter(Files::isRegularFile)
            .filter(FileUtil::isCFile)
            .sorted(Comparator.reverseOrder())
            .collect(Collectors.toList());
        List<Path> srcPaths = Files.walk(dir)
            .filter(Files::isRegularFile)
            .filter(FileUtil::isCFile)
            .sorted(Comparator.reverseOrder())
            .collect(Collectors.toList());
        Map<String, Path> fileStringToFilePath = new HashMap<String, Path>();
        for (Path path : includePaths) {
            String fileName = path.getFileName().toString();
            if (path.getFileName().toString().contains("CMakeLists.txt")) continue;
            if (fileStringToFilePath.put(fileName, path) != null) {
                throw new IOException("Directory has different files with the same name. Cannot Relativize.");
            }
        }
        Pattern regexExpression = Pattern.compile("#include\s+[\"]([^\"]+)*[\"]");
        for (Path path : srcPaths) {
            String fileName = path.getFileName().toString();
            String fileContents = Files.readString(path);
            Matcher matcher = regexExpression.matcher(fileContents);
            int lastIndex = 0;
            StringBuilder output = new StringBuilder();
            while (matcher.find()) {
                output.append(fileContents, lastIndex, matcher.start())
                    .append(fileNameMatchConverter(matcher.group(1), path, fileStringToFilePath));
                lastIndex = matcher.end();
            }
            if (lastIndex < fileContents.length()) {
                output.append(fileContents, lastIndex, fileContents.length());
            }
            writeToFile(output.toString(), path);
        }
    }

    /**
     * Recursively delete a directory if it exists.
     *
     * @throws IOException If an I/O error occurs.
     */
    public static void deleteDirectory(Path dir) throws IOException {
        if (Files.isDirectory(dir)) {
            System.out.println("Cleaning " + dir);
            List<Path> pathsToDelete = Files.walk(dir)
                    .sorted(Comparator.reverseOrder())
                    .collect(Collectors.toList());
            for (Path path : pathsToDelete) {
                Files.deleteIfExists(path);
            }
        }
    }

    /**
     * Return an absolute path to the given file or directory if it can be found within the package.
     * Otherwise, return null.
     *
     * NOTE: If the given file or directory is given as an absolute path but cannot be found, it is
     * interpreted as a relative path with respect to the project root.
     *
     * @param fileOrDirectory The file or directory to look for.
     * @param fileConfig A file configuration that determines where the package is located.
     * @return An absolute path of the file or directory was found; null otherwise.
     */
    public static Path findInPackage(Path fileOrDirectory, FileConfig fileConfig) {
        if (fileOrDirectory.isAbsolute() && Files.exists(fileOrDirectory)) {
            return fileOrDirectory;
        } else {
            Path relPath;
            // Disregard root and interpret as relative path
            if (fileOrDirectory.isAbsolute()) {
                relPath = Paths.get(
                    String.valueOf(fileOrDirectory).replaceFirst(
                        String.valueOf(fileOrDirectory.getRoot()),
                        "")
                );
            } else {
                relPath = fileOrDirectory;
            }

            // Look relative to the source file and relative to the package root.
            var locations = List.of(fileConfig.srcPath, fileConfig.srcPkgPath);
            var found = locations.stream().filter(
                    loc -> Files.exists(loc.resolve(relPath))
                ).findFirst();
            if (found.isPresent()) {
                return found.get().resolve(relPath).toAbsolutePath();
            }
        }
        return null;
    }

    /**
     * Get the iResource corresponding to the provided resource if it can be
     * found.
     */
    public static IResource getIResource(Resource r) throws IOException {
        return getIResource(FileUtil.toPath(r).toFile().toURI());
    }

    /**
     * Get the specified path as an Eclipse IResource or null if it is not found.
     */
    public static IResource getIResource(Path path) {
        IResource ret = getIResource(path.toUri());
        if (ret != null) return ret;
        try {
            // Handle a bug that not everyone can reproduce in which a path originating in the Ecore model is a relative
            // path prefixed with a segment named "resource".
            return ResourcesPlugin.getWorkspace().getRoot().findMember(org.eclipse.core.runtime.Path.fromOSString(
                path.subpath(1, path.getNameCount()).toString()
            ));
        } catch (IllegalStateException e) {
            // We are outside of Eclipse.
        }
        return null;
    }

    /**
     * Get the specified uri as an Eclipse IResource or null if it is not found.
     *
     * Also returns null if this is not called from within a running Eclipse instance.
     *
     * @param uri A java.net.uri of the form "file://path".
     */
    public static IResource getIResource(java.net.URI uri) {
        // For some peculiar reason known only to Eclipse developers,
        // the resource cannot be used directly but has to be converted
        // a resource relative to the workspace root.
        try {
            IWorkspaceRoot workspaceRoot = ResourcesPlugin.getWorkspace().getRoot();

            IFile[] files = workspaceRoot.findFilesForLocationURI(uri);
            if (files != null && files.length > 0 && files[0] != null) {
                return files[0];
            }
        } catch (IllegalStateException e) {
            // We are outside of Eclipse.
        }
        return null;
    }

    /**
     * Write text to a file.
     * @param text The text to be written.
     * @param path The file to write the code to.
     * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not be changed
     */
    public static void writeToFile(String text, Path path, boolean skipIfUnchanged) throws IOException {
        Files.createDirectories(path.getParent());
        final byte[] bytes = text.getBytes();
        if (skipIfUnchanged && Files.isRegularFile(path)) {
            if (Arrays.equals(bytes, Files.readAllBytes(path))) {
                return;
            }
        }
        Files.write(path, text.getBytes());
    }

    /**
     * Write text to a file.
     * @param text The text to be written.
     * @param path The file to write the code to.
     */
    public static void writeToFile(String text, Path path) throws IOException {
        writeToFile(text, path, false);
    }

    /**
     * Write text to a file.
     * @param text The text to be written.
     * @param path The file to write the code to.
     */
    public static void writeToFile(CharSequence text, Path path) throws IOException {
        writeToFile(text.toString(), path, false);
    }

    public static void createDirectoryIfDoesNotExist(File dir) {
        if (!dir.exists()) dir.mkdirs();
    }

    /**
     * Return a list of files ending with "str".
     *
     * @param currentDir The current directory.
     * @param str The pattern to match against.
     */
    public static List<Path> globFilesEndsWith(Path currentDir, String str) {
        List<Path> matches = new ArrayList<>();
        File[] files = currentDir.toFile().listFiles();
        if (files != null) {
            for (File file : files) {
                if (file.isDirectory()) {
                    matches.addAll(globFilesEndsWith(file.toPath(), str));
                } else {
                    if (file.getName().endsWith(str)) {
                        matches.add(file.getAbsoluteFile().toPath());
                    }
                }
            }
        }
        return matches;
    }
}
