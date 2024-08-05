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
import org.lflang.FileConfig;
import org.lflang.MessageReporter;

public class FileUtil {

  /**
   * Return the name of the file excluding its file extension.
   *
   * @param file A Path object
   * @return The name of the file excluding its file extension.
   */
  public static String nameWithoutExtension(Path file) {
    String name = file.getFileName().toString();
    int idx = name.lastIndexOf('.');
    return idx < 0 ? name : name.substring(0, idx);
  }

  /**
   * Return the name of the file associated with the given resource, excluding its file extension.
   *
   * @param r Any {@code Resource}.
   * @return The name of the file associated with the given resource, excluding its file extension.
   * @throws IllegalArgumentException If the resource has an invalid URI.
   */
  public static String nameWithoutExtension(Resource r) {
    return nameWithoutExtension(toPath(r));
  }

  /**
   * Return a java.nio.Path object corresponding to the given URI.
   *
   * @throws IllegalArgumentException If the given URI is invalid.
   */
  public static Path toPath(URI uri) {
    return Paths.get(toIPath(uri).toFile().getAbsolutePath());
  }

  /**
   * Return a java.nio.Path object corresponding to the given Resource.
   *
   * @throws IllegalArgumentException If the given resource has an invalid URI.
   */
  public static Path toPath(Resource resource) {
    return toPath(resource.getURI());
  }

  /**
   * Return an org.eclipse.core.runtime.Path object corresponding to the given URI.
   *
   * @throws IllegalArgumentException If the given URI is invalid.
   */
  public static IPath toIPath(URI uri) {
    if (uri.isPlatform()) {
      IPath path = new org.eclipse.core.runtime.Path(uri.toPlatformString(true));
      if (path.segmentCount() == 1) {
        return ResourcesPlugin.getWorkspace()
            .getRoot()
            .getProject(path.lastSegment())
            .getLocation();
      } else {
        return ResourcesPlugin.getWorkspace().getRoot().getFile(path).getLocation();
      }
    } else if (uri.isFile()) {
      return new org.eclipse.core.runtime.Path(uri.toFileString());
    } else {
      throw new IllegalArgumentException("Unrecognized file protocol in URI " + uri);
    }
  }

  /**
   * Convert a given path to a unix-style string.
   *
   * <p>This ensures that '/' is used instead of '\' as file separator.
   */
  public static String toUnixString(Path path) {
    return path.toString().replace('\\', '/');
  }

  /**
   * Parse the string as file location and return it as URI. Supports URIs, plain file paths, and
   * paths relative to a model.
   *
   * @param path the file location as string.
   * @param resource the model resource this file should be resolved relatively. May be null.
   * @return the (Java) URI or null if no file can be located.
   */
  public static java.net.URI locateFile(String path, Resource resource) {
    // Check if path is URL
    try {
      var uri = new java.net.URI(path);
      if (uri.getScheme() != null) { // check if path was meant to be a URI
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
            sourceURI =
                new java.net.URI(
                    sourceURI.getScheme(),
                    null,
                    sourceURI.getPath().substring(0, sourceURI.getPath().lastIndexOf("/")),
                    null);
          } else if (eURI.isPlatformResource()) {
            IResource iFile =
                ResourcesPlugin.getWorkspace().getRoot().findMember(eURI.toPlatformString(true));
            sourceURI =
                iFile != null ? iFile.getRawLocation().toFile().getParentFile().toURI() : null;
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
   * Recursively copy the contents of the given source directory into the given destination
   * directory. Existing files of the destination may be overwritten.
   *
   * @param srcDir The source directory path.
   * @param dstDir The destination directory path.
   * @param skipIfUnchanged If true, don't overwrite anything in the destination if its content
   *     would not be changed.
   * @throws IOException If the operation fails.
   */
  public static void copyDirectoryContents(
      final Path srcDir, final Path dstDir, final boolean skipIfUnchanged) throws IOException {
    try (Stream<Path> stream = Files.walk(srcDir)) {
      stream.forEach(
          source -> {
            // Handling checked exceptions in lambda expressions is
            // hard. See
            // https://www.baeldung.com/java-lambda-exceptions#handling-checked-exceptions.
            // An alternative would be to create a custom Consumer interface and use that
            // here.
            if (Files.isRegularFile(source)) { // do not copy directories
              try {
                Path target = dstDir.resolve(srcDir.relativize(source));
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
   * Copy the given source directory into the given destination directory. For example, if the
   * source directory is {@code foo/bar} and the destination is {@code baz}, then copies of the
   * contents of {@code foo/bar} will be located in {@code baz/bar}.
   *
   * @param srcDir The source directory path.
   * @param dstDir The destination directory path.
   * @param skipIfUnchanged If true, don't overwrite anything in the destination if its content
   *     would not be changed.
   * @throws IOException If the operation fails.
   */
  public static void copyDirectory(
      final Path srcDir, final Path dstDir, final boolean skipIfUnchanged) throws IOException {
    copyDirectoryContents(srcDir, dstDir.resolve(srcDir.getFileName()), skipIfUnchanged);
  }

  /**
   * Recursively copy the contents of the given source directory into the given destination
   * directory. Existing files of the destination may be overwritten.
   *
   * @param srcDir The directory to copy files from.
   * @param dstDir The directory to copy files to.
   * @throws IOException if copy fails.
   */
  public static void copyDirectoryContents(final Path srcDir, final Path dstDir)
      throws IOException {
    copyDirectoryContents(srcDir, dstDir, false);
  }

  /**
   * Copy a given source file to a given destination file.
   *
   * <p>This also creates new directories on the path to {@code dstFile} that do not yet exist.
   *
   * @param srcFile The source file path.
   * @param dstFile The destination file path.
   * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not
   *     be changed.
   * @throws IOException If the operation fails.
   */
  public static void copyFile(Path srcFile, Path dstFile, boolean skipIfUnchanged)
      throws IOException {
    BufferedInputStream stream = new BufferedInputStream(new FileInputStream(srcFile.toFile()));
    try (stream) {
      copyInputStream(stream, dstFile, skipIfUnchanged);
    }
  }

  /**
   * Copy a given source file to a given destination file.
   *
   * <p>This also creates new directories for any directories on the path to {@code dstFile} that do
   * not yet exist.
   *
   * @param srcFile The source file path.
   * @param dstFile The destination file path.
   * @throws IOException if copy fails.
   */
  public static void copyFile(Path srcFile, Path dstFile) throws IOException {
    copyFile(srcFile, dstFile, false);
  }

  /**
   * Find the given {@code file} in the package and return the path to the file that was found; null
   * if it was not found.
   *
   * @param file The file to look for.
   * @param dstDir The directory to copy it to.
   * @param fileConfig The file configuration that specifies where look for the file.
   * @return The path to the file that was found, or null if it was not found.
   */
  public static Path findAndCopyFile(String file, Path dstDir, FileConfig fileConfig) {
    var path = Paths.get(file);
    var found = FileUtil.findInPackage(path, fileConfig);
    if (found != null) {
      try {
        FileUtil.copyFile(found, dstDir.resolve(path.getFileName()));
        return found;
      } catch (IOException e) {
        return null;
      }
    } else {
      return null;
    }
  }

  /**
   * Given a list of files or directories, attempt to find each entry based on the given generator
   * context and copy it to the destination directory. Entries are searched for in the file system
   * first, relative to the source file and relative to the package root. Entries that cannot be
   * found in the file system are looked for on the class path.
   *
   * <p>If {@code contentsOnly} is true, then for each entry that is a directory, only its contents
   * are copied, not the directory itself. For example, if the entry is a directory {@code foo/bar}
   * and the destination is {@code baz}, then copies of the contents of {@code foo/bar} will be
   * located directly in {@code baz}. If {@code contentsOnly} is false, then copies of the contents
   * of {@code foo/bar} will be located in {@code baz/bar}.
   *
   * @param entries The files or directories to copy from.
   * @param dstDir The location to copy the files to.
   * @param fileConfig The file configuration that specifies where the find entries the given
   *     entries.
   * @param messageReporter An error reporter to report problems.
   */
  public static void copyFilesOrDirectories(
      List<String> entries,
      Path dstDir,
      FileConfig fileConfig,
      MessageReporter messageReporter,
      boolean fileEntriesOnly) {
    for (String fileOrDirectory : entries) {
      var path = Paths.get(fileOrDirectory);
      var found = FileUtil.findInPackage(path, fileConfig);
      if (found != null) {
        try {
          if (fileEntriesOnly) {
            FileUtil.copyFile(found, dstDir.resolve(path.getFileName()));
          } else {
            FileUtil.copyFromFileSystem(found, dstDir, false);
          }
          messageReporter.nowhere().info("Copied '" + fileOrDirectory + "' from the file system.");
        } catch (IOException e) {
          String message =
              "Unable to copy '"
                  + fileOrDirectory
                  + "' from the file system. Reason: "
                  + e.toString();
          messageReporter.nowhere().error(message);
        }
      } else {
        try {
          if (fileEntriesOnly) {
            copyFileFromClassPath(fileOrDirectory, dstDir, false);
          } else {
            FileUtil.copyFromClassPath(fileOrDirectory, dstDir, false, false);
            messageReporter.nowhere().info("Copied '" + fileOrDirectory + "' from the class path.");
          }
        } catch (IOException e) {
          String message =
              "Unable to copy '"
                  + fileOrDirectory
                  + "' from the class path. Reason: "
                  + e.toString();
          messageReporter.nowhere().error(message);
        }
      }
    }
  }

  /**
   * If the given {@code entry} is a file, then copy it into the destination. If the {@code entry}
   * is a directory and {@code contentsOnly} is true, then copy its contents to the destination
   * directory. If the {@code entry} is a directory and {@code contentsOnly} is true, then copy it
   * including its contents to the destination directory.
   *
   * @param entry A file or directory to copy to the destination directory.
   * @param dstDir A directory to copy the entry or its contents to.
   * @param contentsOnly If true and {@code entry} is a directory, then copy its contents but not
   *     the directory itself.
   * @throws IOException If the operation fails.
   */
  public static void copyFromFileSystem(Path entry, Path dstDir, boolean contentsOnly)
      throws IOException {
    if (Files.isDirectory(entry)) {
      if (contentsOnly) {
        copyDirectoryContents(entry, dstDir);
      } else {
        copyDirectory(entry, dstDir, false);
      }
    } else if (Files.isRegularFile(entry)) {
      FileUtil.copyFile(entry, dstDir.resolve(entry.getFileName()));
    } else {
      throw new IllegalArgumentException("Source is neither a directory nor a regular file.");
    }
  }

  /**
   * Copy a given input stream to a destination file.
   *
   * <p>This also creates new directories for any directories on the destination path that do not
   * yet exist.
   *
   * @param source The source input stream.
   * @param destination The destination file path.
   * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not
   *     be changed.
   * @throws IOException If the operation fails.
   */
  private static void copyInputStream(InputStream source, Path destination, boolean skipIfUnchanged)
      throws IOException {
    // Read the stream once and keep a copy of all bytes. This is required as a stream cannot be
    // read twice.
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
    } else if (Files.isDirectory(destination)) {
      deleteDirectory(destination);
    } else if (!Files.exists(parent)) {
      Files.createDirectories(parent);
    }

    Files.write(destination, bytes);
  }

  /**
   * Look up the given {@code entry} in the classpath. If it is found and is a file, copy it into
   * the destination directory. If the entry is not found or not a file, throw an exception.
   *
   * @param entry A file copy to the destination directory.
   * @param dstDir A directory to copy the entry to.
   * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not
   *     be changed.
   * @throws IOException If the operation failed.
   */
  public static void copyFileFromClassPath(
      final String entry, final Path dstDir, final boolean skipIfUnchanged) throws IOException {
    final URL resource = FileConfig.class.getResource(entry);

    if (resource == null) {
      throw new TargetResourceNotFoundException(entry);
    }

    final URLConnection connection = resource.openConnection();
    if (connection instanceof JarURLConnection) {
      if (!copyFileFromJar((JarURLConnection) connection, dstDir, skipIfUnchanged)) {
        throw new IOException("'" + entry + "' is not a file");
      }
    } else {
      try {
        Path path = Paths.get(FileLocator.toFileURL(resource).toURI());
        copyFile(path, dstDir.resolve(path.getFileName()), skipIfUnchanged);
      } catch (URISyntaxException e) {
        // This should never happen as toFileURL should always return a valid URL
        throw new IOException("Unexpected error while resolving " + entry + " on the classpath");
      }
    }
  }

  /**
   * Look up the given {@code entry} in the classpath. If it is a file, copy it into the destination
   * directory. If the {@code entry} is a directory and {@code contentsOnly} is true, then copy its
   * contents to the destination directory. If the {@code entry} is a directory and {@code
   * contentsOnly} is true, then copy it including its contents to the destination directory.
   *
   * <p>This also creates new directories for any directories on the destination path that do not
   * yet exist.
   *
   * @param entry The entry to be found on the class path and copied to the given destination.
   * @param dstDir The file system path that found files are to be copied to.
   * @param skipIfUnchanged If true, don't overwrite the file or directory if its content would not
   *     be changed
   * @param contentsOnly If true and the entry is a directory, then copy its contents but not the
   *     directory itself.
   * @throws IOException If the operation failed.
   */
  public static void copyFromClassPath(
      final String entry,
      final Path dstDir,
      final boolean skipIfUnchanged,
      final boolean contentsOnly)
      throws IOException {
    final URL resource = FileConfig.class.getResource(entry);

    if (resource == null) {
      throw new TargetResourceNotFoundException(entry);
    }

    final URLConnection connection = resource.openConnection();
    if (connection instanceof JarURLConnection) {
      boolean copiedFiles =
          copyFromJar((JarURLConnection) connection, dstDir, skipIfUnchanged, contentsOnly);
      if (!copiedFiles) {
        throw new TargetResourceNotFoundException(entry);
      }
    } else {
      try {
        Path path = Paths.get(FileLocator.toFileURL(resource).toURI());
        if (path.toFile().isDirectory()) {
          if (contentsOnly) {
            copyDirectoryContents(path, dstDir, skipIfUnchanged);
          } else {
            copyDirectory(path, dstDir, skipIfUnchanged);
          }

        } else {
          copyFile(path, dstDir.resolve(path.getFileName()), skipIfUnchanged);
        }
      } catch (URISyntaxException e) {
        // This should never happen as toFileURL should always return a valid URL
        throw new IOException("Unexpected error while resolving " + entry + " on the classpath");
      }
    }
  }

  /**
   * Return true if the given connection points to a file.
   *
   * @param connection A connection to a JAR file.
   * @throws IOException If the connection is faulty.
   */
  private static boolean isFileInJar(JarURLConnection connection) throws IOException {
    return connection.getJarFile().stream()
        .anyMatch(it -> it.getName().equals(connection.getEntryName()));
  }

  /**
   * Given a JAR file and a {@code srcFile} entry, copy it into the given destination directory.
   *
   * @param jar The JAR file from which to copy {@code srcFile}.
   * @param srcFile The source file to copy from the given {@code jar}.
   * @param dstDir The directory to top the source file into.
   * @param skipIfUnchanged If true, don't overwrite the destination file if its content would * not
   *     be changed.
   * @throws IOException If the operation fails.
   */
  private static void copyFileFromJar(
      JarFile jar, String srcFile, Path dstDir, boolean skipIfUnchanged) throws IOException {
    var entry = jar.getJarEntry(srcFile);
    var filename = Paths.get(entry.getName()).getFileName();
    InputStream is = jar.getInputStream(entry);
    try (is) {
      copyInputStream(is, dstDir.resolve(filename), skipIfUnchanged);
    }
  }

  /**
   * Copy the contents from an entry in a JAR to destination directory in the filesystem. The entry
   * may be a file, in which case it will be copied under the same name into the destination
   * directory. If the entry is a directory, then if {@code contentsOnly} is true, only the contents
   * of the directory will be copied into the destination directory (not the directory itself). A
   * directory will be copied as a whole, including its contents, if {@code contentsOnly} is false.
   *
   * <p>This method should only be used in standalone mode (lfc).
   *
   * <p>This also creates new directories for any directories on the destination path that do not
   * yet exist.
   *
   * @param connection a URLConnection to the source entry within the jar
   * @param dstDir The file system path that entries are copied to.
   * @param skipIfUnchanged If true, don't overwrite the file if its content would not be changed.
   * @param contentsOnly If true, and the connection points to a directory, copy its contents only
   *     (not the directory itself).
   * @return true if any files were copied
   * @throws IOException If the given source cannot be copied.
   */
  private static boolean copyFromJar(
      JarURLConnection connection,
      Path dstDir,
      final boolean skipIfUnchanged,
      final boolean contentsOnly)
      throws IOException {

    if (copyFileFromJar(connection, dstDir, skipIfUnchanged)) {
      return true;
    }
    return copyDirectoryFromJar(connection, dstDir, skipIfUnchanged, contentsOnly);
  }

  /**
   * Given a connection to a JAR file that points to an entry that is a directory, recursively copy
   * all entries located in that directory into the given {@code dstDir}.
   *
   * <p>If {@code contentsOnly} is true, only the contents of the directory will be copied into the
   * destination directory (not the directory itself). The directory will be copied as a whole,
   * including its contents, if {@code contentsOnly} is false.
   *
   * @param connection A connection to a JAR file that points to a directory entry.
   * @param dstDir The destination directory to copy the matching entries to.
   * @param skipIfUnchanged
   * @param contentsOnly
   * @return
   * @throws IOException
   */
  private static boolean copyDirectoryFromJar(
      JarURLConnection connection,
      Path dstDir,
      final boolean skipIfUnchanged,
      final boolean contentsOnly)
      throws IOException {
    final JarFile jar = connection.getJarFile();
    final String source = connection.getEntryName();

    boolean copiedFiles = false;
    if (!contentsOnly) {
      dstDir = dstDir.resolve(Paths.get(source).getFileName());
    }
    // Iterate all entries in the jar file.
    for (Enumeration<JarEntry> e = jar.entries(); e.hasMoreElements(); ) {
      final JarEntry entry = e.nextElement();
      final String entryName = entry.getName();
      if (entryName.startsWith(source)) {
        String filename = entry.getName().substring(source.length() + 1);
        Path currentFile = dstDir.resolve(filename);
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
   * Given a connection to a JAR file that points to an entry that is a file, copy the file into the
   * given {@code dstDir}.
   *
   * @param connection A connection to a JAR file that points to a directory entry.
   * @param dstDir The destination directory to copy the file to.
   * @param skipIfUnchanged
   * @return {@code true} the connection entry is a file, and it was copied successfully; {@code
   *     false} if the connection entry is not a file and the copy operation was aborted.
   * @throws IOException If the operation failed.
   */
  private static boolean copyFileFromJar(
      JarURLConnection connection, Path dstDir, final boolean skipIfUnchanged) throws IOException {
    final JarFile jar = connection.getJarFile();
    final String source = connection.getEntryName();

    if (!isFileInJar(connection)) {
      return false;
    }
    copyFileFromJar(jar, source, dstDir, skipIfUnchanged);

    return true;
  }

  /**
   * Delete unused Files from Arduino-CLI based compilation.
   *
   * <p>Arduino-CLI (the build system) uses lazy compilation (i.e. compiles every file recursively
   * from a source directory). This does the work of CMake by explicitly deleting files that
   * shouldn't get compiled by the CLI. Generally, we delete all CMake artifacts and multithreaded
   * support files (including semaphores and thread folders)
   *
   * @param srcGenPath The folder to search for folders and files to delete.
   * @throws IOException If the given folder and unneeded files cannot be deleted.
   */
  public static void arduinoDeleteHelper(Path srcGenPath, boolean threadingOn) throws IOException {
    // Remove all threading-related sources and headers unless we are targeting the threaded
    // runtime.
    if (!threadingOn) {
      deleteDirectory(srcGenPath.resolve("src/core/threaded"));
      deleteDirectory(srcGenPath.resolve("include/core/threaded"));
      deleteDirectory(srcGenPath.resolve("src/core/platform/arduino_mbed"));
    }
    deleteDirectory(srcGenPath.resolve("src").resolve("trace"));
    // Delete all the federated headers
    deleteDirectory(srcGenPath.resolve("include/core/federated"));
    // arduino-cli needs all headers to be under a "include" directory.
    // Create one for the RTI headers
    srcGenPath.resolve("include/core/federated/RTI").toFile().mkdirs();
    // Copy the necessary RTI headers to the newly created directory
    copyFile(
        srcGenPath.resolve("src/core/federated/RTI/rti_local.h"),
        srcGenPath.resolve("include/core/federated/RTI/rti_local.h"));
    copyFile(
        srcGenPath.resolve("src/core/federated/RTI/rti_common.h"),
        srcGenPath.resolve("include/core/federated/RTI/rti_common.h"));
    // Delete the remaining federated sources and headers
    deleteDirectory(srcGenPath.resolve("src/core/federated"));

    List<Path> allPaths = Files.walk(srcGenPath).sorted(Comparator.reverseOrder()).toList();
    for (Path path : allPaths) {
      String toCheck = path.toString().toLowerCase();
      if (toCheck.contains("cmake")) {
        Files.delete(path);
      }
    }
  }

  /**
   * Helper function for getting the string representation of the relative path to take to get from
   * one file (currPath) to get to the other (fileName).
   *
   * <p>Generally, this is useful for converting includes to have relative pathing when you lack
   * access to adding additional include paths when compiling.
   *
   * @param fileName File to search for.
   * @param currPath The current path to the file whose include statements we are modifying.
   * @param fileStringToFilePath Mapping of File Names to their paths.
   */
  private static String fileNameMatchConverter(
      String fileName, Path currPath, Map<String, Path> fileStringToFilePath)
      throws NullPointerException {
    // First get the child file
    int lastPath = fileName.lastIndexOf(File.separator);
    if (lastPath != -1) {
      fileName = fileName.substring(lastPath + 1);
    }
    Path p = fileStringToFilePath.get(fileName);
    if (p == null) {
      return "#include \"" + fileName + "\"";
    }
    String relativePath = currPath.getParent().relativize(p).toString();
    return "#include \"" + relativePath + "\"";
  }

  /** Return true if the given path points to a C file, false otherwise. */
  public static boolean isCFile(Path path) {
    String fileName = path.getFileName().toString();
    return fileName.endsWith(".c") || fileName.endsWith(".cpp") || fileName.endsWith(".h");
  }

  /**
   * Convert all includes recursively inside files within a specified folder to relative links
   *
   * @param dir The folder to search for includes to change.
   * @param messageReporter Error reporter
   * @throws IOException If the given set of files cannot be relativized.
   */
  public static void relativeIncludeHelper(
      Path dir, Path includePath, MessageReporter messageReporter) throws IOException {
    messageReporter.nowhere().info("Relativizing all includes in " + dir.toString());
    List<Path> includePaths =
        Files.walk(includePath)
            .filter(Files::isRegularFile)
            .filter(FileUtil::isCFile)
            .sorted(Comparator.reverseOrder())
            .toList();
    List<Path> srcPaths =
        Files.walk(dir)
            .filter(Files::isRegularFile)
            .filter(FileUtil::isCFile)
            .sorted(Comparator.reverseOrder())
            .toList();
    Map<String, Path> fileStringToFilePath = new HashMap<String, Path>();
    for (Path path : includePaths) {
      String fileName = path.getFileName().toString();
      if (path.getFileName().toString().contains("CMakeLists.txt")) continue;
      if (fileStringToFilePath.put(fileName, path) != null) {
        throw new IOException(
            String.format(
                "Directory has different files with the same name (%s). Cannot relativize.",
                fileName));
      }
    }
    Pattern regexExpression = Pattern.compile("#include\s+[\"]([^\"]+)*[\"]");
    for (Path path : srcPaths) {
      String fileContents = Files.readString(path);
      Matcher matcher = regexExpression.matcher(fileContents);
      int lastIndex = 0;
      StringBuilder output = new StringBuilder();
      while (matcher.find()) {
        output
            .append(fileContents, lastIndex, matcher.start())
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
   * Delete the given file or directory if it exists. If {@code fileOrDirectory} is a directory,
   * deletion is recursive.
   *
   * @param fileOrDirectory The file or directory to delete.
   * @throws IOException If the operation failed.
   */
  public static void delete(Path fileOrDirectory) throws IOException {
    if (Files.isRegularFile(fileOrDirectory)) {
      Files.deleteIfExists(fileOrDirectory);
    }
    if (Files.isDirectory(fileOrDirectory)) {
      deleteDirectory(fileOrDirectory);
    }
  }

  /**
   * Recursively delete a directory if it exists.
   *
   * @throws IOException If an I/O error occurs.
   */
  public static void deleteDirectory(Path dir) throws IOException {
    if (Files.isDirectory(dir)) {
      // fixme system.out
      System.out.println("Cleaning " + dir);
      List<Path> pathsToDelete = Files.walk(dir).sorted(Comparator.reverseOrder()).toList();
      for (Path path : pathsToDelete) {
        Files.deleteIfExists(path);
      }
    }
  }

  /**
   * Return an absolute path to the given file or directory if it can be found within the package.
   * Otherwise, return null.
   *
   * <p>NOTE: If the given file or directory is given as an absolute path but cannot be found, it is
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
        relPath =
            Paths.get(
                String.valueOf(fileOrDirectory)
                    .replaceFirst(String.valueOf(fileOrDirectory.getRoot()), ""));
      } else {
        relPath = fileOrDirectory;
      }

      // Look relative to the source file and relative to the package root.
      var locations = List.of(fileConfig.srcPath, fileConfig.srcPkgPath);
      var found = locations.stream().filter(loc -> Files.exists(loc.resolve(relPath))).findFirst();
      if (found.isPresent()) {
        return found.get().resolve(relPath).toAbsolutePath();
      }
    }
    return null;
  }

  public static Path getRelativePath(Resource source, Resource target) {
    return FileUtil.toPath(source.getURI())
        .getParent()
        .relativize(FileUtil.toPath(target.getURI()).getParent());
  }

  /** Get the iResource corresponding to the provided resource if it can be found. */
  public static IResource getIResource(Resource r) {
    return getIResource(FileUtil.toPath(r).toFile().toURI());
  }

  /** Get the specified path as an Eclipse IResource or null if it is not found. */
  public static IResource getIResource(Path path) {
    IResource ret = getIResource(path.toUri());
    if (ret != null) return ret;
    try {
      // Handle a bug that not everyone can reproduce in which a path originating in the Ecore model
      // is a relative
      // path prefixed with a segment named "resource".
      return ResourcesPlugin.getWorkspace()
          .getRoot()
          .findMember(
              org.eclipse.core.runtime.Path.fromOSString(
                  path.subpath(1, path.getNameCount()).toString()));
    } catch (IllegalStateException e) {
      // We are outside of Eclipse.
    }
    return null;
  }

  /**
   * Get the specified uri as an Eclipse IResource or null if it is not found.
   *
   * <p>Also returns null if this is not called from within a running Eclipse instance.
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
   * Check if the content of a file is equal to a given string.
   *
   * @param text The text to compare with.
   * @param path The file to compare with.
   * @return true, if the given text is identical to the file content.
   */
  public static boolean isSame(String text, Path path) throws IOException {
    if (Files.isRegularFile(path)) {
      final byte[] bytes = text.getBytes();
      return Arrays.equals(bytes, Files.readAllBytes(path));
    }
    return false;
  }

  /**
   * Write text to a file.
   *
   * @param text The text to be written.
   * @param path The file to write the code to.
   * @param skipIfUnchanged If true, don't overwrite the destination file if its content would not
   *     be changed
   */
  public static void writeToFile(String text, Path path, boolean skipIfUnchanged)
      throws IOException {
    if (!skipIfUnchanged || !isSame(text, path)) {
      Files.createDirectories(path.getParent());
      Files.write(path, text.getBytes());
    }
  }

  /**
   * Write text to a file.
   *
   * @param text The text to be written.
   * @param path The file to write the code to.
   */
  public static void writeToFile(String text, Path path) throws IOException {
    writeToFile(text, path, false);
  }

  /**
   * Write text to a file.
   *
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
