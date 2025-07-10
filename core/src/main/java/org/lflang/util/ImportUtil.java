package org.lflang.util;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.eclipse.emf.ecore.resource.Resource;

/**
 * Utility class for handling package-related URIs in the context of LF (Lingua Franca) libraries.
 * This class provides methods to build URIs for accessing library files based on their location in
 * a project structure, specifically targeting the "build/lfc_include" directory for library
 * inclusion.
 */
public class ImportUtil {

  /**
   * Builds a package URI based on the provided URI string and resource. It traverses upwards from
   * the current resource URI until it finds the "src" directory, then constructs the final URI
   * pointing to the library file within the "build/lfc_include" directory.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param resource The resource from which the URI resolution should start.
   * @return The constructed package URI as a string.
   * @throws IllegalArgumentException if the URI string does not contain both library and file
   *     names.
   */
  public static String buildPackageURI(String uriStr, Resource resource) {

    Path rootPath = FileUtil.toPath(resource);
    Path uriPath = Paths.get(uriStr.trim());

    if (uriPath.getNameCount() < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }

    // Initialize the path as the current directory
    Path finalPath = Paths.get("");

    // Traverse upwards until we reach the "src/" directory
    while (!rootPath.endsWith("src")) {
      rootPath = rootPath.getParent();
      if (rootPath == null) {
        throw new IllegalArgumentException("The 'src' directory was not found in the given path.");
      }
      finalPath = finalPath.resolve("..");
    }

    // Build the final path
    finalPath =
        finalPath
            .resolve("build")
            .resolve("lfc_include")
            .resolve(uriPath.getName(0))
            .resolve("src")
            .resolve("lib")
            .resolve(uriPath.getName(1));

    return finalPath.toString();
  }

  /**
   * Builds a package URI based on the provided URI string and source path. This method works
   * similarly to the `buildPackageURI`, but it accepts a direct source path instead of a resource.
   * It traverses upwards to locate the "src/" directory and then constructs the URI pointing to the
   * library file.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param root The root path from which the URI resolution should start.
   * @return The constructed package URI as a string.
   * @throws IllegalArgumentException if the URI string or source path is null, empty, or does not
   *     contain both the library name and file name.
   */
  public static Path buildPackageURIfromSrc(String uriStr, String root) {
    if (uriStr == null || root == null || uriStr.trim().isEmpty() || root.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    Path uriPath = Paths.get(uriStr.trim());

    if (uriPath.getNameCount() < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }

    // Use the src path to create a base path
    Path rootPath = Paths.get(root).toAbsolutePath();

    // Traverse upwards until we reach the "src/" directory
    while (!rootPath.endsWith("src")) {
      rootPath = rootPath.getParent();
      if (rootPath == null) {
        throw new IllegalArgumentException("The 'src' directory was not found in the given path.");
      }
    }

    Path finalPath =
        rootPath
            .resolveSibling("build")
            .resolve("lfc_include")
            .resolve(uriPath.getName(0)) // library name
            .resolve("src")
            .resolve("lib")
            .resolve(uriPath.getName(1)); // file name

    return finalPath;
  }
}
