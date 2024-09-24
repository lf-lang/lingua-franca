package org.lflang.util;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;

/**
 * Utility class for handling package-related URIs in the context of LF (Lingua Franca) libraries.
 * This class provides methods to build URIs for accessing library files based on their location in
 * a project structure, specifically targeting the "target/lfc_include" directory for library
 * inclusion.
 */
public class ImportUtil {

  /**
   * Builds a package URI based on the provided URI string and resource. It traverses upwards from
   * the current resource URI until it finds the "src/" directory, then constructs the final URI
   * pointing to the library file within the "target/lfc_include" directory.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param resource The resource from which the URI resolution should start.
   * @return The constructed package URI as a string.
   * @throws IllegalArgumentException if the URI string does not contain both library and file
   *     names.
   */
  public static String buildPackageURI(String uriStr, Resource resource) {
    URI currentURI = URI.createURI(".");
    URI rootURI = currentURI.resolve(resource.getURI());
    StringBuilder pathBuilder = new StringBuilder();

    String[] uriParts = uriStr.split("/");

    if (uriParts.length < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }

    // Traverse upwards until we reach the "src/" directory
    while (!rootURI.toString().endsWith("src/")) {
      currentURI = URI.createURI("..");
      rootURI = currentURI.resolve(resource.getURI());
      pathBuilder.append("../");
    }

    pathBuilder
        .append("../target/lfc_include/")
        .append(uriParts[0])
        .append("/src/lib/")
        .append(uriParts[1]);

    return pathBuilder.toString();
  }

  /**
   * Builds a package URI based on the provided URI string and source path. This method works
   * similarly to the `buildPackageURI`, but it accepts a direct source path instead of a resource.
   * It traverses upwards to locate the "src/" directory and then constructs the URI pointing to the
   * library file.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param src The source path from which the URI resolution should start.
   * @return The constructed package URI as a string.
   * @throws IllegalArgumentException if the URI string or source path is null, empty, or does not
   *     contain both the library name and file name.
   */
  public static String buildPackageURIfromSrc(String uriStr, String src) {
    if (uriStr == null || src == null || uriStr.trim().isEmpty() || src.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    String[] uriParts = uriStr.trim().split("/");

    if (uriParts.length < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }

    // Use the src path to create a base path
    Path rootPath = Paths.get(src).toAbsolutePath();

    // Traverse upwards until we reach the "src/" directory
    while (!rootPath.endsWith("src")) {
      rootPath = rootPath.getParent();
      if (rootPath == null) {
        throw new IllegalArgumentException("The 'src' directory was not found in the given path.");
      }
    }

    Path finalPath =
        rootPath
            .resolveSibling("target/lfc_include/")
            .resolve(uriParts[0].trim()) // library name
            .resolve("src/lib")
            .resolve(uriParts[1].trim()); // file name

    return finalPath.toString();
  }
}
