package org.lflang.util;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.eclipse.emf.ecore.resource.Resource;

/**
 * Utility class for handling package-related URIs in the context of LF (Lingua Franca) libraries.
 * This class provides methods to build URIs for accessing library files based on their location in
 * a project structure, searching "build/lfc_include", "lf-packages", and the LF_PACKAGES environment
 * variable for library inclusion.
 * @ingroup Utilities
 */
public class ImportUtil {

  /**
   * Build a package URI based on the provided URI string and resource. This traverses upwards from
   * the current resource URI until it finds the nearest parent directory named "src", then searches
   * for the imported package under {@code <root>/build/lfc_include}, {@code <root>/lf-packages}, and
   * the {@code LF_PACKAGES} environment variable.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param resource The resource from which the URI resolution should start.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI string does not contain both library and file
   *     names, if no "src" directory is found, or if the package cannot be located.
   */
  public static String buildPackageURI(String uriStr, Resource resource) {
    Path uriPath = parseImportUri(uriStr);
    String packageName = uriPath.getName(0).toString();
    String fileName = uriPath.getName(1).toString();
    Path root = findProjectRoot(FileUtil.toPath(resource));
    return resolvePackageFile(root, packageName, fileName).toString();
  }

  /**
   * Builds a package URI based on the provided URI string and source path. This method works
   * similarly to the {@link #buildPackageURI}, but it accepts a direct source path instead of a
   * resource. It traverses upwards to locate the nearest parent directory named "src", then
   * searches for the imported package under {@code <root>/build/lfc_include}, {@code
   * <root>/lf-packages}, and the {@code LF_PACKAGES} environment variable.
   *
   * @param uriStr A string representing the URI of the file. It must contain both the library name
   *     and file name, separated by a '/'.
   * @param srcPath The path from which the URI resolution should start.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI string or source path is null, empty, or does not
   *     contain both the library name and file name, if no "src" directory is found, or if the
   *     package cannot be located.
   */
  public static Path buildPackageURIfromSrc(String uriStr, String srcPath) {
    if (uriStr == null || srcPath == null || uriStr.trim().isEmpty() || srcPath.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    Path uriPath = parseImportUri(uriStr);
    String packageName = uriPath.getName(0).toString();
    String fileName = uriPath.getName(1).toString();
    Path root = findProjectRoot(Paths.get(srcPath).toAbsolutePath());
    return resolvePackageFile(root, packageName, fileName);
  }

  private static Path parseImportUri(String uriStr) {
    Path uriPath = Paths.get(uriStr.trim());
    if (uriPath.getNameCount() < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }
    return uriPath;
  }

  private static Path findProjectRoot(Path startPath) {
    Path srcPath = startPath;
    while (!srcPath.endsWith("src")) {
      srcPath = srcPath.getParent();
      if (srcPath == null) {
        throw new IllegalArgumentException("The 'src' directory was not found in the given path.");
      }
    }
    return srcPath.getParent();
  }

  private static Path resolvePackageFile(Path root, String packageName, String fileName) {
    Path buildLfIncludePackage = root.resolve("build").resolve("lfc_include").resolve(packageName);
    if (Files.exists(buildLfIncludePackage)) {
      return buildLfIncludePackage.resolve("src").resolve("lib").resolve(fileName);
    }

    Path lfModulesPackage = root.resolve("lf-packages").resolve(packageName);
    if (Files.exists(lfModulesPackage)) {
      return lfModulesPackage.resolve("src").resolve("lib").resolve(fileName);
    }

    String lfModulesEnv = System.getenv("LF_PACKAGES");
    if (lfModulesEnv != null && !lfModulesEnv.isEmpty()) {
      Path envLfModulesPackage = Paths.get(lfModulesEnv).resolve(packageName);
      if (Files.exists(envLfModulesPackage)) {
        return envLfModulesPackage.resolve("src").resolve("lib").resolve(fileName);
      }
    }

    StringBuilder searchedLocations = new StringBuilder();
    searchedLocations.append(buildLfIncludePackage);
    searchedLocations.append(", ").append(lfModulesPackage);
    if (lfModulesEnv != null && !lfModulesEnv.isEmpty()) {
      searchedLocations.append(", ").append(Paths.get(lfModulesEnv).resolve(packageName));
    }

    throw new IllegalArgumentException(
        "Could not find package '"
            + packageName
            + "' for import '"
            + packageName
            + "/"
            + fileName
            + "'. Searched: "
            + searchedLocations
            + ".");
  }
}
