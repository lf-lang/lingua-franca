package org.lflang.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.LfPackage;

/**
 * Utility class for handling package-related URIs in the context of LF (Lingua Franca) libraries.
 * This class provides methods to build URIs for accessing library files based on their location in
 * a project structure, searching "build/lfc_include", "lf-packages", and the LF_PACKAGES environment
 * variable for library inclusion.
 *
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
    if (uriPath.getNameCount() < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }
    String packageName = uriPath.getName(0).toString();
    String fileName = uriPath.getName(1).toString();
    Path root = findProjectRoot(FileUtil.toPath(resource));
    return resolvePackageFile(root, packageName, fileName).toString();
  }

  /**
   * Like {@link #buildPackageURI(String, Resource)}, but allows the imported file name to be
   * omitted. If {@code uriStr} has only one segment, then {@code defaultFileName} is used as the
   * library file name inside {@code src/lib/}.
   *
   * <p>This is used to support import statements of the form:
   *
   * <pre>{@code
   * import ReactorClassName from <packageName>
   * }</pre>
   *
   * @param uriStr The package import string (e.g., {@code packageName} or {@code
   *     packageName/file.lf}).
   * @param resource The resource from which the URI resolution should start.
   * @param defaultFileName The file name to use if {@code uriStr} omits it.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI string does not contain a package name, if no "src"
   *     directory is found, or if the package cannot be located.
   */
  public static String buildPackageURI(String uriStr, Resource resource, String defaultFileName) {
    Path uriPath = parseImportUri(uriStr);
    if (uriPath.getNameCount() >= 2) {
      // Explicit file name.
      return buildPackageURI(uriStr, resource);
    }

    if (defaultFileName == null || defaultFileName.isBlank()) {
      throw new IllegalArgumentException("Missing library file name for import '" + uriStr + "'.");
    }

    String packageName = uriPath.getName(0).toString();
    Path root = findProjectRoot(FileUtil.toPath(resource));
    return resolvePackageFile(root, packageName, defaultFileName).toString();
  }

  /**
   * Resolve the library file path(s) for a package import.
   *
   * <p>If {@code uriStr} contains an explicit file ({@code package/file.lf}), a single path is
   * returned. If only a package name is given, then:
   *
   * <ul>
   *   <li>if {@code defaultFileName} is provided, that file under {@code src/lib/} is returned;
   *   <li>otherwise all {@code .lf} files under the package's {@code src/lib/} directory are
   *       returned.
   * </ul>
   *
   * The latter case supports {@code import ReactorClassName from <packageName>} during linking,
   * when the reactor class name may not yet be readable from the AST.
   */
  public static List<String> buildPackageURIs(
      String uriStr, Resource resource, String defaultFileName) {
    Path uriPath = parseImportUri(uriStr);
    Path root = findProjectRoot(FileUtil.toPath(resource));
    String packageName = uriPath.getName(0).toString();

    if (uriPath.getNameCount() >= 2) {
      return List.of(
          resolvePackageFile(root, packageName, uriPath.getName(1).toString()).toString());
    }

    if (defaultFileName != null && !defaultFileName.isBlank()) {
      return List.of(resolvePackageFile(root, packageName, defaultFileName).toString());
    }

    Path packageDir = findPackageDirectory(root, packageName);
    Path libDir = packageDir.resolve("src").resolve("lib");
    if (!Files.isDirectory(libDir)) {
      throw new IllegalArgumentException(
          "Package '" + packageName + "' has no src/lib directory at " + libDir + ".");
    }

    try (Stream<Path> files = Files.list(libDir)) {
      List<String> uris =
          files
              .filter(Files::isRegularFile)
              .filter(path -> path.getFileName().toString().endsWith(".lf"))
              .map(Path::toString)
              .sorted()
              .toList();
      if (uris.isEmpty()) {
        throw new IllegalArgumentException(
            "Package '" + packageName + "' has no .lf files in " + libDir + ".");
      }
      return uris;
    } catch (IOException e) {
      throw new IllegalArgumentException(
          "Failed to list library files for package '" + packageName + "' in " + libDir + ".", e);
    }
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
    if (uriPath.getNameCount() < 2) {
      throw new IllegalArgumentException("URI must contain both library name and file name.");
    }
    String packageName = uriPath.getName(0).toString();
    String fileName = uriPath.getName(1).toString();
    Path root = findProjectRoot(Paths.get(srcPath).toAbsolutePath());
    return resolvePackageFile(root, packageName, fileName);
  }

  /**
   * Like {@link #buildPackageURIfromSrc(String, String)}, but allows the imported file name to be
   * omitted. If {@code uriStr} has only one segment, then {@code defaultFileName} is used as the
   * library file name inside {@code src/lib/}.
   *
   * @param uriStr The package import string (e.g., {@code packageName} or {@code
   *     packageName/file.lf}).
   * @param srcPath The path from which the URI resolution should start.
   * @param defaultFileName The file name to use if {@code uriStr} omits it.
   * @return The path to the imported library file.
   */
  public static Path buildPackageURIfromSrc(String uriStr, String srcPath, String defaultFileName) {
    if (uriStr == null || srcPath == null || uriStr.trim().isEmpty() || srcPath.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    Path uriPath = parseImportUri(uriStr);
    if (uriPath.getNameCount() >= 2) {
      return buildPackageURIfromSrc(uriStr, srcPath);
    }

    if (defaultFileName == null || defaultFileName.isBlank()) {
      throw new IllegalArgumentException("Missing library file name for import '" + uriStr + "'.");
    }

    String packageName = uriPath.getName(0).toString();
    Path root = findProjectRoot(Paths.get(srcPath).toAbsolutePath());
    return resolvePackageFile(root, packageName, defaultFileName);
  }

  /**
   * Return the reactor class name referenced by an ImportedReactor.
   *
   * <p>During linking, {@code getReactorClass()} may be an unresolved proxy or null. Accessing the
   * linked object (or its name) can re-enter linking, so this method never resolves the
   * cross-reference. It prefers the textual identifier from the parse tree.
   *
   * @param importedReactor The imported reactor AST node.
   * @return The reactor class name from the import statement, or {@code null} if unavailable.
   */
  public static String getImportedReactorClassName(ImportedReactor importedReactor) {
    if (importedReactor == null) {
      return null;
    }

    // Prefer the parse-tree text. Do not call getReactorClass()/getName() here: that can re-enter
    // linking while the scope for this cross-reference is still being computed.
    var featureNodes =
        NodeModelUtils.findNodesForFeature(
            importedReactor, LfPackage.Literals.IMPORTED_REACTOR__REACTOR_CLASS);
    if (!featureNodes.isEmpty()) {
      String text = featureNodes.stream().map(INode::getText).reduce("", String::concat).trim();
      if (!text.isEmpty()) {
        return text;
      }
    }

    // Fallback: first non-hidden leaf under the ImportedReactor node is the class identifier.
    var node = NodeModelUtils.getNode(importedReactor);
    if (node != null) {
      for (ILeafNode leaf : node.getLeafNodes()) {
        if (leaf.isHidden()) {
          continue;
        }
        String text = leaf.getText().trim();
        if (text.isEmpty() || text.equals("as") || text.equals(",")) {
          continue;
        }
        return text;
      }
    }

    // Only use a fully resolved reactor class as a last resort (e.g., after linking).
    Object unresolved =
        importedReactor.eGet(LfPackage.Literals.IMPORTED_REACTOR__REACTOR_CLASS, false);
    if (unresolved instanceof org.lflang.lf.Reactor reactor && !reactor.eIsProxy()) {
      String name = reactor.getName();
      if (name != null && !name.isBlank()) {
        return name;
      }
    }

    return null;
  }

  private static Path parseImportUri(String uriStr) {
    Path uriPath = Paths.get(uriStr.trim());
    if (uriPath.getNameCount() < 1) {
      throw new IllegalArgumentException("URI must contain a package name.");
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

  private static Path findPackageDirectory(Path root, String packageName) {
    List<Path> candidates = new ArrayList<>();
    candidates.add(root.resolve("build").resolve("lfc_include").resolve(packageName));
    candidates.add(root.resolve("lf-packages").resolve(packageName));

    String lfPackagesEnv = System.getenv("LF_PACKAGES");
    if (lfPackagesEnv != null && !lfPackagesEnv.isEmpty()) {
      candidates.add(Paths.get(lfPackagesEnv).resolve(packageName));
    }

    for (Path candidate : candidates) {
      if (Files.exists(candidate)) {
        return candidate;
      }
    }

    throw new IllegalArgumentException(
        "Could not find package '"
            + packageName
            + "'. Searched: "
            + candidates.stream().map(Path::toString).reduce((a, b) -> a + ", " + b).orElse("")
            + ".");
  }

  private static Path resolvePackageFile(Path root, String packageName, String fileName) {
    return findPackageDirectory(root, packageName).resolve("src").resolve("lib").resolve(fileName);
  }
}
