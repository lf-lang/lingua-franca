package org.lflang.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
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
 * a project structure, searching {@code build/lfc_include}, {@code lf-packages}, and the {@code
 * LF_PACKAGES} environment variable for library inclusion.
 *
 * <p>Package imports use the form {@code <package/...>} and are resolved under the package's {@code
 * src/lib} directory. The last path segment may be a library file ({@code .lf} or {@code .ulf}). If
 * it is not, {@code ReactorClassName.lf} is used as the file name, which supports both {@code
 * import R from <package>} and imports that name a subdirectory such as {@code <package/subdir>}.
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
   * @param uriStr A package import path. The last segment must be a {@code .lf} or {@code .ulf}
   *     file (e.g., {@code package/file.lf} or {@code package/subdir/file.lf}).
   * @param resource The resource from which the URI resolution should start.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI does not end in a library file, if no "src"
   *     directory is found, or if the package cannot be located.
   */
  public static String buildPackageURI(String uriStr, Resource resource) {
    Path uriPath = parseImportUri(uriStr);
    if (!lastSegmentIsLibraryFile(uriPath)) {
      throw new IllegalArgumentException(
          "URI must end with a .lf or .ulf library file name: '" + uriStr + "'.");
    }
    Path root = findProjectRoot(FileUtil.toPath(resource));
    return toFileUriString(
        resolvePackageFile(root, uriPath.getName(0).toString(), relativeLibPath(uriPath, null)));
  }

  /**
   * Like {@link #buildPackageURI(String, Resource)}, but allows the imported file name to be
   * omitted. If the last path segment is not a {@code .lf}/{@code .ulf} file, then {@code
   * defaultFileName} is appended under {@code src/lib/} (and any intervening subdirectory
   * segments).
   *
   * <p>This is used to support import statements of the form:
   *
   * <pre>{@code
   * import ReactorClassName from <packageName>
   * import ReactorClassName from <packageName/subdir>
   * }</pre>
   *
   * @param uriStr The package import string (e.g., {@code packageName}, {@code
   *     packageName/subdir}, or {@code packageName/file.lf}).
   * @param resource The resource from which the URI resolution should start.
   * @param defaultFileName The file name to use if {@code uriStr} does not end in a library file.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI string does not contain a package name, if no "src"
   *     directory is found, or if the package cannot be located.
   */
  public static String buildPackageURI(String uriStr, Resource resource, String defaultFileName) {
    Path uriPath = parseImportUri(uriStr);
    Path relative = relativeLibPath(uriPath, defaultFileName);
    if (relative == null) {
      throw new IllegalArgumentException("Missing library file name for import '" + uriStr + "'.");
    }
    Path root = findProjectRoot(FileUtil.toPath(resource));
    return toFileUriString(resolvePackageFile(root, uriPath.getName(0).toString(), relative));
  }

  /**
   * Resolve the library file path(s) for a package import.
   *
   * <p>If {@code uriStr} ends with a {@code .lf}/{@code .ulf} file, a single path is returned
   * (preserving any subdirectory segments). Otherwise:
   *
   * <ul>
   *   <li>if {@code defaultFileName} is provided, that file is appended under {@code src/lib/} (and
   *       any subdirectory segments);
   *   <li>otherwise all {@code .lf}/{@code .ulf} files under the corresponding {@code src/lib}
   *       directory (or subdirectory) are returned.
   * </ul>
   *
   * <p>Returned values are {@code file:} URI strings (from {@link Path#toUri()}) so they round-trip
   * through {@code URI.createURI(...)} on all platforms.
   *
   * The latter case supports {@code import ReactorClassName from <packageName>} during linking,
   * when the reactor class name may not yet be readable from the AST.
   */
  public static List<String> buildPackageURIs(
      String uriStr, Resource resource, String defaultFileName) {
    Path uriPath = parseImportUri(uriStr);
    Path root = findProjectRoot(FileUtil.toPath(resource));
    String packageName = uriPath.getName(0).toString();
    Path relative = relativeLibPath(uriPath, defaultFileName);

    if (relative != null) {
      return List.of(toFileUriString(resolvePackageFile(root, packageName, relative)));
    }

    // No explicit or default file: list library files in src/lib[/subdir].
    Path libDir = resolvePackageLibDirectory(root, packageName, directoryPrefix(uriPath));
    if (!Files.isDirectory(libDir)) {
      throw new IllegalArgumentException(
          "Package '" + packageName + "' has no library directory at " + libDir + ".");
    }

    try (Stream<Path> files = Files.list(libDir)) {
      List<String> uris =
          files
              .filter(Files::isRegularFile)
              .filter(path -> isLibraryFileName(path.getFileName().toString()))
              .map(ImportUtil::toFileUriString)
              .sorted()
              .toList();
      if (uris.isEmpty()) {
        throw new IllegalArgumentException(
            "Package '" + packageName + "' has no .lf/.ulf files in " + libDir + ".");
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
   * @param uriStr A package import path. The last segment must be a {@code .lf} or {@code .ulf}
   *     file (e.g., {@code package/file.lf} or {@code package/subdir/file.lf}).
   * @param srcPath The path from which the URI resolution should start.
   * @return The path to the imported library file.
   * @throws IllegalArgumentException if the URI string or source path is null, empty, or does not
   *     end in a library file, if no "src" directory is found, or if the package cannot be located.
   */
  public static Path buildPackageURIfromSrc(String uriStr, String srcPath) {
    if (uriStr == null || srcPath == null || uriStr.trim().isEmpty() || srcPath.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    Path uriPath = parseImportUri(uriStr);
    if (!lastSegmentIsLibraryFile(uriPath)) {
      throw new IllegalArgumentException(
          "URI must end with a .lf or .ulf library file name: '" + uriStr + "'.");
    }
    Path root = findProjectRoot(Paths.get(srcPath).toAbsolutePath());
    return resolvePackageFile(root, uriPath.getName(0).toString(), relativeLibPath(uriPath, null));
  }

  /**
   * Like {@link #buildPackageURIfromSrc(String, String)}, but allows the imported file name to be
   * omitted. If the last path segment is not a {@code .lf}/{@code .ulf} file, then {@code
   * defaultFileName} is appended under {@code src/lib/} (and any intervening subdirectory
   * segments).
   *
   * @param uriStr The package import string (e.g., {@code packageName}, {@code
   *     packageName/subdir}, or {@code packageName/file.lf}).
   * @param srcPath The path from which the URI resolution should start.
   * @param defaultFileName The file name to use if {@code uriStr} does not end in a library file.
   * @return The path to the imported library file.
   */
  public static Path buildPackageURIfromSrc(String uriStr, String srcPath, String defaultFileName) {
    if (uriStr == null || srcPath == null || uriStr.trim().isEmpty() || srcPath.trim().isEmpty()) {
      throw new IllegalArgumentException("URI string and source path must not be null or empty.");
    }

    Path uriPath = parseImportUri(uriStr);
    Path relative = relativeLibPath(uriPath, defaultFileName);
    if (relative == null) {
      throw new IllegalArgumentException("Missing library file name for import '" + uriStr + "'.");
    }
    Path root = findProjectRoot(Paths.get(srcPath).toAbsolutePath());
    return resolvePackageFile(root, uriPath.getName(0).toString(), relative);
  }

  /**
   * Return whether {@code uriStr} already names a library file ({@code .lf} or {@code .ulf}) as its
   * last path segment.
   */
  public static boolean specifiesLibraryFile(String uriStr) {
    return lastSegmentIsLibraryFile(parseImportUri(uriStr));
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

  /** Convert a filesystem path to a {@code file:} URI string for EMF/Xtext resource loading. */
  private static String toFileUriString(Path path) {
    return path.toUri().toString();
  }

  private static boolean isLibraryFileName(String name) {
    String lower = name.toLowerCase(Locale.ROOT);
    return lower.endsWith(".lf") || lower.endsWith(".ulf");
  }

  private static boolean lastSegmentIsLibraryFile(Path uriPath) {
    return isLibraryFileName(uriPath.getFileName().toString());
  }

  /**
   * Return the path relative to {@code src/lib} for this import, or {@code null} if the caller
   * should list all library files in the corresponding directory.
   *
   * <p>If the last segment is a {@code .lf}/{@code .ulf} file, all segments after the package name
   * are returned. Otherwise {@code defaultFileName} is appended after any subdirectory segments. If
   * there is no default file name, returns {@code null}.
   */
  private static Path relativeLibPath(Path uriPath, String defaultFileName) {
    Path afterPackage =
        uriPath.getNameCount() > 1 ? uriPath.subpath(1, uriPath.getNameCount()) : null;

    if (afterPackage != null && lastSegmentIsLibraryFile(afterPackage)) {
      return afterPackage;
    }

    if (defaultFileName == null || defaultFileName.isBlank()) {
      return null;
    }

    if (afterPackage == null) {
      return Paths.get(defaultFileName);
    }
    return afterPackage.resolve(defaultFileName);
  }

  /** Directory prefix under {@code src/lib} when the import does not end in a library file. */
  private static Path directoryPrefix(Path uriPath) {
    if (uriPath.getNameCount() <= 1 || lastSegmentIsLibraryFile(uriPath)) {
      return null;
    }
    return uriPath.subpath(1, uriPath.getNameCount());
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
      if (Files.isDirectory(candidate)) {
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

  private static Path resolvePackageLibDirectory(
      Path root, String packageName, Path relativeDirectory) {
    Path libDir = findPackageDirectory(root, packageName).resolve("src").resolve("lib");
    if (relativeDirectory == null) {
      return libDir;
    }
    return libDir.resolve(relativeDirectory);
  }

  private static Path resolvePackageFile(Path root, String packageName, Path relativeLibPath) {
    Path libDir = findPackageDirectory(root, packageName).resolve("src").resolve("lib");
    Path resolved = libDir.resolve(relativeLibPath).normalize();
    if (!resolved.startsWith(libDir)) {
      throw new IllegalArgumentException(
          "Invalid import path; must stay within " + libDir + ": '" + relativeLibPath + "'.");
    }
    return resolved;
  }
}
