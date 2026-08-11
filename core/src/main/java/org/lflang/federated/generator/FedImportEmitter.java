package org.lflang.federated.generator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.lflang.ast.FormattingUtil;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Import;
import org.lflang.lf.Model;
import org.lflang.util.ImportUtil;

/**
 * Helper class to generate import statements for a federate.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FedImportEmitter {

  private static final Set<Import> visitedImports = new HashSet<>();

  /** Generate import statements for `federate`. */
  String generateImports(FederateInstance federate, FederationFileConfig fileConfig) {
    var imports =
        ((Model) federate.instantiation.eContainer().eContainer())
            .getImports().stream().filter(federate::references).toList();

    // Transform the URIs
    imports.stream()
        .filter(i -> !visitedImports.contains(i))
        .forEach(
            i -> {
              visitedImports.add(i);
              Path importPath;
              if (i.getImportURI() != null) {
                importPath =
                    fileConfig.srcPath.resolve(Paths.get(i.getImportURI())).toAbsolutePath();
              } else {
                // Support import syntax where the library file may be omitted:
                //   import ReactorClassName from <packageName>
                //   import ReactorClassName from <packageName/subdir>
                // which map to src/lib[/subdir]/ReactorClassName.lf (or .ulf)
                if (!ImportUtil.specifiesLibraryFile(i.getImportPackage())) {
                  if (i.getReactorClasses().size() != 1) {
                    throw new IllegalArgumentException(
                        "Import from <"
                            + i.getImportPackage()
                            + "> omitted the library file name, but the import statement "
                            + "imports multiple reactor classes. Use <package/file.lf> instead.");
                  }
                  var reactorClassName =
                      ImportUtil.getImportedReactorClassName(i.getReactorClasses().get(0));
                  if (reactorClassName == null || reactorClassName.isBlank()) {
                    throw new IllegalArgumentException(
                        "Cannot resolve imported reactor class name for import from <"
                            + i.getImportPackage()
                            + ">.");
                  }
                  importPath =
                      fileConfig
                          .srcPath
                          .resolve(
                              ImportUtil.buildPackageURIfromSrc(
                                  i.getImportPackage(),
                                  fileConfig.srcPath.toString(),
                                  reactorClassName + ".lf"))
                          .toAbsolutePath();
                } else {
                  importPath =
                      fileConfig
                          .srcPath
                          .resolve(
                              ImportUtil.buildPackageURIfromSrc(
                                  i.getImportPackage(), fileConfig.srcPath.toString()))
                          .toAbsolutePath();
                }
              }
              i.setImportURI(
                  fileConfig.getSrcPath().relativize(importPath).toString().replace('\\', '/'));
            });

    var importStatements = new CodeBuilder();

    // Add import statements needed for the ordinary functionality of the federate
    importStatements.pr(
        imports.stream()
            .map(
                i -> {
                  var new_import = EcoreUtil.copy(i);
                  new_import
                      .getReactorClasses()
                      .removeIf(importedReactor -> !federate.references(importedReactor));
                  return new_import;
                })
            .map(FormattingUtil.renderer(federate.targetConfig.target))
            .collect(Collectors.joining("\n")));

    return importStatements.getCode();
  }
}
