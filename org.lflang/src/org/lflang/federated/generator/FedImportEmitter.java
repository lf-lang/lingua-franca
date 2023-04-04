package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.util.EcoreUtil;

import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Import;
import org.lflang.lf.Model;

/**
 * Helper class to generate import statements for a federate.
 *
 * @author Soroush Bateni
 */
public class FedImportEmitter {

    private static Set<Import> visitedImports = new HashSet<>();

    /**
     * Generate import statements for {@code federate}.
     */
    String generateImports(FederateInstance federate, FedFileConfig fileConfig) {
        var imports = ((Model) federate.instantiation.eContainer().eContainer())
            .getImports()
            .stream()
            .filter(federate::contains).toList();

        // Transform the URIs
        imports.stream()
               .filter(i -> !visitedImports.contains(i))
               .forEach(i -> {
                   visitedImports.add(i);
                   Path importPath =
                       fileConfig.srcPath
                           .resolve(i.getImportURI()).toAbsolutePath();
                   i.setImportURI(fileConfig.getSrcPath().relativize(importPath)
                                      .toString().replace('\\', '/')
                   );
               });

        var importStatements = new CodeBuilder();

        // Add import statements needed for the ordinary functionality of the federate
        importStatements.pr(imports.stream()
                                   .map(i -> {
                                       var new_import = EcoreUtil.copy(i);
                                       new_import.getReactorClasses().removeIf(
                                           importedReactor -> !federate.contains(importedReactor)
                                       );
                                       return new_import;
                                   })
                                   .map(FormattingUtils.renderer(federate.target))
                                   .collect(Collectors.joining("\n")));

        return importStatements.getCode();
    }
}
