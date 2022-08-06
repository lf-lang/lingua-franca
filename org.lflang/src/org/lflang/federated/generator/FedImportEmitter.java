package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;

/**
 * Helper class to generate import statements for a federate.
 *
 * @author Soroush Bateni (soroush@berkeley.edu)
 */
public class FedImportEmitter {
    /**
     * Generate import statements for {@code federate}.
     */
    String generateImports(FederateInstance federate, FedFileConfig fileConfig) {
        var imports = ((Model) federate.instantiation.eContainer().eContainer())
            .getImports()
            .stream()
            .filter(federate::contains).toList();

        // Transform the URIs
        imports.forEach(imp -> {
            Path importPath =
                fileConfig.srcPath
                    .resolve(imp.getImportURI()).toAbsolutePath();
            imp.setImportURI(fileConfig.getFedSrcPath().relativize(importPath)
                                         .toString()
            );
        });

        var importStatements = new CodeBuilder();

        // Add import statements needed for the ordinary functionality of the federate
        importStatements.pr(imports.stream()
                                   .map(FormattingUtils.renderer(federate.target))
                                   .collect(Collectors.joining("\n")));

        return importStatements.getCode();
    }
}