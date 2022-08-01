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

        importStatements.pr(generateImportsForUpstreamInterfaces(federate, new ArrayList<>()));

        return importStatements.getCode();
    }

    /**
     * Generate import statements for all upstream federates' interfaces.
     */
    private String generateImportsForUpstreamInterfaces(FederateInstance federate, List<Reactor> alreadyImported) {
        CodeBuilder importStatements = new CodeBuilder();
        // Add import statements for causality interfaces of upstream federates.
        var upstreamFederates =
            federate.getZeroDelayImmediateUpstreamFederates();

        upstreamFederates.forEach(federateInstance -> {
            var reactorDecl = federateInstance.instantiation.getReactorClass();
            var reactorClass = ASTUtils.toDefinition(reactorDecl);
            if (!alreadyImported.contains(reactorClass)) {
                alreadyImported.add(reactorClass);
                importStatements.pr(
                """
                import %1$s as _lf_%2$s_interface from "include/interfaces/%1$s_interface.lf"
                """.formatted(
                    federateInstance.instantiation.getReactorClass().getName(),
                    // Handle renamed reactors
                    reactorClass.getName()
                )
                );
            }
            // Keep going upstream
            importStatements.pr(generateImportsForUpstreamInterfaces(federateInstance, alreadyImported));
        });
        return importStatements.getCode();
    }
}