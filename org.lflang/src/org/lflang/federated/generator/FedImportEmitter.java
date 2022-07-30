package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;

public class FedImportEmitter {
    /**
     * Generate import statements for federate
     *
     * @param federate
     * @return
     */
    String generateImports(FederateInstance federate, FedFileConfig fileConfig) {
        var imports = ((Model) federate.instantiation.eContainer().eContainer())
            .getImports()
            .stream()
            .filter(federate::contains).collect(Collectors.toList());

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

        importStatements.pr(generateImportsForUpstreamInterfaces(federate));

        return importStatements.getCode();
    }

    private String generateImportsForUpstreamInterfaces(FederateInstance federate) {
        CodeBuilder importStatements = new CodeBuilder();
        // Add import statements for causality interfaces of upstream federates.
        var upstreamFederates =
            federate.getZeroDelayImmediateUpstreamFederates();

        upstreamFederates.forEach(federateInstance -> {
            importStatements.pr(
            """
            import %s from "include/interfaces/%s_interface.lf"
            """.formatted(
                federateInstance.instantiation.getReactorClass().getName(),
                federateInstance.instantiation.getName()));
            importStatements.pr(generateImportsForUpstreamInterfaces(federateInstance));
        });
        return importStatements.getCode();
    }
}