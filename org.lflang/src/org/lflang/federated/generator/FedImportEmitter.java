package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.stream.Collectors;

import org.lflang.ast.ToLf;
import org.lflang.lf.Model;

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

        return imports.stream()
                      .map(ToLf.instance::doSwitch)
                      .collect(Collectors.joining("\n"));
    }
}