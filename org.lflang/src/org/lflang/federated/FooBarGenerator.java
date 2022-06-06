package org.lflang.federated;

import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.LFGeneratorContext;

public class FooBarGenerator {

    public FooBarGenerator(FileConfig fileConfig, ErrorReporter errorReporter) {

    }
    public boolean doGenerate(Resource resource, LFGeneratorContext context) {
        System.out.println("Federated program detected.");
        return false;
    }
}
