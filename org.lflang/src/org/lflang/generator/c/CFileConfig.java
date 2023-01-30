package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Path;

import org.eclipse.emf.ecore.resource.Resource;

import org.lflang.FileConfig;

public class CFileConfig extends FileConfig {
    public CFileConfig(Resource resource, Path srcGenBasePath, boolean useHierarchicalBin) throws IOException {
        super(resource, srcGenBasePath, useHierarchicalBin);
    }
}





