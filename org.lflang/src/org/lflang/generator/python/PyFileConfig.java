package org.lflang.generator.python;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.eclipse.emf.ecore.resource.Resource;

import org.lflang.FileConfig;
import org.lflang.util.LFCommand;

public class PyFileConfig extends FileConfig {
    public PyFileConfig(Resource resource, Path srcGenBasePath, boolean useHierarchicalBin) throws IOException {
        super(resource, srcGenBasePath, useHierarchicalBin);
    }

    @Override
    public LFCommand getCommand() {
        return LFCommand.get("python3",
                      List.of(srcPkgPath.relativize(getExecutable()).toString()),
                      true,
                      srcPkgPath);
    }
}





