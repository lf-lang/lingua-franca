package org.lflang.generator.c;

import org.lflang.FileConfig;
import org.lflang.federated.FederateInstance;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.lf.Reactor;
import org.lflang.util.FileUtil;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;


public class CFileConfig extends FileConfig {
    boolean isFederated = false;
    FederateInstance federate = null;

    public CFileConfig(
        Resource resource,
        Path srcGenBasePath,
        boolean useHierarchicalBin,
        FederateInstance federate,
        boolean isFederated
    ) throws IOException {
        super(resource, srcGenBasePath, useHierarchicalBin);
        this.isFederated = isFederated;
        this.federate = federate;
        cBuildDirectories.forEach(this::deleteDirectory);
    }

    public CFileConfig(
        FileConfig fileConfig, 
        FederateInstance federate,
        boolean isFederated
    ) throws IOException {
        super(fileConfig.resource, 
              fileConfig.getSrcGenBasePath(), 
              fileConfig.useHierarchicalBin);
        this.isFederated = isFederated;
        this.federate = federate;
        cBuildDirectories.forEach(this::deleteDirectory);
    }

    List<Path> cBuildDirectories = List.of(
        getOutPath().resolve("build"),
        getOutPath().resolve("lib"),
        getOutPath().resolve("include"),
        getOutPath().resolve("share")
    );

    private Path getGenDir(Resource r) throws IOException { 
        return getDirectory(r).resolve(name); 
    }

    private void deleteDirectory(Path dir) {
        try {
            FileUtil.deleteDirectory(dir);
        } catch (Exception ex) {
            throw new RuntimeException(ex);
        }
    }

    public Path getReactorHeaderPath(Reactor r) throws IOException {
        return getGenDir(r.eResource()).resolve(r.getName() + ".h");
    }

    public Path getReactorSourcePath(Reactor r) throws IOException {
        return getGenDir(r.eResource()).resolve(r.getName() + ".c");
    }

    public Path getDockerPath(Reactor r) throws IOException {
        return getGenDir(r.eResource()).resolve(r.getName() + ".Dockerfile");
    }
}
