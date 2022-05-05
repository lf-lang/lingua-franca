package org.lflang.generator.c;

import org.lflang.FileConfig;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.lf.Reactor;
import org.lflang.util.FileUtil;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;


public class CFileConfig extends FileConfig {
    public CFileConfig(
        Resource resource, 
        Path srcGenBasePath,
        boolean useHierarchicalBin
    ) throws IOException {
        super(resource, srcGenBasePath, useHierarchicalBin);
        cBuildDirectories.forEach(this::deleteDirectory);
    }

    List<Path> cBuildDirectories = List.of(
        getOutPath().resolve("build"),
        getOutPath().resolve("lib"),
        getOutPath().resolve("include"),
        getOutPath().resolve("share")
    );

    private Path getGenDir(Reactor r) throws IOException { 
        return getDirectory(r.eResource()).resolve(r.getName()); 
    }

    private void deleteDirectory(Path dir) {
        try {
            FileUtil.deleteDirectory(dir);
        } catch (Exception ex) {
            throw new RuntimeException(ex);
        }
    }

    public Path getReactorHeaderPath(Reactor r) throws IOException {
        return getGenDir(r).resolve(r.getName() + ".h");
    }

    public Path getReactorSourcePath(Reactor r) throws IOException {
        return getGenDir(r).resolve(r.getName() + ".c");
    }
}
