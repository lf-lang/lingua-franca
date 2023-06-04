package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Path;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.FileConfig;

public class CFileConfig extends FileConfig {
  private final Path includePath;

  public CFileConfig(Resource resource, Path srcGenBasePath, boolean useHierarchicalBin)
      throws IOException {
    super(resource, srcGenBasePath, useHierarchicalBin);
    var includeDir = getOutPath().resolve("include");
    includePath =
        !useHierarchicalBin
            ? includeDir
            : includeDir
                .resolve(getOutPath().relativize(srcPath))
                .resolve(srcFile.getFileName().toString().split("\\.")[0]);
  }

  public Path getIncludePath() {
    return includePath;
  }

  public String getRuntimeIncludePath() {
    return "/lib/c/reactor-c/include";
  }
}
