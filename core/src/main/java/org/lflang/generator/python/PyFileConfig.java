package org.lflang.generator.python;

import java.io.IOException;
import java.nio.file.Path;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.c.CFileConfig;

public class PyFileConfig extends CFileConfig {
  public PyFileConfig(Resource resource, Path srcGenBasePath, boolean useHierarchicalBin)
      throws IOException {
    super(resource, srcGenBasePath, useHierarchicalBin);
  }

  @Override
  protected String getExecutableExtension() {
    return GeneratorUtils.isHostWindows() ? ".bat" : "";
  }
}
