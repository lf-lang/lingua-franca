package org.lflang.generator.c;

import org.eclipse.emf.common.util.URI;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.FileConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;

public class CFileConfig extends FileConfig {
  private final Path includePath;
  private Map<String, Map<URI, Integer>> nameMap;

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

  public void setNameMap(List<Reactor> reactors) {
    Map<String, Integer> countMap = new HashMap<>();
    assert nameMap == null;
    nameMap = new HashMap<>();
    for (var reactor : reactors) {
      var def = ASTUtils.toDefinition(reactor);
      if (nameMap.containsKey(def.getName())) {
        nameMap.get(def.getName()).put(def.eResource().getURI(), countMap.get(def.getName()));
        countMap.put(def.getName(), countMap.get(def.getName()));
      } else {
        nameMap.put(def.getName(), new HashMap<>());
        nameMap.get(def.getName()).put(def.eResource().getURI(), 0);
        countMap.put(def.getName(), 1);
      }
    }
  }

  public String uniqueName(ReactorDecl decl) {
    var name = decl.getName();
    return name + (nameMap.get(name).get(decl.eResource().getURI()) == 0 ? "" : nameMap.get(name));
  }

  public Path getIncludePath() {
    return includePath;
  }

  public String getRuntimeIncludePath() {
    return "/lib/c/reactor-c/include";
  }
}
