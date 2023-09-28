package org.lflang.target.property;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.eclipse.emf.ecore.EObject;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.type.UnionType;
import org.lflang.util.FileUtil;
import org.lflang.util.StringUtil;

public class RustIncludeProperty extends TargetPropertyConfig<List<Path>> {

  public RustIncludeProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.Rust);
  }

  @Override
  public List<Path> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public List<Path> fromAst(Element value, MessageReporter err) {
    var list = new ArrayList<Path>();
    Path referencePath;
    try {
      referencePath = FileUtil.toPath(value.eResource().getURI()).toAbsolutePath();
    } catch (IllegalArgumentException e) {
      err.at(value).error("Invalid path? " + e.getMessage());
      throw e;
    }

    // we'll resolve relative paths to check that the files
    // are as expected.

    if (value.getLiteral() != null) {
      Path resolved = referencePath.resolveSibling(StringUtil.removeQuotes(value.getLiteral()));
      if (this.checkTopLevelModule(resolved, value, err)) {
        list.add(resolved);
      }
    } else if (value.getArray() != null) {
      for (Element element : value.getArray().getElements()) {
        String literal = StringUtil.removeQuotes(element.getLiteral());
        Path resolved = referencePath.resolveSibling(literal);
        if (this.checkTopLevelModule(resolved, value, err)) {
          list.add(resolved);
        }
      }
    }
    return list;
  }

  @Override
  protected List<Path> fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement() {
    // do not check paths here, and simply copy the absolute path over
    List<Path> paths = this.value;
    if (paths.isEmpty()) {
      return null;
    } else if (paths.size() == 1) {
      return ASTUtils.toElement(paths.get(0).toString());
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      Array arr = LfFactory.eINSTANCE.createArray();
      for (Path p : paths) {
        arr.getElements().add(ASTUtils.toElement(p.toString()));
      }
      e.setArray(arr);
      return e;
    }
  }

  private boolean checkTopLevelModule(Path path, EObject errorOwner, MessageReporter err) {
    String fileName = path.getFileName().toString();
    if (!Files.exists(path)) {
      err.at(errorOwner).error("File not found");
    } else if (Files.isRegularFile(path) && !fileName.endsWith(".rs")) {
      err.at(errorOwner).error("Not a rust file");
    } else if (fileName.equals("main.rs")) {
      err.at(errorOwner).error("Cannot use 'main.rs' as a module name (reserved)");
    } else if (fileName.equals("reactors") || fileName.equals("reactors.rs")) {
      err.at(errorOwner).error("Cannot use 'reactors' as a module name (reserved)");
    } else if (Files.isDirectory(path) && !Files.exists(path.resolve("mod.rs"))) {
      err.at(errorOwner).error("Cannot find module descriptor in directory");
    } else {
      return true;
    }
    return false;
  }
}
