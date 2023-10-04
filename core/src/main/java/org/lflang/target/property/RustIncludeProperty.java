package org.lflang.target.property;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.eclipse.emf.ecore.EObject;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.type.UnionType;
import org.lflang.util.FileUtil;
import org.lflang.util.StringUtil;

/**
 * List of module files to link into the crate as top-level. For instance, a {@code target Rust {
 * rust-modules: [ "foo.rs" ] }} will cause the file to be copied into the generated project, and
 * the generated {@code main.rs} will include it with a {@code mod foo;}. If one of the paths is a
 * directory, it must contain a {@code mod.rs} file, and all its contents are copied.
 */
public class RustIncludeProperty extends AbstractTargetProperty<List<Path>, UnionType> {

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
  public List<Path> fromAst(Element node, MessageReporter reporter) {
    var list = new ArrayList<Path>();
    Path referencePath;
    try {
      referencePath = FileUtil.toPath(node.eResource().getURI()).toAbsolutePath();
    } catch (IllegalArgumentException e) {
      reporter.at(node).error("Invalid path? " + e.getMessage());
      throw e;
    }

    // we'll resolve relative paths to check that the files
    // are as expected.

    if (node.getLiteral() != null) {
      Path resolved = referencePath.resolveSibling(StringUtil.removeQuotes(node.getLiteral()));
      if (this.checkTopLevelModule(resolved, node, reporter)) {
        list.add(resolved);
      }
    } else if (node.getArray() != null) {
      for (Element element : node.getArray().getElements()) {
        String literal = StringUtil.removeQuotes(element.getLiteral());
        Path resolved = referencePath.resolveSibling(literal);
        if (this.checkTopLevelModule(resolved, node, reporter)) {
          list.add(resolved);
        }
      }
    }
    return list;
  }

  @Override
  protected List<Path> fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement() {
    // do not check paths here, and simply copy the absolute path over
    List<Path> paths = this.get();
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

  @Override
  public String name() {
    return "rust-include";
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
