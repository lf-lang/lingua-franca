package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.UnionType;

public abstract class FileListProperty extends TargetProperty<List<String>, UnionType> {

  public FileListProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public void update(TargetConfig config, List<String> value) {
    var files = new ArrayList<>(value);
    var existing = config.get(this);
    if (config.isSet(this)) {
      existing.forEach(
          f -> {
            if (!files.contains(f)) {
              files.add(f);
            }
          });
    }
    config.set(this, files.stream().sorted(String::compareTo).toList());
  }

  @Override
  protected List<String> fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.elementToListOfStrings(node);
  }

  @Override
  protected List<String> fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement(List<String> value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public boolean loadFromImport() {
    return true;
  }
}
