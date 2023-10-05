package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.UnionType;

public abstract class AbstractFileListProperty
    extends AbstractTargetProperty<List<String>, UnionType> {

  public AbstractFileListProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public void update(TargetConfig config, Element node, MessageReporter reporter) {
    if (config.isSet(this)) {
      config.get(this).addAll(fromAst(node, reporter));
    }
  }

  @Override
  public List<String> fromAst(Element node, MessageReporter reporter) {
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
}
