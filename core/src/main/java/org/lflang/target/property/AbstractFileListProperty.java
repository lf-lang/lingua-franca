package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public abstract class AbstractFileListProperty extends AbstractTargetProperty<List<String>> {

  public AbstractFileListProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public void set(Element node, MessageReporter reporter) {
    if (!this.isSet) {
      super.set(node, reporter);
    } else {
      this.get().addAll(fromAst(node, reporter));
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
  public Element toAstElement() {
    return ASTUtils.toElement(get());
  }
}
