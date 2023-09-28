package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public abstract class DefaultFileListProperty extends TargetPropertyConfig<List<String>> {

  public DefaultFileListProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public void set(Element value, MessageReporter err) {
    if (!this.isSet) {
      super.set(value, err);
    } else {
      this.value.addAll(fromAst(value, err));
    }
  }

  @Override
  public List<String> fromAst(Element value, MessageReporter err) {
    return ASTUtils.elementToListOfStrings(value);
  }

  @Override
  protected List<String> fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(value);
  }
}
