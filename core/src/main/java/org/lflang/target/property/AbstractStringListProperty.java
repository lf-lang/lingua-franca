package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.UnionType;

/** Note: {@code set} implements an "append" semantics. */
public abstract class AbstractStringListProperty
    extends AbstractTargetProperty<List<String>, UnionType> {

  public AbstractStringListProperty() {
    super(UnionType.STRING_OR_STRING_ARRAY);
  }

  public void add(TargetConfig config, String entry) {
    config.markSet(this);
    config.get(this).add(entry);
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
    return List.of(string.split(" ")); // FIXME: this does not look right
  }

  @Override
  public Element toAstElement(List<String> value) {
    return ASTUtils.toElement(value);
  }
}
