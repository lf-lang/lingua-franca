package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

/** Note: {@code set} implements an "append" semantics. */
public abstract class AbstractStringListProperty
    extends AbstractTargetProperty<List<String>, UnionType> {

  public AbstractStringListProperty() {
    super(UnionType.STRING_OR_STRING_ARRAY);
  }

  public void add(String entry) {
    this.isSet = true;
    var value = this.get();
    value.add(entry);
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
      this.get().addAll(this.fromAst(node, reporter));
    }
  }

  @Override
  public void set(String string, MessageReporter err) {
    if (!this.isSet) {
      super.set(string, err);
    } else {
      this.get().addAll(this.fromString(string, err));
    }
  }

  @Override
  public List<String> fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.elementToListOfStrings(node);
  }

  @Override
  protected List<String> fromString(String string, MessageReporter reporter) {
    return List.of(string.split(" "));
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(get());
  }
}
