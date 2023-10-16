package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.UnionType;

/** Note: {@code set} implements an "append" semantics. */
public abstract class StringListProperty extends TargetProperty<List<String>, UnionType> {

  public StringListProperty() {
    super(UnionType.STRING_OR_STRING_ARRAY);
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
