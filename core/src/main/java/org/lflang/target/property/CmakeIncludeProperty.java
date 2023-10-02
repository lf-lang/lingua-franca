package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public class CmakeIncludeProperty extends AbstractTargetProperty<List<String>> {

  public CmakeIncludeProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
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
      // NOTE: This merging of lists is potentially dangerous since
      // the incoming list of cmake-includes can belong to a .lf file that is
      // located in a different location, and keeping just filename
      // strings like this without absolute paths is incorrect.
      this.get().addAll(ASTUtils.elementToListOfStrings(node));
    }
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
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.CPP, Target.C, Target.CCPP);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get());
  }

  @Override
  public String name() {
    return "cmake-include";
  }
}
