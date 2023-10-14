package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.UnionType;

/**
 * Directive to specify a cmake to be included by the generated build systems.
 *
 * <p>This gives full control over the C/C++ build as any cmake parameters can be adjusted in the
 * included file.
 */
public class CmakeIncludeProperty extends AbstractTargetProperty<List<String>, UnionType> {

  public CmakeIncludeProperty() {
    super(UnionType.FILE_OR_FILE_ARRAY);
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
    var files = fromAst(node, reporter);
    var existing = config.get(this);
    if (config.isSet(this)) {
      files.stream()
          .forEach(
              f -> {
                if (!existing.contains(f)) {
                  existing.add(f);
                }
              });

    } else {
      config.get(this).addAll(files);
      config.markSet(this);
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
  public Element toAstElement(List<String> value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "cmake-include";
  }
}
