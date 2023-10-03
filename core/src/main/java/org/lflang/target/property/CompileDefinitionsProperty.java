package org.lflang.target.property;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.StringDictionaryType;

/** Directive to specify compile-time definitions. */
public class CompileDefinitionsProperty extends AbstractTargetProperty<Map<String, String>> {

  public CompileDefinitionsProperty() {
    super(StringDictionaryType.COMPILE_DEFINITION);
  }

  public void put(String k, String v) {
    this.isSet = true;
    var value = this.get();
    value.put(k, v);
  }

  @Override
  public Map<String, String> initialValue() {
    return new HashMap<>();
  }

  @Override
  protected Map<String, String> fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.elementToStringMaps(node);
  }

  @Override
  protected Map<String, String> fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get());
  }

  @Override
  public String name() {
    return "compile-definitions";
  }
}
