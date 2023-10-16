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
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.StringDictionaryType;

/**
 * Additional (preprocessor) definitions to add to the compile command if appropriate.
 *
 * <p>The first string is the definition itself, and the second string is the value to attribute to
 * that definition, if any. The second value could be left empty.
 */
public class CompileDefinitionsProperty
    extends AbstractTargetProperty<Map<String, String>, StringDictionaryType> {

  public CompileDefinitionsProperty() {
    super(StringDictionaryType.COMPILE_DEFINITION);
  }

  @Override
  public void update(TargetConfig config, Map<String, String> value) {
    var pairs = new HashMap<>(value);
    var existing = config.get(this);
    if (config.isSet(this)) {
      existing.forEach(
          (k, v) -> {
            if (!pairs.containsKey(k)) {
              pairs.put(k, v);
            }
          });
    }
    config.set(this, pairs);
  }

  @Override
  public Map<String, String> initialValue() {
    return Map.of();
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
  public Element toAstElement(Map<String, String> value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "compile-definitions";
  }
}
