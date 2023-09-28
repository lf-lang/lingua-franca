package org.lflang.target.property;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.StringDictionaryType;

public class CompileDefinitionsProperty extends TargetPropertyConfig<Map<String, String>> {

  public CompileDefinitionsProperty() {
    super(StringDictionaryType.COMPILE_DEFINITION);
  }

  @Override
  public Map<String, String> initialValue() {
    return new HashMap<>();
  }

  @Override
  protected Map<String, String> fromAst(Element value, MessageReporter err) {
    return ASTUtils.elementToStringMaps(value);
  }

  @Override
  protected Map<String, String> fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.value);
  }
}
