package org.lflang.target.property;

import java.util.HashMap;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.StringDictionaryType;

/**
 * Directive to specify additional CMake configure arguments.
 *
 * <p>Each key-value pair is converted to a CMake configure definition of the form {@code -Dkey=value}.
 * This is applied when invoking CMake (not when generating CMakeLists.txt).
 */
public final class CmakeArgsProperty extends TargetProperty<Map<String, String>, StringDictionaryType> {

  /** Singleton target property instance. */
  public static final CmakeArgsProperty INSTANCE = new CmakeArgsProperty();

  private CmakeArgsProperty() {
    // Reuse the existing "string keys -> string values" dictionary type.
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
  public Element toAstElement(Map<String, String> value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "cmake-args";
  }
}


