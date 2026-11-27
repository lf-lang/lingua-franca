package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.PrimitiveType;

/**
 * Size of the heap given to the global allocator on embedded boards ({@code platform: { board:
 * ... }}), in KiB. The runtime allocates its reactors and event queue on it.
 */
public final class HeapSizeProperty extends TargetProperty<Integer, PrimitiveType> {

  /** Singleton target property instance. */
  public static final HeapSizeProperty INSTANCE = new HeapSizeProperty();

  /** Heap size used when the property is not set, in KiB. */
  public static final int DEFAULT_KIB = 32;

  private HeapSizeProperty() {
    super(PrimitiveType.NON_NEGATIVE_INTEGER);
  }

  @Override
  public Integer initialValue() {
    return DEFAULT_KIB;
  }

  @Override
  protected Integer fromString(String string, MessageReporter reporter) {
    return Integer.parseInt(string);
  }

  @Override
  protected Integer fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.toInteger(node);
  }

  @Override
  public Element toAstElement(Integer value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    if (!config.isSet(this)) return;
    if (config.get(this) == 0) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
          .error("heap-size must be greater than 0 (KiB).");
    }
    if (!config.isSet(PlatformProperty.INSTANCE)
        || !config.get(PlatformProperty.INSTANCE).board().setByUser()) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__NAME)
          .warning("heap-size only applies to embedded boards (platform: { board: ... }); ignored.");
    }
  }

  @Override
  public String name() {
    return "heap-size";
  }
}
