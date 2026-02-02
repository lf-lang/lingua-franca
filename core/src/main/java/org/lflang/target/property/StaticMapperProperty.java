package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.StaticMapperType;
import org.lflang.target.property.type.StaticMapperType.StaticMapper;

/** Directive for specifying the use of a specific static mapper. */
public final class StaticMapperProperty extends TargetProperty<StaticMapper, StaticMapperType> {

  /** Singleton target property instance. */
  public static final StaticMapperProperty INSTANCE = new StaticMapperProperty();

  private StaticMapperProperty() {
    super(new StaticMapperType());
  }

  @Override
  public StaticMapper initialValue() {
    return StaticMapper.getDefault();
  }

  @Override
  public StaticMapper fromAst(Element node, MessageReporter reporter) {
    var mapper = fromString(ASTUtils.elementToSingleString(node), reporter);
    if (mapper != null) {
      return mapper;
    } else {
      return StaticMapper.getDefault();
    }
  }

  @Override
  protected StaticMapper fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public Element toAstElement(StaticMapper value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "mapper";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    // Do nothing for now.
  }
}
