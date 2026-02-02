package org.lflang.target.property.type;

import org.lflang.target.property.type.StaticMapperType.StaticMapper;

public class StaticMapperType extends OptionsType<StaticMapper> {

  @Override
  protected Class<StaticMapper> enumClass() {
    return StaticMapper.class;
  }

  /** Supported static mappers. */
  public enum StaticMapper {
    LB;

    public static StaticMapper getDefault() {
      return LB;
    }
  }
}
