package org.lflang.target.property;

/** Directive to instruct the code generator to not produce line directives. */
public final class NoSourceMappingProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final NoSourceMappingProperty INSTANCE = new NoSourceMappingProperty();

  private NoSourceMappingProperty() {
    super();
  }

  @Override
  public String name() {
    return "no-source-mapping";
  }
}
