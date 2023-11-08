package org.lflang.target.property;

/** Directive to allow including OpenSSL libraries and process HMAC authentication. */
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
