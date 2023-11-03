package org.lflang.target.property;

/** Directive to allow including OpenSSL libraries and process HMAC authentication. */
public final class AuthProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final AuthProperty INSTANCE = new AuthProperty();

  private AuthProperty() {
    super();
  }

  @Override
  public String name() {
    return "auth";
  }
}
