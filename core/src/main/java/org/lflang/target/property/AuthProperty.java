package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/** Directive to allow including OpenSSL libraries and process HMAC authentication. */
public final class AuthProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final AuthProperty INSTANCE = new AuthProperty();

  private AuthProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public String name() {
    return "auth";
  }
}
