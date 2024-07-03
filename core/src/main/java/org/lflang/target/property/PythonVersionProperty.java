package org.lflang.target.property;

/** Directive for specifying a specific version of the reactor runtime library. */
public final class PythonVersionProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final PythonVersionProperty INSTANCE = new PythonVersionProperty();

  private PythonVersionProperty() {
    super();
  }

  @Override
  public String name() {
    return "python-version";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    String version = config.get(PythonVersionProperty.INSTANCE);
    if (!version.isEmpty() && !version.contains("3.10")) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__NAME)
          .warning(
              "Python "
                  + version
                  + " is currently unsupported and untested. As such, it may fail in unexpected"
                  + " ways.");
    }
  }
}
