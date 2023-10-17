package org.lflang.target.property;

/**
 * If true, the resulting binary will output a yaml file describing the whole reactor structure of
 * the program.
 *
 * <p>This option is currently only used for C++. This export function is a valuable tool for
 * debugging LF programs and performing external analysis.
 */
public final class ExportToYamlProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final ExportToYamlProperty INSTANCE = new ExportToYamlProperty();

  private ExportToYamlProperty() {
    super();
  }

  @Override
  public String name() {
    return "export-to-yaml";
  }
}
