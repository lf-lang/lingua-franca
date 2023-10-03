package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive to let the runtime export the program structure to a yaml file.
 *
 * <p>This is a debugging feature and currently only used for C++ programs.
 */
public class ExportToYamlProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "export-to-yaml";
  }
}
