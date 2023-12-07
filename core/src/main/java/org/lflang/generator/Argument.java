package org.lflang.generator;

import org.lflang.MessageReporter;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TargetProperty;

/**
 * A record that ties a target property to a value obtained as a parameter to a compilation run.
 *
 * @param property A target property.
 * @param value The value to assign to it.
 * @param <T> The type of the value.
 * @author Marten Lohstroh
 */
public record Argument<T>(TargetProperty<T, ?> property, T value) {

  /**
   * Update the target configuration if the value of this argument is not {@code null}.
   *
   * @param config The target configuration to update.
   * @param reporter An error reporter to report the use of unsupported target properties.
   */
  public void update(TargetConfig config, MessageReporter reporter) {
    if (value != null) {
      if (!config.forName(property.name()).isPresent()) {
        config.reportUnsupportedTargetProperty(property().name(), reporter.nowhere());
      } else {
        property.update(config, value);
      }
    }
  }
}
