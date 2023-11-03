package org.lflang.generator;

import org.lflang.MessageReporter;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TargetProperty;

public record Argument<T>(TargetProperty<T, ?> property, T value) {

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
