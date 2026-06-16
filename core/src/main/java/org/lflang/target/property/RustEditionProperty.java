package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;

/**
 * Directive to specify the Rust edition at generation time.
 */
public final class RustEditionProperty extends StringProperty {

  // ** Singleton target property instance. */
  public static final RustEditionProperty INSTANCE = new RustEditionProperty();

  // ** List of valid editions */
  private static final List<String> VALID_EDITIONS = List.of("2018", "2021", "2024");

  private RustEditionProperty() {
    super();
  }

  @Override
  public String name() {
    return "edition";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var edition = config.get(RustEditionProperty.INSTANCE);
    if (!edition.isEmpty() && !VALID_EDITIONS.contains(edition)) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
          .error(
              "edition: "
                  + edition
                  + " is not a valid edition. "
                  + "Please use one of the following: "
                  + String.join(", ", VALID_EDITIONS));
    }
  }
}
