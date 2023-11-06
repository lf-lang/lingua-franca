package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

public abstract class OptionsType<T extends Enum<?>> implements TargetPropertyType {

  public final List<T> optionsList() {
    return Arrays.stream(enumClass().getEnumConstants()).collect(Collectors.toList());
  }

  protected abstract Class<T> enumClass();

  @Override
  public boolean validate(Element e) {
    if (e.getKeyvalue() != null) {
      return false; // Not a literal.
    }
    return optionsList().stream()
        .anyMatch(v -> v.toString().equalsIgnoreCase(ASTUtils.elementToSingleString(e)));
  }

  public String optionsString() {
    return optionsList().stream().map(v -> v.toString()).collect(Collectors.joining(", "));
  }

  @Override
  public boolean check(Element e, String name, MessageReporter r) {
    var valid = this.validate(e);
    if (!valid) {
      r.at(e).error(String.format("%s is required to be %s.", name, this));
      return false;
    } else {
      return true;
    }
  }

  @Override
  public String toString() {
    return "a choice of " + this.optionsString();
  }

  /**
   * Return option among those listed that matches the given name.
   *
   * @param name The string to match against.
   * @return The matching option (or null if there is none).
   */
  public T forName(String name) {
    var match = optionsList().stream().filter(o -> o.toString().equalsIgnoreCase(name)).findFirst();
    if (match.isPresent()) {
      return match.get();
    }
    return null;
  }
}
