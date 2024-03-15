package org.lflang.generator.python;

import java.util.regex.Pattern;
import org.lflang.InferredType;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.ParameterReference;

public class PythonTypes extends CTypes {

  // Regular expression pattern for pointer types. The star at the end has to be visible.
  static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");
  private static final PythonTypes INSTANCE = new PythonTypes();

  @Override
  public String getTargetUndefinedType() {
    return "PyObject*";
  }

  /**
   * This generator inherits types from the CGenerator. This function reverts them back to Python
   * types For example, the types double is converted to float, the * for pointer types is removed,
   * etc.
   *
   * @param type The type
   * @return The Python equivalent of a C type
   */
  public String getPythonType(InferredType type) {
    var result = super.getTargetType(type);

    result =
        switch (result) {
          case "double" -> "float";
          case "string" -> "object";
          default -> result;
        };

    var matcher = pointerPatternVariable.matcher(result);
    if (matcher.find()) {
      return matcher.group(1);
    }

    return result;
  }

  @Override
  public String getTargetParamRef(ParameterReference expr, InferredType typeOrNull) {
    return "self." + expr.getParameter().getName();
  }

  public static PythonTypes getInstance() {
    return INSTANCE;
  }

  public static PythonTypes generateParametersIn(ReactorInstance instance) {
    return new PythonTypes() {
      @Override
      public String getTargetParamRef(ParameterReference expr, InferredType typeOrNull) {
        return PyUtil.reactorRef(instance) + "." + expr.getParameter().getName();
      }
    };
  }
}
