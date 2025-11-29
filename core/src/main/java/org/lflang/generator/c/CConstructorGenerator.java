package org.lflang.generator.c;

import org.lflang.generator.CodeBuilder;

/**
 * Generates C constructor code for a reactor.
 *
 * @ingroup Generator
 */
public class CConstructorGenerator {
  /**
   * Generate a constructor for the specified reactor in the specified federate.
   *
   * @param tpr The type-parameterized reactor.
   * @param constructorCode Lines of code previously generated that need to go into the constructor.
   */
  public static String generateConstructor(TypeParameterizedReactor tpr, String constructorCode) {
    var structType = CUtil.selfType(tpr);
    var code = new CodeBuilder();
    code.pr(structType + "* new_" + CUtil.getName(tpr) + "() {");
    code.indent();
    code.pr(
        structType + "* self = (" + structType + "*)lf_new_reactor(sizeof(" + structType + "));");
    code.pr(constructorCode);
    code.pr("return self;");
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  public static String generateConstructorPrototype(TypeParameterizedReactor tpr) {
    return CUtil.selfType(tpr) + "* new_" + CUtil.getName(tpr) + "();";
  }
}
