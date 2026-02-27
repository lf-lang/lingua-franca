package org.lflang.generator.c;

import static org.lflang.ast.ASTUtils.allMethods;

import org.lflang.InferredType;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Method;
import org.lflang.lf.Reactor;
import org.lflang.util.StringUtil;

/**
 * Collection of functions to generate C code to declare methods.
 *
 * @author Edward A. Lee
 * @ingroup Generator
 */
public class CMethodGenerator {

  /**
   * Generate macro definitions for methods.
   *
   * @param tpr The reactor.
   * @param body The place to put the macro definitions.
   */
  public static void generateMacrosForMethods(TypeParameterizedReactor tpr, CodeBuilder body) {
    for (Method method : allMethods(tpr.reactor())) {
      var functionName = methodFunctionName(tpr, method);
      // If the method has no arguments. Do not pass it any variadic arguments.
      if (method.getArguments().size() > 0) {
        body.pr("#define " + method.getName() + "(...) " + functionName + "(self, ##__VA_ARGS__)");
      } else {
        body.pr("#define " + method.getName() + "() " + functionName + "(self)");
      }
    }
  }

  /**
   * Generate macro undefinitions for methods.
   *
   * @param reactor The reactor.
   * @param body The place to put the macro definitions.
   */
  public static void generateMacroUndefsForMethods(Reactor reactor, CodeBuilder body) {
    for (Method method : allMethods(reactor)) {
      body.pr("#undef " + method.getName());
    }
  }

  /**
   * Generate a method function definition for a reactor. This function will have a first argument
   * that is a void* pointing to the self struct, followed by any arguments given in its definition.
   *
   * @param method The method.
   * @param tpr The concrete reactor class.
   * @param types The C-specific type conversion functions.
   * @param suppressLineDirectives Whether to suppress the generation of line directives.
   */
  public static String generateMethod(
      Method method, TypeParameterizedReactor tpr, CTypes types, boolean suppressLineDirectives) {
    var code = new CodeBuilder();
    var body = ASTUtils.toText(method.getCode());

    code.prSourceLineNumber(method, suppressLineDirectives);
    // Define macros for functions such as lf_tag(), lf_time_logical(), lf_set(), etc.
    code.pr("#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetHeader()));

    code.prComment("Implementation of method " + method.getName() + "()");
    code.pr(generateMethodSignature(method, tpr, types) + " {");
    code.indent();

    // Define the "self" struct.
    String structType = CUtil.selfType(tpr);
    // A null structType means there are no inputs, state,
    // or anything else. No need to declare it.
    if (structType != null) {
      code.pr(
          String.join(
              "\n",
              structType
                  + "* self = ("
                  + structType
                  + "*)instance_args;"
                  + " SUPPRESS_UNUSED_WARNING(self);"));
    }
    code.prSourceLineNumber(method.getCode(), suppressLineDirectives);
    code.pr(body);
    code.unindent();
    code.pr("}");
    code.pr("#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetUndefHeader()));
    code.prEndSourceLineNumber(suppressLineDirectives);
    return code.toString();
  }

  /**
   * Generate method functions definition for a reactor. These functions have a first argument that
   * is a void* pointing to the self struct.
   *
   * @param tpr The reactor.
   * @param code The place to put the code.
   * @param types The C-specific type conversion functions.
   * @param suppressLineDirectives Whether to suppress the generation of line directives.
   */
  public static void generateMethods(
      TypeParameterizedReactor tpr,
      CodeBuilder code,
      CTypes types,
      boolean suppressLineDirectives) {
    var reactor = tpr.reactor();
    code.prComment("***** Start of method declarations.");
    signatures(tpr, code, types);
    generateMacrosForMethods(tpr, code);
    for (Method method : allMethods(reactor)) {
      code.pr(CMethodGenerator.generateMethod(method, tpr, types, suppressLineDirectives));
    }
    generateMacroUndefsForMethods(reactor, code);
    code.prComment("***** End of method declarations.");
  }

  /**
   * Generate function signatures for methods. This can be used to declare all the methods with
   * signatures only before giving the full definition so that methods may call each other (and
   * themselves) regardless of the order of definition.
   *
   * @param tpr The reactor declaration.
   * @param body The code builder for the body.
   * @param types The C-specific type conversion functions.
   */
  public static void signatures(TypeParameterizedReactor tpr, CodeBuilder body, CTypes types) {
    Reactor reactor = tpr.reactor();
    for (Method method : allMethods(reactor)) {
      body.pr(generateMethodSignature(method, tpr, types) + ";");
    }
  }

  /**
   * Return the function name for specified method of the specified reactor.
   *
   * @param tpr The reactor
   * @param method The method.
   * @return The function name for the method.
   */
  private static String methodFunctionName(TypeParameterizedReactor tpr, Method method) {
    return CUtil.getName(tpr) + "_method_" + method.getName();
  }

  /**
   * Generate a method function signature for a reactor. This function will have a first argument
   * that is a void* pointing to the self struct, followed by any arguments given in its definition.
   *
   * @param method The method.
   * @param tpr The reactor declaration.
   * @param types The C-specific type conversion functions.
   */
  public static String generateMethodSignature(
      Method method, TypeParameterizedReactor tpr, CTypes types) {
    var functionName = methodFunctionName(tpr, method);

    StringBuilder result = new StringBuilder();
    if (method.getReturn() != null) {
      result.append(types.getTargetType(InferredType.fromAST(method.getReturn())));
      result.append(" ");
    } else {
      result.append("void ");
    }
    result.append(functionName);
    result.append("(void* instance_args");
    if (method.getArguments() != null) {
      for (var arg : method.getArguments()) {
        result.append(", ");
        result.append(types.getTargetType(InferredType.fromAST(arg.getType())));
        result.append(" ");
        result.append(arg.getName());
      }
    }
    result.append(")");
    return result.toString();
  }
}
