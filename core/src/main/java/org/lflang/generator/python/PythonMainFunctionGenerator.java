package org.lflang.generator.python;

/**
 * Responsible for creating the main function for the generated Python target programs.
 *
 * @author Soroush Bateni
 * @ingroup Generator
 */
public final class PythonMainFunctionGenerator {

  /*
   * Generate the main function code
   */
  public static String generateCode() {
    StringBuilder code = new StringBuilder();
    code.append(
        "# The main function\n"
            + "def main(argv):\n"
            + "    start(argv)\n"
            + "    # Suppress Python shutdown errors\n"
            + "    # Workaround for https://github.com/lf-lang/lingua-franca/issues/1906\n"
            + "    import atexit\n"
            + "    atexit.register(os._exit, 0)\n"
            + "\n"
            + "# As is customary in Python programs, the main() function\n"
            + "# should only be executed if the main module is active.\n"
            + "if __name__==\"__main__\":\n"
            + "    main(sys.argv)\n");
    return code.toString();
  }
}
