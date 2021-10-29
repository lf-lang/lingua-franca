package org.lflang.generator.python;

public final class PythonMainGenerator {
    
    public static String generateCode() {
        StringBuilder code = new StringBuilder();
        code.append(
                  "# The main function\n"
                + "def main(argv):\n"
                + "    start(argv)\n"
                + "\n"
                + "# As is customary in Python programs, the main() function\n"
                + "# should only be executed if the main module is active.\n"
                + "if __name__==\"__main__\":\n"
                + "    main(sys.argv)\n"
       );
       return code.toString();
    }
}
