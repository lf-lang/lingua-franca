package org.lflang.generator.c;

public class CMainGenerator {
    public static String generateCode() {
        return String.join("\n",
            "int main(int argc, char* argv[]) {",
            "    return lf_reactor_c_main(argc, argv);",
            "}"
        );
    }
}
