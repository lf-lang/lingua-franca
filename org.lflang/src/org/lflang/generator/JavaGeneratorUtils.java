package org.lflang.generator;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.CharSequence;

/**
 * A helper class with functions that may be useful for code
 * generators.
 * This is created to ease our transition from Xtend and
 * possibly Eclipse. All functions in this class should
 * instead by in GeneratorUtils.kt, but Eclipse cannot
 * handle Kotlin files.
 */
public class JavaGeneratorUtils {

    private JavaGeneratorUtils() {
        // utility class
    }

    /**
     * Write the source code to file.
     * @param code The code to be written.
     * @param path The file to write the code to.
     */
    public static void writeSourceCodeToFile(CharSequence code, String path) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            for (int i = 0; i < code.length(); i++) {
                writer.write(code.charAt(i));
            }
        }
    }
}
