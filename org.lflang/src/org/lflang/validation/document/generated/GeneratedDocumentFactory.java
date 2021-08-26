package org.lflang.validation.document.generated;

import java.io.File;
import java.util.List;

/**
 * Produces the appropriate <code>GeneratedDocument</code>
 * for a given generated file.
 */
public class GeneratedDocumentFactory {

    /**
     * Returns a GeneratedDocument instance representing
     * <code>f</code> if <code>f</code> is a file in a
     * supported language; otherwise, returns
     * <code>null</code>.
     * @param lines the contents of a generated File
     * @param extension the file extension of the generated
     *                  file
     * @return a GeneratedDocument instance representing
     * <code>f</code> if <code>f</code> is a file in a
     * supported language, or <code>null</code> otherwise
     */
    public static GeneratedDocument getGeneratedDocument(List<String> lines, File directory, String extension) {
        switch (extension) {
        case "c":
        case "cpp":
            return CCppDocument.getCCppDocument(lines, directory, extension);
        case "ts":
            return new TypeScriptDocument(lines, directory);
        default:
            return null;
        }
    }
}
