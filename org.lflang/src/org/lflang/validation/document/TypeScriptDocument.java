package org.lflang.validation.document;

import static org.lflang.generator.ts.sourcemap.SourceMapKt.getSourceMapOf;
import org.lflang.generator.ts.sourcemap.SourceMap;
import org.lflang.generator.ts.sourcemap.SourceMapSegment;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class TypeScriptDocument extends GeneratedDocument {

    private static final Pattern ESLINT_ERROR = Pattern.compile(""); // TODO

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    public TypeScriptDocument(List<String> lines, File directory) {
        super(lines, getMap(lines, directory), directory);
    }

    /* ---------------------  PROTECTED METHODS  ------------------------ */

    @Override
    protected ProcessBuilder getVerificationProcess() {
        return null;
    }


    @Override
    protected void addDiagnostic(String line, DiagnosticAcceptor acceptor) {
        // TODO
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /**
     * Returns a <code>NavigableMap</code> with mappings
     * from the content of a generated TypeScript document
     * to the content of the source code from which it
     * was generated.
     * @param lines the contents of the TypeScript document
     *              of interest
     * @param directory the directory in which the
     *                  TypeScript document lives
     * @return a <code>NavigableMap</code> with mappings
     * from the content of a generated TypeScript document
     * to the content of the source code from which it
     * was generated
     */
    private static NavigableMap<Position, Position> getMap(List<String> lines, File directory) {
        NavigableMap<Position, Position> ret = new TreeMap<>();
        SourceMap sourceMap;
        try {
            sourceMap = getSourceMapOf(lines, directory);
        } catch (IOException e) {
            return ret;
        }
        if (sourceMap != null && sourceMap.getMappings() != null) {
            for (SourceMapSegment segment : sourceMap.getMappings()) {
                ret.put(
                    Position.fromZeroBased(segment.getTargetLine(), segment.getTargetColumn()),
                    Position.fromZeroBased(segment.getSourceLine(), segment.getSourceColumn())
                );
            }
        }
        return ret;
    }
}
