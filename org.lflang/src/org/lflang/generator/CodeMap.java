package org.lflang.generator;

import java.nio.file.Path;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jetbrains.annotations.NotNull;

/**
 * Encapsulates data about the correspondence between
 * ranges of generated code and ranges of a Lingua Franca
 * file.
 */
public class CodeMap {

    public static class Correspondence {
        // This pattern has the markers /* */ that some languages use as line comments. This is not part
        //  of any serious effort to make it possible to embed the string representation of this in code
        //  without rendering it invalid. Instead, it is done simply because it is easy.
        private static final Pattern PATTERN = Pattern.compile(String.format(
            "/\\*Correspondence: (?<lfRange>%s) \\-> (?<generatedRange>%s) \\(src=(?<path>%s)\\)\\*/",
            Position.removeNamedCapturingGroups(Range.PATTERN),
            Position.removeNamedCapturingGroups(Range.PATTERN),
            ".*"
        ));

        private final Path path;
        private final Range lfRange;
        private final Range generatedRange;

        public Correspondence(Path path, Range lfRange, Range generatedRange) {
            this.path = path;
            this.lfRange = lfRange;
            this.generatedRange = generatedRange;
        }

        public Path getPath() {
            return path;
        }

        public Range getLfRange() {
            return lfRange;
        }

        public Range getGeneratedRange() {
            return generatedRange;
        }

        @Override
        public String toString() {
            return String.format(
                "/*Correspondence: %s -> %s (src=%s)*/",
                lfRange.toString(), generatedRange.toString(), path.toString()
            );
        }

        /**
         * Returns the Correspondence represented by <code>s
         * </code>.
         * @param s an arbitrary String
         * @return the Correspondence represented by <code>s
         * </code>
         */
        @NotNull
        public static Correspondence fromString(String s) {
            return fromString(s, Position.fromZeroBased(0, 0));
        }

        /**
         * Returns the Correspondence represented by <code>s
         * </code>.
         * @param s an arbitrary String
         * @param relativeTo the position relative to which
         *                   the positions given are given
         * @return the Correspondence represented by <code>s
         * </code>
         */
        @NotNull
        public static Correspondence fromString(String s, Position relativeTo) {
            Matcher matcher = PATTERN.matcher(s);
            if (matcher.matches()) {
                Range lfRange = Range.fromString(matcher.group("lfRange"), relativeTo);
                Range generatedRange = Range.fromString(matcher.group("generatedRange"), relativeTo);
                return new Correspondence(
                    Path.of(matcher.group("path")),
                    lfRange, generatedRange
                );
            }
            throw new IllegalArgumentException(String.format("Could not parse %s as a Correspondence.", s));
        }
    }

    /**
     * The content of the generated file represented by
     * this.
     */
    private final String generatedCode;
    /**
     * A mapping from Lingua Franca source paths to mappings
     * from ranges in the generated file represented by this
     * to ranges in Lingua Franca files.
     */
    private final Map<Path, NavigableMap<Range, Range>> map;

    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Instantiates a <code>CodeMap</code> from
     * <code>internalGeneratedCode</code>.
     * <code>internalGeneratedCode</code> may be invalid
     * code that is different from the final generated code
     * because it should contain deserializable
     * representations of <code>Correspondences</code>
     * @param internalGeneratedCode code from a code
     *                              generator that contains
     *                              serialized
     *                              Correspondences
     * @return a CodeMap documenting the provided code
     */
    public static CodeMap fromGeneratedCode(String internalGeneratedCode) {
        StringBuilder generatedCode = new StringBuilder();
        Map<Path, NavigableMap<Range, Range>> map = new TreeMap<>();
        int lineNumber = 0;
        // The following is not pretty, and it uses
        // extra space, but it seems the alternative is to
        // use some kind of mutable integer as a workaround.
        for (String line : internalGeneratedCode.lines().toArray(String[]::new)) {
            Matcher matcher = Correspondence.PATTERN.matcher(line);
            while (matcher.find()) {
                Correspondence c = Correspondence.fromString(line, Position.fromZeroBased(lineNumber, matcher.start()));
                map.get(c.path).put(c.generatedRange, c.lfRange);
                line = line.substring(0, matcher.start()) + line.substring(matcher.end());
                matcher = Correspondence.PATTERN.matcher(line);
            }
            generatedCode.append(line);
            lineNumber++;
        }
        return new CodeMap(generatedCode.toString(), map);
    }

    /**
     * Returns the generated code (without Correspondences).
     * @return the generated code (without Correspondences)
     */
    public String getGeneratedCode() {
        return generatedCode;
    }

    /**
     * Returns the position in <code>lfFile</code>
     * corresponding to <code>generatedFilePosition</code>
     * if such a position is known, or null otherwise.
     * @param lfFile the path to an arbitrary Lingua Franca
     *               source file
     * @param generatedFilePosition a position in
     * @return
     */
    public Position adjusted(Path lfFile, Position generatedFilePosition) {
        NavigableMap<Range, Range> mapOfInterest = map.get(lfFile);
        Map.Entry<Range, Range> nearestEntry = mapOfInterest.floorEntry(Range.degenerateRange(generatedFilePosition));
        if (nearestEntry.getKey().contains(generatedFilePosition)) {
            return nearestEntry.getKey().getStartInclusive().translated(
                nearestEntry.getValue().lineDelta(generatedFilePosition),
                nearestEntry.getValue().columnDelta(generatedFilePosition)
            );
        }
        return null;
    }

    public Range adjusted(Path lfFile, Range generatedFileRange) {
        return new Range(
            adjusted(lfFile, generatedFileRange.getEndExclusive()),
            adjusted(lfFile, generatedFileRange.getEndExclusive())
        );
    }

    /* ------------------------- PRIVATE METHODS ------------------------- */

    private CodeMap(String generatedCode, Map<Path, NavigableMap<Range, Range>> map) {
        this.generatedCode = generatedCode;
        this.map = map;
    }
}
