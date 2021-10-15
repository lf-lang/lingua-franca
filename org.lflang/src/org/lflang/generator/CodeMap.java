package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.HashMap;
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
            "/\\*Correspondence: (?<lfRange>%s) \\-> (?<generatedRange>%s) \\(src=(?<path>%s)\\)\\*/\n",
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
                "/*Correspondence: %s -> %s (src=%s)*/\n",
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
            return fromString(s, 0);
        }

        /**
         * Returns the Correspondence represented by <code>s
         * </code>.
         * @param s an arbitrary String
         * @param relativeTo the offset relative to which
         *                   the offsets given are given
         * @return the Correspondence represented by <code>s
         * </code>
         */
        @NotNull
        public static Correspondence fromString(String s, int relativeTo) {
            Matcher matcher = PATTERN.matcher(s);
            if (matcher.matches()) {
                Range lfRange = Range.fromString(matcher.group("lfRange"));
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
        Map<Path, NavigableMap<Range, Range>> map = new HashMap<>();
        // The following is not pretty, and it uses
        // extra space, but it seems the alternative is to
        // use some kind of mutable integer as a workaround.
        Matcher matcher = Correspondence.PATTERN.matcher(internalGeneratedCode);
        int removedCharsCount = 0;
        List<Integer> inclusionToggles = new ArrayList<>();
        inclusionToggles.add(0);
        while (matcher.find()) {
            Correspondence c = Correspondence.fromString(matcher.group(), matcher.start() - removedCharsCount);
            if (!map.containsKey(c.path)) map.put(c.path, new TreeMap<>());
            map.get(c.path).put(c.generatedRange, c.lfRange);
            removedCharsCount += matcher.end() - matcher.start();
            inclusionToggles.add(matcher.start());
            inclusionToggles.add(matcher.end());
        }
        return new CodeMap(getSelectedSubstrings(internalGeneratedCode, inclusionToggles), map);
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
     * if such a position is known, or 0 otherwise.
     * @param lfFile the path to an arbitrary Lingua Franca
     *               source file
     * @param generatedFilePosition a position in a
     *                              generated file
     * @return the position in <code>lfFile</code>
     * corresponding to <code>generatedFilePosition</code>,
     * or 0 otherwise
     */
    public int adjusted(Path lfFile, int generatedFilePosition) {
        NavigableMap<Range, Range> mapOfInterest = map.get(lfFile);
        Map.Entry<Range, Range> nearestEntry = mapOfInterest.floorEntry(Range.degenerateRange(generatedFilePosition));
        if (nearestEntry == null) return 0;
        if (nearestEntry.getKey().contains(generatedFilePosition)) {
            return nearestEntry.getKey().getStartInclusive() + generatedFilePosition;
        }
        return 0;
    }

    public Position adjusted(Path lfFile, Position generatedFilePosition, Path generatedFile) {
        String generatedFileContent;
        String lfFileContent;
        try {
            generatedFileContent = Files.readString(generatedFile);
        } catch (IOException e) {
            System.err.println("Failed to read the contents of the generated file " + generatedFile);
            return Position.ORIGIN;
        }
        try {
            lfFileContent = Files.readString(lfFile);
        } catch (IOException e) {
            System.err.println("Failed to read the contents of the LF source file " + lfFile);
            return Position.ORIGIN;
        }
        return Position.fromOffset(
            adjusted(lfFile, generatedFilePosition.getOffset(generatedFileContent)),
            lfFileContent
        );
    }

    public Range adjusted(Path lfFile, Range generatedFileRange) {
        return new Range(
            adjusted(lfFile, generatedFileRange.getStartInclusive()),
            adjusted(lfFile, generatedFileRange.getEndExclusive())
        );
    }

    /* ------------------------- PRIVATE METHODS ------------------------- */

    private CodeMap(String generatedCode, Map<Path, NavigableMap<Range, Range>> map) {
        this.generatedCode = generatedCode;
        this.map = map;
    }

    /**
     * Returns the substrings selected by
     * <code>inclusionToggles</code>.
     * @param s an arbitrary String
     * @param inclusionToggles the indices at which to
     *                         toggle between including and
     *                         excluding characters. Initial
     *                         state is including.
     * @return the substrings selected by
     * <code>inclusionToggles</code>
     */
    private static String getSelectedSubstrings(String s, List<Integer> inclusionToggles) {
        StringBuilder ret = new StringBuilder();
        int i;
        for (i = 0; i < inclusionToggles.size() - 1; i += 2) {
            ret.append(s, inclusionToggles.get(i), inclusionToggles.get(i + 1));
        }
        if (i < inclusionToggles.size()) ret.append(s.substring(inclusionToggles.get(i)));
        return ret.toString();
    }
}
