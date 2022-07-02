package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.ecore.EObject;

/**
 * Utility functions that determine the specific behavior of the LF formatter.
 * @author {Peter Donovan <peterdonovan@berkeley.edu>}
 * @author {Billy Bao <billybao@berkeley.edu>}
 */
public class FormattingUtils {
    /**
     * The minimum number of columns that should be allotted to a comment.
     * This is relevant in case of high indentation/small wrapLength.
     */
    private static final int MINIMUM_COMMENT_WIDTH_IN_COLUMNS = 15;

    /** Match a multiline comment with lines starting with stars. */
    private static final Pattern MULTILINE_COMMENT_LINES_STARTING_WITH_STARS = Pattern.compile(
        "\\s*/(\\s*\\*(\\S*\\h*)*)+"
    );

    // TODO: Ideally, ToLf would not need to access the value of INDENTATION.
    /** The number of spaces to prepend to a line per indentation level. */
    static final int INDENTATION = 4;

    public static final int DEFAULT_LINE_LENGTH = 80;

    /**
     * Return a String representation of {@code object}, with lines wrapped at
     * {@code lineLength}.
     */
    public static String render(EObject object, int lineLength) {
        // The following looks useless, but it wraps the representation in an
        // enclosing object that ensures that top-level comments are rendered.
        MalleableString ms = new MalleableString.Builder()
            .append(ToLf.instance.doSwitch(object))
            .get();
        ms.findBestRepresentation(
            ms::toString,
            astRepresentationComparator(lineLength),
            lineLength
        );
        return ms.toString();
    }

    /**
     * Return a String representation of {@code object} using a reasonable
     * default line length.
     */
    public static String render(EObject object) { return render(object, DEFAULT_LINE_LENGTH); }

    private static Comparator<String> astRepresentationComparator(int lineLength) {
        return Comparator.comparing(countCharactersViolatingLineLength(lineLength))
            .thenComparing(FormattingUtils::countNewlines);
    }

    private static Function<String, Integer> countCharactersViolatingLineLength(int lineLength) {
        return s -> s.lines().mapToInt(it -> Math.max(0, it.length() - lineLength)).sum();
    }

    private static long countNewlines(String s) {
        return s.lines().count();
    }

    /**
     * Break lines at spaces so that each line is no more than {@code width}
     * columns long, if possible. Normalize whitespace.
     */
    static String lineWrapComment(String comment, int width) {
        width = Math.max(width, MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
        List<String> simpleWhitespace = Arrays.stream(
            comment.strip()
                .replaceAll("^/?((\\*|//)\\s*)+", "")
                .replaceAll("\\s*\\*/$", "")
                .replaceAll("(?<=" + System.lineSeparator() + ")\\h*(\\*|//)\\h*", "")
                .split(String.format("(%n\\s*){2,}"))
            ).map(s -> s.replaceAll("\\s+", " "))
            .toList();
        if (MULTILINE_COMMENT_LINES_STARTING_WITH_STARS.matcher(comment).matches()) {
            if (comment.length() < width && simpleWhitespace.size() == 1) {
                return String.format("/** %s */", simpleWhitespace.get(0));
            }
            return String.format("/**%n%s%n */", lineWrapComment(simpleWhitespace, width, " * "));
        }
        return lineWrapComment(simpleWhitespace, width, "// ");
    }

    static String lineWrapComment(
        List<String> simpleWhitespace,
        int width,
        String linePrefix
    ) {
        width = Math.max(width - linePrefix.length(), MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
        return wrapLines(simpleWhitespace, width)
            .map(s -> (linePrefix + s.stripLeading()).stripTrailing())
            .collect(Collectors.joining(System.lineSeparator()));
    }

    static Stream<String> wrapLines(List<String> paragraphs, int width) {
        var ret = new ArrayList<String>();
        for (String s : paragraphs) {
            if (!ret.isEmpty()) ret.add("");
            int numCharactersProcessed = 0;
            while (numCharactersProcessed + width < s.length()) {
                // try to wrap at space
                int breakAt = s.lastIndexOf(' ', numCharactersProcessed + width);
                // if unable to find space in limit, extend to the first space we find
                if (breakAt < numCharactersProcessed) {
                    breakAt = s.indexOf(' ', numCharactersProcessed + width);
                }
                if (breakAt < numCharactersProcessed) breakAt = s.length();
                ret.add(s.substring(numCharactersProcessed, breakAt));
                numCharactersProcessed = breakAt + 1;
            }
            if (numCharactersProcessed < s.length()) ret.add(s.substring(numCharactersProcessed));
        }
        return ret.stream();
    }

    /**
     * Merge {@code comment} into the given list of strings without changing the
     * length of the list, preferably in a place that indicates that
     * {@code comment} is associated with the {@code i}th string.
     * @param comment A comment associated with an element of
     * {@code components}.
     * @param components A list of strings that will be rendered in sequence.
     * @param i The position of the component associated with {@code comment}.
     * @param width The ideal number of columns available for comments that
     * appear on their own line.
     * @param keepCommentsOnSameLine Whether to make a best-effort attempt to
     * keep the comment on the same line as the associated string.
     */
    static void placeComment(
        String comment,
        List<String> components,
        int i,
        int width,
        boolean keepCommentsOnSameLine
    ) {
        String wrapped = FormattingUtils.lineWrapComment(comment, width);
        if (comment.isBlank()) return;
        if (keepCommentsOnSameLine && wrapped.lines().count() == 1 && !wrapped.startsWith("/**")) {
            for (int j = i; j < components.size(); j++) {
                if (components.get(j).contains(System.lineSeparator())) {
                    components.set(j, components.get(j).replaceFirst(
                        System.lineSeparator(),
                        String.format(" %s%n", wrapped)
                    ));
                    return;
                }
            }
        }
        for (int j = i - 1; j >= 0; j--) {
            if (components.get(j).endsWith(System.lineSeparator())) {
                components.set(j, String.format("%s%s%n", components.get(j), wrapped));
                return;
            }
        }
        components.set(
            0,
            String.format("%s%n%s", wrapped, components.isEmpty() ? "" : components.get(0))
        );
    }

    static String normalizeEol(String s) {
        return s.replaceAll("\\r\\n?", "\n");
    }
}
