package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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

    /**
     * Break lines at spaces so that each line is no more than {@code width}
     * columns long, if possible. Normalize whitespace.
     */
    public static String lineWrapComment(String comment, int width) {
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

    private static String lineWrapComment(
        List<String> simpleWhitespace,
        int width,
        String linePrefix
    ) {
        width = Math.max(width - linePrefix.length(), MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
        return wrapLines(simpleWhitespace, width)
            .map(s -> (linePrefix + s.stripLeading()).stripTrailing())
            .collect(Collectors.joining(System.lineSeparator()));
    }

    private static Stream<String> wrapLines(List<String> paragraphs, int width) {
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
}
