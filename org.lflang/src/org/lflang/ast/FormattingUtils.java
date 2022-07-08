package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.ToLongFunction;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.ecore.EObject;

import org.lflang.ASTUtils;
import org.lflang.lf.Model;

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

    /** Match a multiline comment. */
    private static final Pattern MULTILINE_COMMENT = Pattern.compile(
        "\\s*/\\*\\v?(\\V*\\v+)*\\V*"
    );

    /** The number of spaces to prepend to a line per indentation level. */
    private static final int INDENTATION = 4;

    public static final int DEFAULT_LINE_LENGTH = 80;

    static final long BADNESS_PER_CHARACTER_VIOLATING_LINE_LENGTH = 20;

    static final long BADNESS_PER_LEVEL_OF_COMMENT_DISPLACEMENT = 1000;

    static final long BADNESS_PER_NEWLINE = 1;

    /**
     * Return a String representation of {@code object}, with lines wrapped at
     * {@code lineLength}.
     */
    public static String render(EObject object, int lineLength) {
        MalleableString ms = ToLf.instance.doSwitch(object);
        String singleLineCommentPrefix = getSingleLineCommentPrefix(object);
        ms.findBestRepresentation(
            () -> ms.render(INDENTATION, singleLineCommentPrefix),
            r -> r.levelsOfCommentDisplacement() * BADNESS_PER_LEVEL_OF_COMMENT_DISPLACEMENT
                + countCharactersViolatingLineLength(lineLength).applyAsLong(r.rendering())
                    * BADNESS_PER_CHARACTER_VIOLATING_LINE_LENGTH
                + countNewlines(r.rendering()) * BADNESS_PER_NEWLINE,
            lineLength,
            INDENTATION,
            singleLineCommentPrefix
        );
        var optimizedRendering = ms.render(INDENTATION, singleLineCommentPrefix);
        List<String> comments = optimizedRendering.unplacedComments().toList();
        return comments.stream().allMatch(String::isBlank) ? optimizedRendering.rendering()
            : lineWrapComments(comments, lineLength, singleLineCommentPrefix)
                + "\n" + optimizedRendering.rendering();
    }

    private static String getSingleLineCommentPrefix(EObject object) {
        if (object instanceof Model model) {
            var targetDecl = ASTUtils.targetDecl(model);
            if (targetDecl != null && targetDecl.getName().toUpperCase().contains("PYTHON")) {
                return "#";
            }
        }
        return "//";
    }

    /**
     * Return a String representation of {@code object} using a reasonable
     * default line length.
     */
    public static String render(EObject object) { return render(object, DEFAULT_LINE_LENGTH); }

    private static ToLongFunction<String> countCharactersViolatingLineLength(int lineLength) {
        return s -> s.lines().mapToInt(it -> Math.max(0, it.length() - lineLength)).sum();
    }

    private static long countNewlines(String s) {
        return s.lines().count();
    }

    /**
     * Break lines at spaces so that each line is no more than {@code width}
     * columns long, if possible. Normalize whitespace.
     */
    static String lineWrapComments(
        List<String> comments,
        int width,
        String singleLineCommentPrefix
    ) {
        StringBuilder ret = new StringBuilder();
        StringBuilder current = new StringBuilder();
        for (String comment : comments) {
            if (comment.stripLeading().startsWith("/*")) {
                ret.append(lineWrapComment(current.toString(), width, singleLineCommentPrefix))
                    .append("\n");
                current.setLength(0);
                ret.append(lineWrapComment(comment, width, singleLineCommentPrefix)).append("\n");
            } else {
                current.append(comment).append("\n");
            }
        }
        if (!current.isEmpty()) {
            ret.append(lineWrapComment(current.toString(), width, singleLineCommentPrefix));
        } else if (!ret.isEmpty()) {
            ret.deleteCharAt(ret.length() - 1);  // Delete final newline
        }
        return ret.toString();
    }
    static String lineWrapComment(String comment, int width, String singleLineCommentPrefix) {
        width = Math.max(width, MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
        List<List<String>> paragraphs = Arrays.stream(
            comment.strip()
                .replaceAll("^/?((\\*|//|#)\\s*)+", "")
                .replaceAll("\\s*\\*/$", "")
                .replaceAll("(?<=" + System.lineSeparator() + ")\\h*(\\*|//|#)\\h*", "")
                .split(String.format("(%n\\s*){2,}"))
            ).map(s -> Arrays.stream(s.split("((\r\n?)|\n)\\s*(?=[@#$%^&*\\-+=:;<>/])")))
            .map(stream -> stream.map(s -> s.replaceAll("\\s+", " ")))
            .map(Stream::toList)
            .toList();
        if (MULTILINE_COMMENT.matcher(comment).matches()) {
            if (
                comment.length() < width && paragraphs.size() == 1 && paragraphs.get(0).size() == 1
            ) {
                return String.format("/** %s */", paragraphs.get(0).get(0));
            }
            return String.format("/**%n%s%n */", lineWrapComment(paragraphs, width, " * "));
        }
        return lineWrapComment(paragraphs, width, singleLineCommentPrefix + " ");
    }

    static String lineWrapComment(
        List<List<String>> paragraphs,
        int width,
        String linePrefix
    ) {
        int widthAfterPrefix = Math.max(
            width - linePrefix.length(),
            MINIMUM_COMMENT_WIDTH_IN_COLUMNS
        );
        return paragraphs.stream()
            .map(paragraph -> wrapLines(paragraph, widthAfterPrefix)
                .map(s -> (linePrefix + s.stripLeading()).stripTrailing())
                .collect(Collectors.joining(System.lineSeparator()))
            ).collect(Collectors.joining(String.format("%n%s%n", linePrefix.stripTrailing())));
    }

    static Stream<String> wrapLines(List<String> subparagraphs, int width) {
        var ret = new ArrayList<String>();
        for (String s : subparagraphs) {
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
     *
     */
    static boolean placeComment(
        List<String> comment,
        List<String> components,
        int i,
        int width,
        boolean keepCommentsOnSameLine,
        String singleLineCommentPrefix
    ) {
        if (comment.stream().allMatch(String::isBlank)) return true;
        String wrapped = FormattingUtils.lineWrapComments(comment, width, singleLineCommentPrefix);
        if (keepCommentsOnSameLine && wrapped.lines().count() == 1 && !wrapped.startsWith("/**")) {
            for (int j = i; j < components.size(); j++) {
                if (components.get(j).contains(System.lineSeparator())) {
                    components.set(j, components.get(j).replaceFirst(
                        System.lineSeparator(),
                        String.format("  %s%n", wrapped)
                    ));
                    return true;
                }
            }
        }
        for (int j = i - 1; j >= 0; j--) {
            if (components.get(j).endsWith(System.lineSeparator())) {
                components.set(j, String.format("%s%s%n", components.get(j), wrapped));
                return true;
            }
        }
        return false;
    }

    static String normalizeEol(String s) {
        return s.replaceAll("(\\r\\n?)|\\n", System.lineSeparator());
    }
}
