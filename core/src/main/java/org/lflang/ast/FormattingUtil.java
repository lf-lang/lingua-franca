package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.ToLongFunction;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.eclipse.emf.ecore.EObject;
import org.lflang.lf.Model;
import org.lflang.target.Target;

/**
 * Utility functions that determine the specific behavior of the LF formatter.
 *
 * @author Peter Donovan
 * @author Billy Bao
 */
public class FormattingUtil {
  /**
   * The minimum number of columns that should be allotted to a comment. This is relevant in case of
   * high indentation/small wrapLength.
   */
  private static final int MINIMUM_COMMENT_WIDTH_IN_COLUMNS = 15;

  /** Match a multiline comment. */
  private static final Pattern MULTILINE_COMMENT = Pattern.compile("\\s*/\\*\\v?(\\V*\\v+)*\\V*");

  /** The number of spaces to prepend to a line per indentation level. */
  private static final int INDENTATION = 2;

  public static final int DEFAULT_LINE_LENGTH = 100;

  static final int MAX_WHITESPACE_USED_FOR_ALIGNMENT = 20;

  static final long BADNESS_PER_CHARACTER_VIOLATING_LINE_LENGTH = 20;

  static final long BADNESS_PER_LEVEL_OF_COMMENT_DISPLACEMENT = 1000;

  static final long BADNESS_PER_NEWLINE = 1;

  /** Return a String representation of {@code object}, with lines wrapped at {@code lineLength}. */
  public static String render(Model object, int lineLength) {
    return render(object, lineLength, inferTarget(object), false);
  }

  /** Return a function that renders AST nodes for the given target. */
  public static Function<EObject, String> renderer(Target target) {
    return object -> render(object, DEFAULT_LINE_LENGTH, target, true);
  }

  /**
   * Return a String representation of {@code object}, with lines wrapped at {@code lineLength},
   * with the assumption that the target language is {@code target}.
   */
  public static String render(EObject object, int lineLength, Target target, boolean codeMapTags) {
    var toLf = new ToLf();
    MalleableString ms = toLf.doSwitch(object);
    String singleLineCommentPrefix = target.getSingleLineCommentPrefix();
    ms.findBestRepresentation(
        () -> ms.render(INDENTATION, singleLineCommentPrefix, codeMapTags, null),
        r ->
            r.levelsOfCommentDisplacement() * BADNESS_PER_LEVEL_OF_COMMENT_DISPLACEMENT
                + countCharactersViolatingLineLength(lineLength).applyAsLong(r.rendering())
                    * BADNESS_PER_CHARACTER_VIOLATING_LINE_LENGTH
                + countNewlines(r.rendering()) * BADNESS_PER_NEWLINE,
        lineLength,
        INDENTATION,
        singleLineCommentPrefix);
    var optimizedRendering = ms.render(INDENTATION, singleLineCommentPrefix, codeMapTags, null);
    List<String> comments = optimizedRendering.unplacedComments().toList();
    return comments.stream().allMatch(String::isBlank)
        ? optimizedRendering.rendering()
        : lineWrapComments(comments, lineLength, singleLineCommentPrefix)
            + "\n"
            + optimizedRendering.rendering();
  }

  /** Infer the target language of the object. */
  private static Target inferTarget(EObject object) {
    if (object instanceof Model model) {
      var targetDecl = ASTUtils.targetDecl(model);
      if (targetDecl != null) {
        return Target.fromDecl(targetDecl);
      }
    }
    throw new IllegalArgumentException("Unable to determine target based on given EObject.");
  }

  /** Return a String representation of {@code object} using a reasonable default line length. */
  public static String render(Model object) {
    return render(object, DEFAULT_LINE_LENGTH);
  }

  /** Return the number of characters appearing in columns exceeding {@code lineLength}. */
  private static ToLongFunction<String> countCharactersViolatingLineLength(int lineLength) {
    return s -> s.lines().mapToInt(it -> Math.max(0, it.length() - lineLength)).sum();
  }

  private static long countNewlines(String s) {
    return s.lines().count();
  }

  /**
   * Break lines at spaces so that each line is no more than {@code width} columns long, if
   * possible. Normalize whitespace. Merge consecutive single-line comments.
   */
  static String lineWrapComments(List<String> comments, int width, String singleLineCommentPrefix) {
    StringBuilder ret = new StringBuilder();
    StringBuilder current = new StringBuilder();
    for (String comment : comments) {
      if (comment.stripLeading().startsWith("/*")) {
        if (!ret.isEmpty() && !current.isEmpty()) ret.append("\n");
        ret.append(lineWrapComment(current.toString(), width, singleLineCommentPrefix));
        current.setLength(0);
        if (!ret.isEmpty()) ret.append("\n");
        ret.append(lineWrapComment(comment.strip(), width, singleLineCommentPrefix));
      } else {
        if (!current.isEmpty()) current.append("\n");
        current.append(comment.strip());
      }
    }
    if (!ret.isEmpty() && !current.isEmpty()) ret.append("\n");
    ret.append(lineWrapComment(current.toString(), width, singleLineCommentPrefix));
    return ret.toString();
  }

  /** Wrap lines. Do not merge lines that start with weird characters. */
  private static String lineWrapComment(String comment, int width, String singleLineCommentPrefix) {
    var multiline = MULTILINE_COMMENT.matcher(comment).matches();
    comment = comment.strip().replaceAll("(^|(?<=\n))\\h*(/\\*+|//|#)", "");
    if (!multiline)
      return comment.isBlank()
          ? ""
          : comment
              .replaceAll("(^|(?<=\n))\s*(?=\\w)", " ")
              .replaceAll("^|(?<=\n)", singleLineCommentPrefix);
    width = Math.max(width, MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
    var stripped =
        comment
            .replaceAll("\\s*\\*/$", "")
            .replaceAll("(?<=(\\r\\n|\\r|\\n))\\h*(\\*|//|#)\\h?(\\h*)", "$3")
            .strip();
    var preformatted = false;
    StringBuilder result = new StringBuilder(stripped.length() * 2);
    for (var part : stripped.split("```")) {
      result.append(
          preformatted
              ? part.lines()
                  .skip(1)
                  .map(it -> " * " + it)
                  .collect(Collectors.joining("\n", "\n * ```\n", "\n * ```\n"))
              : lineWrapComment(
                  Arrays.stream(part.split("(\n\\s*){2,}"))
                      .map(
                          s ->
                              Arrays.stream(s.split("(\\r\\n|\\r|\\n)\\h*(?=[@#$%^&*\\-+=:;<>/])")))
                      .map(stream -> stream.map(s -> s.replaceAll("\\s+", " ")))
                      .map(Stream::toList)
                      .toList(),
                  width,
                  " * "));
      preformatted = !preformatted;
    }
    if (result.indexOf("\n") == -1) {
      String singleLineRepresentation =
          String.format("/** %s */", result.substring(result.indexOf(" * ") + 3));
      if (singleLineRepresentation.length() <= width) return singleLineRepresentation;
    }
    return String.format("/**\n%s\n */", result);
  }

  /**
   * Wrap lines.
   *
   * @param paragraphs A list of lists of subparagraphs.
   * @param width The preferred maximum number of columns per line.
   * @param linePrefix A string to prepend to each line of comment.
   */
  private static String lineWrapComment(
      List<List<String>> paragraphs, int width, String linePrefix) {
    int widthAfterPrefix = Math.max(width - linePrefix.length(), MINIMUM_COMMENT_WIDTH_IN_COLUMNS);
    return paragraphs.stream()
        .map(
            paragraph ->
                wrapLines(paragraph, widthAfterPrefix)
                    .map(s -> (linePrefix + s.stripLeading()).stripTrailing())
                    .collect(Collectors.joining("\n")))
        .collect(Collectors.joining(String.format("\n%s\n", linePrefix.stripTrailing())));
  }

  /** Wrap a given paragraph. */
  private static Stream<String> wrapLines(List<String> subparagraphs, int width) {
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
   * Merge {@code comment} into the given list of strings without changing the length of the list,
   * preferably in a place that indicates that {@code comment} is associated with the {@code i}th
   * string.
   *
   * @param comment A comment associated with an element of {@code components}.
   * @param components A list of strings that will be rendered in sequence.
   * @param i The position of the component associated with {@code comment}.
   * @param width The ideal number of columns available for comments that appear on their own line.
   * @param keepCommentsOnSameLine Whether to make a best-effort attempt to keep the comment on the
   *     same line as the associated string.
   * @param singleLineCommentPrefix The prefix that marks the start of a single-line comment.
   * @param startColumn The ideal starting column of a comment
   * @return Whether the comment placement succeeded.
   */
  static boolean placeComment(
      List<String> comment,
      List<String> components,
      int i,
      int width,
      boolean keepCommentsOnSameLine,
      String singleLineCommentPrefix,
      int startColumn) {
    if (comment.stream().allMatch(String::isBlank)) return true;
    String wrapped = FormattingUtil.lineWrapComments(comment, width, singleLineCommentPrefix);
    if (keepCommentsOnSameLine && wrapped.lines().count() == 1 && !wrapped.startsWith("/**")) {
      int sum = 0;
      for (int j = 0; j < components.size(); j++) {
        String current = components.get(j);
        if (j >= i && current.contains("\n")) {
          if (components.get(j).lines().filter(it -> !it.isBlank()).count() > 1) break;
          components.set(
              j,
              components
                  .get(j)
                  .replaceFirst(
                      "\n",
                      " ".repeat(Math.max(2, startColumn - sum - components.get(j).indexOf("\n")))
                          + wrapped
                          + "\n"));
          return true;
        } else if (current.contains("\n")) {
          sum = current.length() - current.lastIndexOf("\n") - 1;
        } else {
          sum += current.length();
        }
      }
    }
    for (int j = i - 1; j >= 0; j--) {
      if (components.get(j).endsWith("\n")) {
        components.set(j, String.format("%s%s\n", components.get(j), wrapped));
        return true;
      }
    }
    return false;
  }
}
