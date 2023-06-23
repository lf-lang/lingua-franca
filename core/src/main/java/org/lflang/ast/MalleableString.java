package org.lflang.ast;

import com.google.common.collect.ImmutableList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.function.ToLongFunction;
import java.util.stream.Collector;
import java.util.stream.Stream;
import org.eclipse.emf.ecore.EObject;
import org.lflang.generator.CodeMap;
import org.lflang.lf.Code;
import org.lflang.util.StringUtil;

/**
 * A {@code MalleableString} is an object with multiple valid textual representations. These textual
 * representations are code that may have associated comments.
 */
public abstract class MalleableString {

  protected List<String> comments = new ArrayList<>();
  protected EObject sourceEObject = null;

  /** Return this, indented by one more level. */
  public MalleableString indent() {
    return new Indented(this);
  }

  /**
   * Change the state of this such that the badness of the supplied render result is minimized.
   *
   * @param providedRender A supplier of render results that should be optimized.
   * @param badness A badness computer for render results.
   * @param width The number of columns permitted for this, excluding indentation applied to the
   *     whole of this.
   * @param indentation The number of spaces used per level of indentation.
   * @param singleLineCommentPrefix The prefix that marks the start of a single-line comment.
   * @return Whether the best representation changed.
   */
  public abstract boolean findBestRepresentation(
      Supplier<RenderResult> providedRender,
      ToLongFunction<RenderResult> badness,
      int width,
      int indentation,
      String singleLineCommentPrefix);

  /** Return whether any representation of this contains text. */
  public abstract boolean isEmpty();

  /** Associate comments with this. */
  public MalleableString addComments(Stream<String> comments) {
    comments.filter(s -> !s.isBlank()).map(String::strip).forEach(this.comments::add);
    return this;
  }

  /** Specify the EObject from which this originated, if applicable. */
  public MalleableString setSourceEObject(EObject sourceEObject) {
    this.sourceEObject = sourceEObject;
    return this;
  }

  /**
   * Render this using {@code indentation} spaces per indentation level and {@code
   * singleLineCommentMarker} to mark the beginnings of single-line comments.
   */
  public abstract RenderResult render(
      int indentation,
      String singleLineCommentMarker,
      boolean codeMapTag,
      EObject enclosingEObject);

  /** Return an object that can be represented as any one of the given alternatives. */
  public static MalleableString anyOf(MalleableString... possibilities) {
    return new Fork(possibilities);
  }

  /** Return an object that can be represented as any one of the given alternatives. */
  public static MalleableString anyOf(String... possibilities) {
    return new Leaf(possibilities);
  }

  /** Return an object that can be represented as any one of the given alternatives. */
  public static MalleableString anyOf(Object... possibilities) {
    return new Leaf(objectArrayToString(possibilities));
  }

  /**
   * Apply the given constraint to leaf strings of this.
   *
   * <p>This is done on a best-effort basis in the sense that if no options satisfy the constraint,
   * the constraint is not applied.
   */
  public abstract MalleableString constrain(Predicate<String> constraint);

  private static String[] objectArrayToString(Object[] objects) {
    String[] ret = new String[objects.length];
    for (int i = 0; i < objects.length; i++) {
      ret[i] = String.valueOf(objects[i]);
    }
    return ret;
  }

  @Override
  public String toString() {
    List<String> temp = comments;
    comments = List.of();
    String ret = render(0, "", false, null).rendering;
    comments = temp;
    return ret;
  }

  /** Build a {@code MalleableString} in a manner analogous to the way we build {@code String}s. */
  public static final class Builder {

    private final List<MalleableString> components = new ArrayList<>();

    /** Append something that can be represented as any of the given possibilities. */
    public Builder append(MalleableString... possibilities) {
      return insert(Function.identity(), Fork::new, possibilities, components::add);
    }

    /** Prepend something that can be represented as any of the given possibilities. */
    public Builder prepend(MalleableString... possibilities) {
      return insert(Function.identity(), Fork::new, possibilities, ms -> components.add(0, ms));
    }

    /** Append something that can be represented as any of the given possibilities. */
    public Builder append(String... content) {
      return insert(Leaf::new, Leaf::new, content, components::add);
    }

    /** Append something that can be represented as any of the given possibilities. */
    @SuppressWarnings("UnusedReturnValue")
    public Builder append(Object... content) {
      return append(objectArrayToString(content));
    }

    /** Append something that can be represented as any of the given possibilities. */
    public MalleableString get() {
      return new Sequence(ImmutableList.copyOf(components));
    }

    /** Append something that can be represented as any of the given possibilities. */
    private <T> Builder insert(
        Function<T, ? extends MalleableString> toMalleableString,
        Function<T[], ? extends MalleableString> multiplePossibilitiesRepresenter,
        T[] possibilities,
        Consumer<MalleableString> addToComponents) {
      boolean allEmpty =
          Arrays.stream(possibilities).map(toMalleableString).allMatch(MalleableString::isEmpty);
      if (!allEmpty) {
        addToComponents.accept(multiplePossibilitiesRepresenter.apply(possibilities));
      }
      return this;
    }
  }

  /** Join {@code MalleableString}s together using the given separator. */
  public static final class Joiner implements Collector<MalleableString, Builder, MalleableString> {
    private final Function<Builder, Builder> appendSeparator;
    private final Function<Builder, Builder> prependPrefix;
    private final Function<Builder, Builder> appendSuffix;

    /** Join strings using {@code separator}. */
    public Joiner(String separator) {
      this(MalleableString.anyOf(separator));
    }

    /** Join strings using {@code separator}. */
    public Joiner(MalleableString separator) {
      this(separator, MalleableString.anyOf(""), MalleableString.anyOf(""));
    }

    /**
     * Join strings using {@code separator} and delimit the result with {@code prefix} and {@code
     * suffix}.
     */
    public Joiner(MalleableString separator, MalleableString prefix, MalleableString suffix) {
      this.appendSeparator =
          builder -> builder.components.isEmpty() ? builder : builder.append(separator);
      this.prependPrefix = builder -> builder.prepend(prefix);
      this.appendSuffix = builder -> builder.append(suffix);
    }

    /**
     * Join strings using {@code separator} and delimit the result with {@code prefix} and {@code
     * suffix}.
     */
    public Joiner(String separator, String prefix, String suffix) {
      this(
          MalleableString.anyOf(separator),
          MalleableString.anyOf(prefix),
          MalleableString.anyOf(suffix));
    }

    @Override
    public Supplier<Builder> supplier() {
      return Builder::new;
    }

    @Override
    public BiConsumer<Builder, MalleableString> accumulator() {
      return (b, ms) -> appendSeparator.apply(b).append(ms);
    }

    @Override
    public BinaryOperator<Builder> combiner() {
      return (builder0, builder1) -> {
        builder1.components.forEach(ms -> accumulator().accept(builder0, ms));
        return builder0;
      };
    }

    @Override
    public Function<Builder, MalleableString> finisher() {
      return ((Function<Builder, MalleableString>) Builder::get)
          .compose(appendSuffix)
          .compose(prependPrefix);
    }

    @Override
    public Set<Characteristics> characteristics() {
      return Set.of();
    }
  }

  /** The result of rendering a {@code MalleableString}. */
  public record RenderResult(
      Stream<String> unplacedComments, String rendering, int levelsOfCommentDisplacement) {
    private RenderResult with(Stream<String> moreUnplacedComments) {
      return new RenderResult(
          Stream.concat(moreUnplacedComments, unplacedComments),
          rendering,
          levelsOfCommentDisplacement);
    }
  }

  /** Represent a list of items that should be rendered in sequence. */
  private static final class Sequence extends MalleableString {

    private final ImmutableList<MalleableString> components;

    private Sequence(ImmutableList<MalleableString> components) {
      this.components = components;
    }

    private boolean keepCommentsOnSameLine = false;
    private int width = 0;

    @Override
    public RenderResult render(
        int indentation,
        String singleLineCommentPrefix,
        boolean codeMapTag,
        EObject enclosingEObject) {
      List<RenderResult> componentRenderings =
          components.stream()
              .map(
                  malleableString ->
                      // The code map tags *should* not affect the correctness of the formatting
                      // since most
                      // formatting has already happened in findBestRepresentation.
                      malleableString.render(
                          indentation,
                          singleLineCommentPrefix,
                          codeMapTag,
                          sourceEObject != null ? sourceEObject : enclosingEObject))
              .toList();
      List<List<String>> commentsFromChildren =
          componentRenderings.stream().map(it -> it.unplacedComments).map(Stream::toList).toList();
      List<String> stringComponents =
          componentRenderings.stream()
              .map(it -> it.rendering)
              .map(StringUtil::normalizeEol)
              .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);
      List<String> commentsThatCouldNotBeHandledHere = new ArrayList<>();
      int startColumn = inlineCommentStartColumn(stringComponents, commentsFromChildren);
      int numCommentsDisplacedHere = 0;
      if (commentsFromChildren.stream().anyMatch(s -> !s.isEmpty())) {
        for (int i = 0; i < commentsFromChildren.size(); i++) {
          if (!FormattingUtil.placeComment(
              commentsFromChildren.get(i),
              stringComponents,
              i,
              width,
              keepCommentsOnSameLine,
              singleLineCommentPrefix,
              startColumn)) {
            commentsThatCouldNotBeHandledHere.addAll(commentsFromChildren.get(i));
            if (i != 0) numCommentsDisplacedHere++;
          }
        }
      }
      return new RenderResult(
          Stream.concat(this.comments.stream(), commentsThatCouldNotBeHandledHere.stream()),
          String.join("", stringComponents),
          componentRenderings.stream().mapToInt(RenderResult::levelsOfCommentDisplacement).sum()
              + numCommentsDisplacedHere);
    }

    /**
     * Return the ideal starting column of an aligned comment, or 0 if comments should not be
     * aligned.
     *
     * @param stringComponents The non-comment components of the sequence.
     * @param comments The comments that will need to be incorporated into the sequence's
     *     representation.
     */
    private int inlineCommentStartColumn(
        List<String> stringComponents, List<List<String>> comments) {
      final int[] lineLengths =
          getLinesOfInterest(stringComponents, comments).stream()
              .mapToInt(String::length)
              .toArray();
      int minNonCommentWidth = Integer.MAX_VALUE;
      int maxNonIgnoredCommentWidth = 0;
      int numIgnored = lineLengths.length;
      for (int i : lineLengths) {
        if (i > 0) minNonCommentWidth = Math.min(minNonCommentWidth, i);
      }
      for (int i : lineLengths) {
        if (i < minNonCommentWidth + FormattingUtil.MAX_WHITESPACE_USED_FOR_ALIGNMENT) {
          maxNonIgnoredCommentWidth = Math.max(maxNonIgnoredCommentWidth, i);
          numIgnored--;
        }
      }
      if (numIgnored > lineLengths.length / 2) return 0;
      final int padding = 2;
      final int maxCommentWidth =
          comments.stream()
              .mapToInt(list -> list.stream().mapToInt(String::length).sum())
              .max()
              .orElse(0);
      return maxNonIgnoredCommentWidth + padding + maxCommentWidth <= width
          ? maxNonIgnoredCommentWidth + padding
          : 0;
    }

    private List<String> getLinesOfInterest(
        List<String> stringComponents, List<List<String>> comments) {
      List<Integer> lineNumbersOfInterest = new ArrayList<>();
      int line = 0;
      for (int i = 0; i < stringComponents.size(); i++) {
        if (!comments.get(i).isEmpty()) lineNumbersOfInterest.add(line);
        int idx = 0;
        while ((idx = stringComponents.get(i).indexOf("\n", idx) + 1) > 0) line++;
      }
      final List<String> lines = String.join("", stringComponents).lines().toList();
      return lineNumbersOfInterest.stream().map(lines::get).toList();
    }

    @Override
    public boolean findBestRepresentation(
        Supplier<RenderResult> providedRender,
        ToLongFunction<RenderResult> badness,
        int width,
        int indentation,
        String singleLineCommentPrefix) {
      this.width = width;
      keepCommentsOnSameLine = true;
      // Multiple calls to optimizeChildren may be required because as parts of the textual
      // representation are updated, the optimal representation of other parts may change.
      // For example, if the text is wider than 100 characters, the line may only need to be
      // broken in one place, but it will be broken in multiple places a second optimization pass
      // is not made. This is a heuristic in the sense that two passes are not guaranteed to result
      // in a fixed point, but since a subsequent call to the formatter will get the same AST and
      // therefore have the same starting point, the formatter as a whole should still be
      // idempotent.
      var everChanged = false;
      var changed =
          optimizeChildren(providedRender, badness, width, indentation, singleLineCommentPrefix);
      everChanged = changed;
      if (changed)
        changed =
            optimizeChildren(providedRender, badness, width, indentation, singleLineCommentPrefix);
      if (components.stream()
          .noneMatch(
              it ->
                  it.render(indentation, singleLineCommentPrefix, false, null)
                      .unplacedComments
                      .findAny()
                      .isPresent())) return changed;
      long badnessTrue = badness.applyAsLong(providedRender.get());
      keepCommentsOnSameLine = false;
      changed =
          optimizeChildren(providedRender, badness, width, indentation, singleLineCommentPrefix);
      everChanged |= changed;
      long badnessFalse = badness.applyAsLong(providedRender.get());
      keepCommentsOnSameLine = badnessTrue < badnessFalse;
      if (changed)
        changed =
            optimizeChildren(providedRender, badness, width, indentation, singleLineCommentPrefix);
      if (changed)
        optimizeChildren(providedRender, badness, width, indentation, singleLineCommentPrefix);
      return everChanged;
    }

    private boolean optimizeChildren(
        Supplier<RenderResult> providedRender,
        ToLongFunction<RenderResult> badness,
        int width,
        int indentation,
        String singleLineCommentPrefix) {
      return components.reverse().stream()
          .anyMatch(
              it ->
                  it.findBestRepresentation(
                      providedRender, badness, width, indentation, singleLineCommentPrefix));
    }

    @Override
    public MalleableString constrain(Predicate<String> constraint) {
      for (var component : components) {
        component.constrain(constraint);
      }
      return this;
    }

    @Override
    public boolean isEmpty() {
      return components.stream().allMatch(MalleableString::isEmpty);
    }
  }

  /** Represent an indented version of another {@code MalleableString}. */
  private static final class Indented extends MalleableString {

    private final MalleableString nested;
    private int width;

    private Indented(MalleableString toIndent) {
      this.nested = toIndent;
    }

    @Override
    public MalleableString indent() {
      return new Indented(this);
    }

    @Override
    public boolean findBestRepresentation(
        Supplier<RenderResult> providedRender,
        ToLongFunction<RenderResult> badness,
        int width,
        int indentation,
        String singleLineCommentPrefix) {
      this.width = width;
      return nested.findBestRepresentation(
          providedRender, badness, width - indentation, indentation, singleLineCommentPrefix);
    }

    @Override
    public boolean isEmpty() {
      return nested.isEmpty();
    }

    @Override
    public RenderResult render(
        int indentation,
        String singleLineCommentPrefix,
        boolean codeMapTag,
        EObject enclosingEObject) {
      var result =
          nested.render(
              indentation,
              singleLineCommentPrefix,
              codeMapTag,
              sourceEObject != null ? sourceEObject : enclosingEObject);
      String renderedComments =
          FormattingUtil.lineWrapComments(
              result.unplacedComments.toList(), width - indentation, singleLineCommentPrefix);
      return new RenderResult(
          this.comments.stream(),
          (renderedComments.isBlank()
                  ? result.rendering
                  : renderedComments + "\n" + result.rendering)
              .replaceAll("(?<=\n|^)(?=\\h*\\S)", " ".repeat(indentation)),
          result.levelsOfCommentDisplacement());
    }

    @Override
    public MalleableString constrain(Predicate<String> constraint) {
      nested.constrain(constraint);
      return this;
    }
  }

  /** Represent a {@code MalleableString} that admits multiple possible representations. */
  private abstract static class MalleableStringWithAlternatives<T> extends MalleableString {
    protected abstract List<T> getPossibilities();

    private T bestPossibility;

    @Override
    public String toString() {
      return getChosenPossibility().toString();
    }

    @Override
    public boolean findBestRepresentation(
        Supplier<RenderResult> providedRender,
        ToLongFunction<RenderResult> badness,
        int width,
        int indentation,
        String singleLineCommentPrefix) {
      var initialChosenPossibility = getChosenPossibility();
      bestPossibility =
          Collections.min(
              getPossibilities(),
              (a, b) -> {
                bestPossibility = a;
                long badnessA = badness.applyAsLong(providedRender.get());
                bestPossibility = b;
                long badnessB = badness.applyAsLong(providedRender.get());
                return Math.toIntExact(badnessA - badnessB);
              });
      if (bestPossibility instanceof MalleableString ms) {
        if (ms.findBestRepresentation(
            providedRender, badness, width, indentation, singleLineCommentPrefix)) return true;
      }
      return getChosenPossibility() != initialChosenPossibility;
    }

    /** Return the best representation of this. */
    protected T getChosenPossibility() {
      if (getPossibilities().isEmpty()) {
        throw new IllegalStateException(
            "A MalleableString must be directly or transitively backed "
                + "by at least one String.");
      }
      return bestPossibility == null ? getPossibilities().get(0) : bestPossibility;
    }
  }

  /** A {@code Fork} can be represented by multiple possible {@code MalleableString}s. */
  private static final class Fork extends MalleableStringWithAlternatives<MalleableString> {
    private final ImmutableList<MalleableString> possibilities;

    private Fork(MalleableString[] possibilities) {
      this.possibilities = ImmutableList.copyOf(possibilities);
    }

    @Override
    protected List<MalleableString> getPossibilities() {
      return this.possibilities;
    }

    @Override
    public boolean isEmpty() {
      return possibilities.stream().allMatch(MalleableString::isEmpty);
    }

    @Override
    public RenderResult render(
        int indentation,
        String singleLineCommentPrefix,
        boolean codeMapTag,
        EObject enclosingEObject) {
      return getChosenPossibility()
          .render(
              indentation,
              singleLineCommentPrefix,
              codeMapTag,
              sourceEObject != null ? sourceEObject : enclosingEObject)
          .with(comments.stream());
    }

    @Override
    public MalleableString constrain(Predicate<String> constraint) {
      for (var possibility : possibilities) {
        possibility.constrain(constraint);
      }
      return this;
    }
  }

  /** A {@code Leaf} can be represented by multiple possible {@code String}s. */
  private static final class Leaf extends MalleableStringWithAlternatives<String> {
    private List<String> possibilities;

    private Leaf(String[] possibilities) {
      this.possibilities = List.of(possibilities);
    }

    private Leaf(String possibility) {
      this.possibilities = List.of(possibility);
    }

    @Override
    protected List<String> getPossibilities() {
      return possibilities;
    }

    @Override
    public boolean isEmpty() {
      return possibilities.stream().allMatch(String::isEmpty);
    }

    @Override
    public RenderResult render(
        int indentation,
        String singleLineCommentPrefix,
        boolean codeMapTag,
        EObject enclosingEObject) {
      return new RenderResult(
          comments.stream(),
          enclosingEObject instanceof Code && codeMapTag
              ? CodeMap.Correspondence.tag(enclosingEObject, getChosenPossibility(), true)
              : getChosenPossibility(),
          0);
    }

    @Override
    public MalleableString constrain(Predicate<String> constraint) {
      var newPossibilities = possibilities.stream().filter(constraint).toList();
      if (!newPossibilities.isEmpty()) possibilities = newPossibilities;
      return this;
    }
  }
}
