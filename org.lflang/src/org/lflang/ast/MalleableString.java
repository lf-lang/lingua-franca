package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.ToLongFunction;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.google.common.collect.ImmutableList;

public abstract class MalleableString {

    protected List<String> comments = new ArrayList<>();

    public MalleableString indent(int indentation) {
        if (indentation < 0) {
            throw new IllegalArgumentException("Indentation must be nonnegative.");
        }
        return new Indented(this, indentation);
    }

    public abstract void findBestRepresentation(
        Supplier<RenderResult> providedRender,
        ToLongFunction<RenderResult> badness,
        int width
    );

    public abstract boolean isEmpty();

    public MalleableString addComments(Collection<String> comments) {
        return addComments(comments.stream());
    }

    public MalleableString addComments(Stream<String> comments) {
        comments.filter(s -> !s.isBlank()).map(String::strip).forEach(this.comments::add);
        return this;
    }

    public abstract RenderResult render();

    public static MalleableString anyOf(MalleableString... possibilities) {
        return new Fork(possibilities);
    }

    public static MalleableString anyOf(String... possibilities) {
        return new Leaf(possibilities);
    }

    public static MalleableString anyOf(Object... possibilities) {
        return new Leaf(objectArrayToString(possibilities));
    }

    private static String[] objectArrayToString(Object[] objects) {
        String[] ret = new String[objects.length];
        for (int i = 0; i < objects.length; i++) {
            ret[i] = String.valueOf(objects[i]);
        }
        return ret;
    }

    public static final class Builder {

        private final List<MalleableString> components = new ArrayList<>();

        public Builder append(MalleableString... possibilities) {
            return insert(Function.identity(), Fork::new, possibilities, components::add);
        }

        public Builder prepend(MalleableString... possibilities) {
            return insert(
                Function.identity(),
                Fork::new,
                possibilities,
                ms -> components.add(0, ms)
            );
        }

        public Builder append(String... content) {
            return insert(Leaf::new, Leaf::new, content, components::add);
        }

        @SuppressWarnings("UnusedReturnValue")
        public Builder append(Object... content) {
            return append(objectArrayToString(content));
        }

        public MalleableString get() {
            return new Sequence(ImmutableList.copyOf(components));
        }

        private <T> Builder insert(
            Function<T, ? extends MalleableString> toMalleableString,
            Function<T[], ? extends MalleableString> multiplePossibilitiesRepresenter,
            T[] possibilities,
            Consumer<MalleableString> addToComponents
        ) {
            boolean allEmpty = Arrays.stream(possibilities)
                .map(toMalleableString)
                .allMatch(MalleableString::isEmpty);
            if (!allEmpty) {
                addToComponents.accept(multiplePossibilitiesRepresenter.apply(possibilities));
            }
            return this;
        }
    }

    public static final class Joiner implements Collector<
        MalleableString,
        Builder,
        MalleableString
    > {
        private final Function<Builder, Builder> appendSeparator;
        private final Function<Builder, Builder> prependPrefix;
        private final Function<Builder, Builder> appendSuffix;

        public Joiner(MalleableString separator) {
            this(separator, MalleableString.anyOf(""), MalleableString.anyOf(""));
        }

        public Joiner(MalleableString separator, MalleableString prefix, MalleableString suffix) {
            this.appendSeparator = builder ->
                builder.components.isEmpty() ? builder : builder.append(separator);
            this.prependPrefix = builder -> builder.prepend(prefix);
            this.appendSuffix = builder -> builder.append(suffix);
        }

        public Joiner(String separator) {
            this(MalleableString.anyOf(separator));
        }

        public Joiner(String separator, String prefix, String suffix) {
            this(
                MalleableString.anyOf(separator),
                MalleableString.anyOf(prefix),
                MalleableString.anyOf(suffix)
            );
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
                .compose(appendSuffix).compose(prependPrefix);
        }

        @Override
        public Set<Characteristics> characteristics() {
            return Set.of();
        }
    }

    public record RenderResult(
        Stream<String> unplacedComments,
        String rendering,
        int levelsOfCommentDisplacement
    ) {
        private RenderResult with(Stream<String> moreUnplacedComments) {
            return new RenderResult(
                Stream.concat(moreUnplacedComments, unplacedComments),
                rendering,
                levelsOfCommentDisplacement
            );
        }
    }

    private static final class Sequence extends MalleableString {

        private final ImmutableList<MalleableString> components;
        private Sequence(ImmutableList<MalleableString> components) {
            this.components = components;
        }

        private boolean keepCommentsOnSameLine = false;
        private int width = 0;

        @Override
        public RenderResult render() {
            List<RenderResult> componentRenderings = components.stream()
                .map(MalleableString::render).toList();
            List<List<String>> commentsFromChildren = componentRenderings.stream()
                .map(it -> it.unplacedComments).map(Stream::toList).toList();
            List<String> stringComponents =  componentRenderings.stream()
                .map(it -> it.rendering)
                .map(FormattingUtils::normalizeEol)
                .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);
            List<String> commentsThatCouldNotBeHandledHere = new ArrayList<>();
            int numCommentsDisplacedHere = 0;
            if (
                commentsFromChildren.stream().anyMatch(s -> !s.isEmpty())
            ) {
                for (int i = 0; i < commentsFromChildren.size(); i++) {
                    if (!FormattingUtils.placeComment(
                        String.join(System.lineSeparator(), commentsFromChildren.get(i)),
                        stringComponents,
                        i,
                        width,
                        keepCommentsOnSameLine
                    )) {
                        commentsThatCouldNotBeHandledHere.addAll(commentsFromChildren.get(i));
                        if (i != 0) numCommentsDisplacedHere++;
                    }
                }
            }
            return new RenderResult(
                Stream.concat(this.comments.stream(), commentsThatCouldNotBeHandledHere.stream()),
                String.join("", stringComponents),
                componentRenderings.stream()
                    .mapToInt(RenderResult::levelsOfCommentDisplacement).sum()
                    + numCommentsDisplacedHere
            );
        }

        @Override
        public void findBestRepresentation(
            Supplier<RenderResult> providedRender,
            ToLongFunction<RenderResult> badness,
            int width
        ) {
            this.width = width;
            keepCommentsOnSameLine = true;
            components.reverse()
                .forEach(it -> it.findBestRepresentation(providedRender, badness, width));
            if (
                components.stream()
                    .noneMatch(it -> it.render().unplacedComments.findAny().isPresent())
            ) return;
            long badnessTrue = badness.applyAsLong(providedRender.get());
            keepCommentsOnSameLine = false;
            components.reverse()
                .forEach(it -> it.findBestRepresentation(providedRender, badness, width));
            long badnessFalse = badness.applyAsLong(providedRender.get());
            keepCommentsOnSameLine = badnessTrue < badnessFalse;
        }

        @Override
        public boolean isEmpty() {
            return components.stream().allMatch(MalleableString::isEmpty);
        }
    }

    private static final class Indented extends MalleableString {

        /**
         * The indentation given by this indent alone (i.e., not including
         * ancestor indents).
         */
        private final int indentation;
        private final MalleableString nested;
        private int width;

        private Indented(MalleableString toIndent, int indentation) {
            this.indentation = indentation;
            this.nested = toIndent;
        }

        @Override
        public MalleableString indent(int indentation) {
            return new Indented(nested, this.indentation + indentation);
        }

        @Override
        public void findBestRepresentation(
            Supplier<RenderResult> providedRender,
            ToLongFunction<RenderResult> badness,
            int width
        ) {
            this.width = width;
            nested.findBestRepresentation(
                providedRender,
                badness,
                width - this.indentation
            );
        }

        @Override
        public boolean isEmpty() {
            return nested.isEmpty();
        }

        @Override
        public RenderResult render() {
            var result = nested.render();
            String renderedComments = FormattingUtils.lineWrapComment(
                result.unplacedComments.collect(Collectors.joining(System.lineSeparator())),
                width - indentation
            );
            return new RenderResult(
                this.comments.stream(),
                (
                    renderedComments.isBlank() ? result.rendering
                        : renderedComments + System.lineSeparator() + result.rendering
                ).replaceAll(
                    "(?<=" + System.lineSeparator() + "|^)(?=\\h*\\S)",
                    " ".repeat(indentation)
                ),
                result.levelsOfCommentDisplacement()
            );
        }
    }

    private abstract static class MalleableStringImpl <T> extends MalleableString {
        protected abstract List<T> getPossibilities();

        private T bestPossibility;

        @Override
        public String toString() {
            return getChosenPossibility().toString();
        }

        @Override
        public void findBestRepresentation(
            Supplier<RenderResult> providedRender,
            ToLongFunction<RenderResult> badness,
            int width
        ) {
            bestPossibility = Collections.min(getPossibilities(), (a, b) -> {
                bestPossibility = a;
                long badnessA = badness.applyAsLong(providedRender.get());
                bestPossibility = b;
                long badnessB = badness.applyAsLong(providedRender.get());
                return Math.toIntExact(badnessA - badnessB);
            });
            if (bestPossibility instanceof MalleableString ms) {
                ms.findBestRepresentation(
                    providedRender,
                    badness,
                    width
                );
            }
        }

        protected T getChosenPossibility() {
            if (getPossibilities().isEmpty()) {
                throw new IllegalStateException(
                    "A MalleableString must be directly or transitively backed "
                        + "by at least one String."
                );
            }
            return bestPossibility == null ? getPossibilities().get(0) : bestPossibility;
        }
    }

    private static final class Fork extends MalleableStringImpl<MalleableString> {
        private final ImmutableList<MalleableString> possibilities;
        private Fork(MalleableString[] possibilities) {
            this.possibilities = ImmutableList.copyOf(possibilities);
        }

        @Override
        protected List<MalleableString> getPossibilities() { return this.possibilities; }

        @Override
        public boolean isEmpty() {
            return possibilities.stream().allMatch(MalleableString::isEmpty);
        }

        @Override
        public RenderResult render() {
            return getChosenPossibility().render().with(comments.stream());
        }
    }

    private static final class Leaf extends MalleableStringImpl<String> {
        private final ImmutableList<String> possibilities;
        private Leaf(String[] possibilities) {
            this.possibilities = ImmutableList.copyOf(possibilities);
        }
        private Leaf(String possibility) {
            this.possibilities = ImmutableList.of(possibility);
        }

        @Override
        protected List<String> getPossibilities() { return this.possibilities; }

        @Override
        public boolean isEmpty() {
            return possibilities.stream().allMatch(String::isEmpty);
        }

        @Override
        public RenderResult render() {
            return new RenderResult(comments.stream(), getChosenPossibility(), 0);
        }
    }
}
