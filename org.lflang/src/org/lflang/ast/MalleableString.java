package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
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

public abstract class MalleableString implements Iterable<MalleableString> {

    protected List<String> comments = new ArrayList<>();

    public MalleableString indent(int indentation) {
        if (indentation < 0) {
            throw new IllegalArgumentException("Indentation must be nonnegative.");
        }
        return new Indented(this, indentation);
    }

    public abstract void findBestRepresentation(
        Supplier<String> representationGetter,
        ToLongFunction<String> badness,
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

    protected Stream<String> getUnhandledComments() {
        return comments.stream();
    }

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

    private static final class Sequence extends MalleableString {
        private final ImmutableList<MalleableString> components;
        private Sequence(ImmutableList<MalleableString> components) {
            this.components = components;
        }

        private boolean keepCommentsOnSameLine = false;
        private int width = 0;

        @Override
        public String toString() {
            List<List<String>> unhandledComments = components.stream()
                .map(MalleableString::getUnhandledComments)
                .map(stream -> stream.map(FormattingUtils::normalizeEol))
                .map(Stream::toList)
                .toList();
            List<String> stringComponents =  components.stream()
                .map(MalleableString::toString)
                .map(FormattingUtils::normalizeEol)
                .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);
            if (
                unhandledComments.stream().anyMatch(s -> !s.isEmpty())
                    && stringComponents.stream().anyMatch(s -> s.contains("\r") || s.contains("\n"))
            ) {
                for (int i = 0; i < unhandledComments.size(); i++) {
                    FormattingUtils.placeComment(
                        String.join(System.lineSeparator(), unhandledComments.get(i)),
                        stringComponents,
                        i,
                        width,
                        keepCommentsOnSameLine
                    );
                }
            }
            return String.join("", stringComponents);
        }

        @SuppressWarnings("NullableProblems")
        @Override
        public Iterator<MalleableString> iterator() {
            return components.iterator();
        }

        @Override
        public void findBestRepresentation(
            Supplier<String> representationGetter,
            ToLongFunction<String> badness,
            int width
        ) {
            this.width = width;
            for (MalleableString component : components) {
                component.findBestRepresentation(
                    representationGetter,
                    s -> getUnhandledComments().count()
                        * FormattingUtils.BADNESS_PER_MISPLACED_COMMENT
                        + badness.applyAsLong(s),
                    width
                );
            }
            keepCommentsOnSameLine = true;
            long badnessTrue = badness.applyAsLong(representationGetter.get());
            keepCommentsOnSameLine = false;
            long badnessFalse = badness.applyAsLong(representationGetter.get());
            keepCommentsOnSameLine = badnessTrue < badnessFalse;
        }

        @Override
        public boolean isEmpty() {
            return components.stream().allMatch(MalleableString::isEmpty);
        }

        @Override
        protected Stream<String> getUnhandledComments() {
            Stream<String> unhandledComments = super.getUnhandledComments();
            for (MalleableString ms : components) {
                unhandledComments = Stream.concat(unhandledComments, ms.getUnhandledComments());
                String s = ms.toString();
                if (s.contains("\r") || s.contains("\n")) break;
            }
            return unhandledComments;
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
            Supplier<String> representationGetter,
            ToLongFunction<String> badness,
            int width
        ) {
            this.width = width;
            nested.findBestRepresentation(
                representationGetter,
                badness,
                width - this.indentation
            );
        }

        @Override
        public boolean isEmpty() {
            return nested.isEmpty();
        }

        @Override
        protected Stream<String> getUnhandledComments() {
            return Stream.of();
        }

        @SuppressWarnings("NullableProblems")
        @Override
        public Iterator<MalleableString> iterator() {
            return Collections.singleton((MalleableString) this).iterator();
        }

        @Override
        public String toString() {
            return (
                nested.getUnhandledComments().map(
                    s -> FormattingUtils.lineWrapComment(s, width - indentation)
                ).collect(Collectors.joining(System.lineSeparator()))
                + nested
            ).replaceAll(
                "(?<=" + System.lineSeparator() + "|^)(?=\\s*\\S)",
                " ".repeat(indentation)
            );
        }
    }

    private abstract static class MalleableStringImpl extends MalleableString {
        protected abstract List<?> getPossibilities();

        private Object bestPossibility;

        @Override
        public String toString() {
            return getChosenPossibility().toString();
        }

        @Override
        public Iterator<MalleableString> iterator() {
            return Collections.singleton((MalleableString) this).iterator();
        }

        @Override
        public void findBestRepresentation(
            Supplier<String> representationGetter,
            ToLongFunction<String> badness,
            int width
        ) {
            bestPossibility = Collections.min(getPossibilities(), (a, b) -> {
                bestPossibility = a;
                long badnessA = badness.applyAsLong(representationGetter.get());
                bestPossibility = b;
                long badnessB = badness.applyAsLong(representationGetter.get());
                return Math.toIntExact(badnessA - badnessB);
            });
            if (bestPossibility instanceof MalleableString ms) {
                ms.findBestRepresentation(
                    representationGetter,
                    badness,
                    width
                );
            }
        }

        protected Object getChosenPossibility() {
            if (getPossibilities().isEmpty()) {
                throw new IllegalStateException(
                    "A MalleableString must be directly or transitively backed "
                        + "by at least one String."
                );
            }
            return bestPossibility == null ? getPossibilities().get(0) : bestPossibility;
        }
    }

    private static final class Fork extends MalleableStringImpl {
        private final ImmutableList<MalleableString> possibilities;
        private Fork(MalleableString[] possibilities) {
            this.possibilities = ImmutableList.copyOf(possibilities);
        }

        @Override
        protected List<?> getPossibilities() { return this.possibilities; }

        @Override
        public boolean isEmpty() {
            return possibilities.stream().allMatch(MalleableString::isEmpty);
        }

        @Override
        protected Stream<String> getUnhandledComments() {
            Stream<String> nestedUnhandled = getChosenPossibility() instanceof MalleableString ms ?
                ms.getUnhandledComments() : Stream.of();
            return Stream.concat(super.getUnhandledComments(), nestedUnhandled);
        }
    }

    private static final class Leaf extends MalleableStringImpl {
        private final ImmutableList<String> possibilities;
        private Leaf(String[] possibilities) {
            this.possibilities = ImmutableList.copyOf(possibilities);
        }
        private Leaf(String possibility) {
            this.possibilities = ImmutableList.of(possibility);
        }

        @Override
        protected List<?> getPossibilities() { return this.possibilities; }

        @Override
        public boolean isEmpty() {
            return possibilities.stream().allMatch(String::isEmpty);
        }
    }
}
