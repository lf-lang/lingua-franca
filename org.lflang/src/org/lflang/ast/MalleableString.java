package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.IntStream;
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
        Comparator<String> whichRepresentationIsBetter,
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
            return insert(Function.identity(), Fork::new, possibilities, ms -> components.add(0, ms));
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
            if (
                Arrays.stream(possibilities)
                    .map(toMalleableString)
                    .allMatch(MalleableString::isEmpty)
            ) {
                return this;
            }
//            if (possibilities.length == 1) {
//                // The resulting MalleableString may be a sequence.
//                //  Stay flat: Let there be no sequences in sequences!
//                toMalleableString.apply(possibilities[0]).forEach(addToComponents);
//            } else {
                addToComponents.accept(multiplePossibilitiesRepresenter.apply(possibilities));
//            }
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
            return ((Function<Builder, MalleableString>) Builder::get).compose(appendSuffix).compose(prependPrefix);
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
            // TODO:
            //  find out whether you have any unhandled comments
            //  if so, optimize the components so that they contain as many newlines as possible?
            List<List<String>> unhandledComments = components.stream()
                .map(MalleableString::getUnhandledComments)
                .map(Stream::toList)
                .toList();
            List<String> stringComponents =  components.stream()
                .map(MalleableString::toString)
                .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);
            List<Integer> newLineIndices = IntStream.range(0, stringComponents.size()).sequential()
                .filter(i -> stringComponents.get(i).endsWith(System.lineSeparator()))
                .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);
            if (unhandledComments.stream().allMatch(List::isEmpty) || newLineIndices.isEmpty()) {
                return String.join("", stringComponents);
            }
            for (int i = 0; i < unhandledComments.size(); i++) {
                placeComments(
                    String.join(System.lineSeparator(), unhandledComments.get(i)),
                    stringComponents,
                    i,
                    width,
                    keepCommentsOnSameLine
                );
            }
            return String.join("", stringComponents);
        }

        private static void placeComments(
            String unhandledComments,
            List<String> components,
            int i,
            int width,
            boolean keepCommentsOnSameLine
        ) {
            String wrapped = FormattingUtils.lineWrapComment(unhandledComments, width);
            if (unhandledComments.isBlank()) return;
            if (keepCommentsOnSameLine && wrapped.lines().count() == 1) {
                for (int j = i; j < components.size(); j++) {
                    if (components.get(j).endsWith(System.lineSeparator())) {
                        components.set(j, components.get(j).replaceFirst(
                            System.lineSeparator() + "$",
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

        @SuppressWarnings("NullableProblems")
        @Override
        public Iterator<MalleableString> iterator() {
            return components.iterator();
        }

        @Override
        public void findBestRepresentation(
            Supplier<String> representationGetter,
            Comparator<String> whichRepresentationIsBetter,
            int width
        ) {
            this.width = width;
            keepCommentsOnSameLine = true;
            var representationTrue = representationGetter.get();
            keepCommentsOnSameLine = false;
            var representationFalse = representationGetter.get();
            keepCommentsOnSameLine = whichRepresentationIsBetter.compare(representationTrue, representationFalse) <= 0;
            for (MalleableString component : components) {
                component.findBestRepresentation(representationGetter, whichRepresentationIsBetter, width);
            }
        }

        @Override
        public boolean isEmpty() {
            return components.stream().allMatch(MalleableString::isEmpty);
        }

        @Override
        protected Stream<String> getUnhandledComments() {
            // FIXME: This is very crude.
            return Stream.concat(
                super.getUnhandledComments(),
                components.stream().anyMatch(it -> it.toString().endsWith(System.lineSeparator())) ? Stream.of()
                    : components.stream().flatMap(MalleableString::getUnhandledComments)
            );
        }
    }

    private static final class Indented extends MalleableString {

        private final int indentation;
        private final MalleableString nested;

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
            Comparator<String> whichRepresentationIsBetter,
            int width
        ) {
            nested.findBestRepresentation(
                representationGetter,
                whichRepresentationIsBetter,
                width - this.indentation
            );
        }

        @Override
        public boolean isEmpty() {
            return nested.isEmpty();
        }

        @Override
        protected Stream<String> getUnhandledComments() {
            return Stream.concat(super.getUnhandledComments(), nested.getUnhandledComments());
        }

        @SuppressWarnings("NullableProblems")
        @Override
        public Iterator<MalleableString> iterator() {
            return Collections.singleton((MalleableString) this).iterator();
        }

        @Override
        public String toString() {
            var nestedString = nested.toString();
            var ret = nestedString.indent(indentation);
            if (!nestedString.endsWith(System.lineSeparator()) && ret.endsWith(System.lineSeparator())) {
                ret = ret.substring(0, ret.length() - System.lineSeparator().length());
            }
            return ret;
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
            Comparator<String> whichRepresentationIsBetter,
            int width
        ) {
            bestPossibility = Collections.min(getPossibilities(), (a, b) -> {
                bestPossibility = a;
                String resultA = representationGetter.get();
                bestPossibility = b;
                String resultB = representationGetter.get();
                return whichRepresentationIsBetter.compare(resultA, resultB);
            });
            if (bestPossibility instanceof MalleableString ms) {
                ms.findBestRepresentation(
                    representationGetter,
                    whichRepresentationIsBetter,
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
