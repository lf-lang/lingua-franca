package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import org.jetbrains.annotations.NotNull;

import com.google.common.collect.ImmutableList;

public interface MalleableString extends Iterable<MalleableString> {

    default MalleableString indent(int indentation) {
        if (indentation < 0) {
            throw new IllegalArgumentException("Indentation must be nonnegative.");
        }
        return new Indented(this, indentation);
    }

    boolean isEmpty();

    static MalleableString anyOf(MalleableString... possibilities) {
        return new Fork(possibilities);
    }

    static MalleableString anyOf(String... possibilities) {
        return new Leaf(possibilities);
    }

    static MalleableString anyOf(Object... possibilities) {
        return new Leaf((String[]) Arrays.stream(possibilities).map(String::valueOf).toArray());
    }

    final class Builder {

        List<MalleableString> components = new ArrayList<>();

        Builder append(MalleableString... possibilities) {
            return append(Function.identity(), Fork::new, possibilities);
        }

        Builder append(String... content) {
            return append(Leaf::new, Leaf::new, content);
        }

        Builder append(Object... content) {
            return append((String[]) Arrays.stream(content).map(Objects::toString).toArray());
        }

        MalleableString get() {
            return new Sequence(ImmutableList.copyOf(components));
        }

        private <T> Builder append(
            Function<T, ? extends MalleableString> toMalleableString,
            Function<T[], ? extends MalleableString> multiplePossibilitiesRepresenter,
            T[] possibilities
        ) {
            if (
                Arrays.stream(possibilities)
                    .map(toMalleableString)
                    .allMatch(MalleableString::isEmpty)
            ) {
                return this;
            }
            if (possibilities.length == 1) {
                // The resulting MalleableString may be a sequence.
                //  Stay flat: Let there be no sequences in sequences!
                toMalleableString.apply(possibilities[0]).forEach(components::add);
            } else {
                components.add(multiplePossibilitiesRepresenter.apply(possibilities));
            }
            return this;
        }
    }

    final class Joiner implements Collector<
        MalleableString,
        Builder,
        MalleableString
    > {
        private final Function<Builder, Builder> appendSeparator;
        private final Function<Builder, Builder> appendPrefix;
        private final Function<Builder, Builder> appendSuffix;

        public Joiner() { this(MalleableString.anyOf(", ")); }

        public Joiner(MalleableString separator) {
            this(separator, MalleableString.anyOf(""), MalleableString.anyOf(""));
        }

        public Joiner(MalleableString separator, MalleableString prefix, MalleableString suffix) {
            this.appendSeparator = builder ->
                builder.components.isEmpty() ? builder : builder.append(separator);
            this.appendPrefix = builder -> builder.append(prefix);
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
            return () -> appendPrefix.apply(new Builder());
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
            return ((Function<Builder, MalleableString>) Builder::get).compose(appendSuffix);
        }

        @Override
        public Set<Characteristics> characteristics() {
            return Set.of();
        }
    }


    @SuppressWarnings("ClassCanBeRecord")
    final class Sequence implements MalleableString {
        private final ImmutableList<MalleableString> components;
        private Sequence(ImmutableList<MalleableString> components) {
            this.components = components;
        }

        @Override
        public String toString() {
            return components.stream()
                .map(MalleableString::toString)
                .collect(Collectors.joining(""));
        }

        @NotNull
        @Override
        public Iterator<MalleableString> iterator() {
            return components.iterator();
        }

        @Override
        public boolean isEmpty() {
            return components.stream().allMatch(MalleableString::isEmpty);
        }
    }

    final class Indented implements MalleableString {

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
        public boolean isEmpty() {
            return nested.isEmpty();
        }

        @NotNull
        @Override
        public Iterator<MalleableString> iterator() {
            return Collections.singleton((MalleableString) this).iterator();
        }

        @Override
        public String toString() {
            return nested.toString().indent(indentation);
        }
    }

    abstract class MalleableStringImpl implements MalleableString {
        protected abstract List<?> getPossibilities();

        @Override
        public String toString() {
            if (getPossibilities().isEmpty()) {
                throw new IllegalStateException(
                    "A MalleableString must be directly or transitively backed "
                        + "by at least one String."
                );
            }
            return getPossibilities().get(0).toString();
        }

        @NotNull
        @Override
        public Iterator<MalleableString> iterator() {
            return Collections.singleton((MalleableString) this).iterator();
        }
    }

    final class Fork extends MalleableStringImpl {
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
    }

    final class Leaf extends MalleableStringImpl {
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
