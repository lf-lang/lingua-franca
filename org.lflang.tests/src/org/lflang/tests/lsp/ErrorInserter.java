package org.lflang.tests.lsp;

import java.io.Closeable;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Stream;

import com.google.common.collect.ImmutableList;

/**
 * Insert problems into integration tests.
 * @author Peter Donovan
 */
@SuppressWarnings("ClassCanBeRecord")
class ErrorInserter {

    /** A basic error inserter builder on which more specific error inserters can be built. */
    private static final Builder BASE_ERROR_INSERTER = new Builder()
        .insertCondition((s0, s1) -> Stream.of(s0, s1).allMatch(it -> Stream.of(";", "}", "{").anyMatch(it::endsWith)))
        .insertCondition((s0, s1) -> !s1.trim().startsWith("else"))
        .insertable("    0 = 1;").insertable("some_undeclared_var1524263 = 9;").insertable("        ++;");
    public static final Builder C = BASE_ERROR_INSERTER
        .replacer("lf_set(", "UNDEFINED_NAME2828376(")
        .replacer("lf_schedule(", "undefined_name15291838(");
    public static final Builder CPP = BASE_ERROR_INSERTER
        .replacer(".get", ".undefined_name15291838")
        .replacer("std::", "undefined_name3286634::");
    public static final Builder PYTHON_SYNTAX_ONLY = new Builder()
        .insertable("        +++++;").insertable("        ..");
    public static final Builder PYTHON = PYTHON_SYNTAX_ONLY
        .replacer("print(", "undefined_name15291838(");
    public static final Builder RUST = BASE_ERROR_INSERTER
        .replacer("println!", "undefined_name15291838!")
        .replacer("ctx.", "undefined_name3286634.");
    public static final Builder TYPESCRIPT = BASE_ERROR_INSERTER
        .replacer("requestErrorStop(", "not_an_attribute_of_util9764(")
        .replacer("const ", "var ");

    /** An {@code AlteredTest} represents an altered version of what was a valid LF file. */
    static class AlteredTest implements Closeable {

        /** A {@code OnceTrue} is randomly true once, and then never again. */
        private static class OnceTrue {
            boolean beenTrue;
            Random random;
            private OnceTrue(Random random) {
                this.beenTrue = false;
                this.random = random;
            }
            private boolean get() {
                if (beenTrue) return false;
                return beenTrue = random.nextBoolean() && random.nextBoolean();
            }
        }

        /** The zero-based indices of the touched lines. */
        private final List<Integer> badLines;
        /** The original test on which this is based. */
        private final Path srcFile;
        /** The content of this test. */
        private final LinkedList<String> lines;
        /** Whether the error inserter is permitted to insert a line before the current line. */
        private final Predicate<ListIterator<String>> insertCondition;

        /**
         * Initialize a possibly altered copy of {@code originalTest}.
         * @param originalTest A path to an LF file that serves as a test.
         * @param insertCondition Whether the error inserter is permitted to insert a line between two given lines.
         * @throws IOException if the content of {@code originalTest} cannot be read.
         */
        private AlteredTest(Path originalTest, BiPredicate<String, String> insertCondition) throws IOException {
            this.badLines = new ArrayList<>();
            this.srcFile = originalTest;
            this.lines = new LinkedList<>();  // Constant-time insertion during iteration is desired.
            this.lines.addAll(Files.readAllLines(srcFile));
            this.insertCondition = it -> {
                it.previous();
                String s0 = it.previous();
                it.next();
                String s1 = it.next();
                return insertCondition.test(s0, s1);
            };
        }

        /** Return the location where the content of {@code this} lives. */
        public Path getSrcFile() {
            return srcFile;
        }

        /**
         * Write the altered version of the test to the file system.
         * @throws IOException If an I/O error occurred.
         */
        public void write() throws IOException {
            Path src = srcFile;
            if (!src.toFile().renameTo(swapFile(src).toFile())) {
                throw new IOException("Failed to create a swap file.");
            }
            try (PrintWriter writer = new PrintWriter(src.toFile())) {
                lines.forEach(writer::println);
            }
        }

        /**
         * Restore the file associated with this test to its original state.
         */
        @Override
        public void close() throws IOException {
            Path src = srcFile;
            if (!swapFile(src).toFile().exists()) throw new IllegalStateException("Swap file does not exist.");
            if (!src.toFile().delete()) {
                throw new IOException("Failed to delete the file associated with the original test.");
            }
            if (!swapFile(src).toFile().renameTo(src.toFile())) {
                throw new IOException("Failed to restore the altered LF file to its original state.");
            }
        }

        @Override
        public String toString() {
            int lengthOfPrefix = 6;
            StringBuilder ret = new StringBuilder(
                lines.stream().mapToInt(String::length).reduce(0, Integer::sum)
                + lines.size() * lengthOfPrefix
            );
            for (int i = 0; i < lines.size(); i++) {
                ret.append(badLines.contains(i) ? "-> " : "   ")
                   .append(String.format("%1$2s ", i))
                   .append(lines.get(i)).append("\n");
            }
            return ret.toString();
        }

        /** Return the lines where this differs from the test from which it was derived. */
        public ImmutableList<Integer> getBadLines() {
            return ImmutableList.copyOf(badLines);
        }

        /**
         * Attempt to replace a line of this test with a different line of target language code.
         * @param replacer A function that replaces lines of code with possibly different lines.
         */
        public void replace(Function<String, String> replacer, Random random) {
            OnceTrue onceTrue = new OnceTrue(random);
            alter((it, current) -> {
                if (!onceTrue.get()) return false;
                String newLine = replacer.apply(current);
                it.remove();
                it.add(newLine);
                return !newLine.equals(current);
            });
        }

        /**
         * Attempt to insert a new line of target language code into this test.
         * @param line The line to be inserted.
         */
        public void insert(String line, Random random) {
            OnceTrue onceTrue = new OnceTrue(random);
            alter((it, current) -> {
                if (insertCondition.test(it) && onceTrue.get()) {
                    it.previous();
                    it.add(line);
                    it.next();
                    return true;
                }
                return false;
            });
        }

        /**
         * Alter the content of this test.
         * @param alterer A function whose first argument is an iterator over the lines of {@code this}, whose second
         *                argument is the line most recently returned by that iterator, and whose return value is
         *                whether an alteration was successfully performed. This function is only applied within
         *                multiline code blocks.
         */
        private void alter(BiFunction<ListIterator<String>, String, Boolean> alterer) {
            ListIterator<String> it = lines.listIterator();
            boolean inCodeBlock = false;
            int lineNumber = 0;
            while (it.hasNext()) {
                String current = it.next();
                String uncommented = current.contains("//") ?
                    current.substring(0, current.indexOf("//")) : current;
                uncommented = uncommented.contains("#") ?
                    uncommented.substring(0, uncommented.indexOf("#")) : current;
                if (uncommented.contains("=}")) inCodeBlock = false;
                if (inCodeBlock && alterer.apply(it, current)) badLines.add(lineNumber);
                if (uncommented.contains("{=")) inCodeBlock = true;
                if (uncommented.contains("{=") && uncommented.contains("=}")) {
                    inCodeBlock = uncommented.lastIndexOf("{=") > uncommented.lastIndexOf("=}");
                }
                lineNumber++;
            }
        }

        /** Return the swap file associated with {@code f}. */
        private static Path swapFile(Path p) {
            return p.getParent().resolve("." + p.getFileName() + ".swp");
        }
    }

    /** A builder for an error inserter. */
    public static class Builder {
        private static class Node<T> implements Iterable<T> {
            private final Node<T> previous;
            private final T item;
            private Node(Node<T> previous, T item) {
                this.previous = previous;
                this.item = item;
            }

            @Override
            public Iterator<T> iterator() {
                NodeIterator<T> ret = new NodeIterator<>();
                ret.current = this;
                return ret;
            }

            private static class NodeIterator<T> implements Iterator<T> {
                private Node<T> current;

                @Override
                public boolean hasNext() {
                    return current != null;
                }

                @Override
                public T next() {
                    T ret = current.item;
                    current = current.previous;
                    return ret;
                }
            }
        }
        private final Node<Function<String, String>> replacers;
        private final Node<String> insertables;
        private final BiPredicate<String, String> insertCondition;

        /** Initializes a builder for error inserters. */
        public Builder() {
            this(null, null, (s0, s1) -> true);
        }

        /** Construct a builder with the given replacers and insertables. */
        private Builder(
            Node<Function<String, String>> replacers,
            Node<String> insertables,
            BiPredicate<String, String> insertCondition
        ) {
            this.replacers = replacers;
            this.insertables = insertables;
            this.insertCondition = insertCondition;
        }

        /**
         * Record that the resulting {@code ErrorInserter} may replace {@code phrase} with {@code alternativePhrase}.
         * @param phrase A phrase in target language code.
         * @param alternativePhrase A phrase that {@code phrase} may be replaced with in order to introduce an error.
         * @return A {@code Builder} that knows about all the edits that {@code this} knows about, plus the edit that
         * replaces {@code phrase} with {@code alternativePhrase}.
         */
        public Builder replacer(String phrase, String alternativePhrase) {
            return new Builder(
                new Node<>(
                    replacers,
                    line -> {
                        int changeableEnd = line.length();
                        for (String bad : new String[]{"#", "//", "\""}) {
                            if (line.contains(bad)) changeableEnd = Math.min(changeableEnd, line.indexOf(bad));
                        }
                        return line.substring(0, changeableEnd).replace(phrase, alternativePhrase)
                            + line.substring(changeableEnd);
                    }
                ),
                insertables,
                insertCondition
            );
        }

        /** Record that {@code} line may be inserted in order to introduce an error. */
        public Builder insertable(String line) {
            return new Builder(replacers, new Node<>(insertables, line), insertCondition);
        }

        /**
         * Record that for any lines X, Y, insertCondition(X, Y) is a necessary condition that a line may be inserted
         * between X and Y.
         */
        public Builder insertCondition(BiPredicate<String, String> insertCondition) {
            return new Builder(replacers, insertables, this.insertCondition.and(insertCondition));
        }

        /** Get the error inserter generated by {@code this}. */
        public ErrorInserter get(Random random) {
            return new ErrorInserter(
                random,
                replacers == null ? ImmutableList.of() : ImmutableList.copyOf(replacers),
                insertables == null ? ImmutableList.of() : ImmutableList.copyOf(insertables),
                insertCondition
            );
        }
    }

    private static final int MAX_ALTERATION_ATTEMPTS = 100;

    private final Random random;
    private final ImmutableList<Function<String, String>> replacers;
    private final ImmutableList<String> insertables;
    private final BiPredicate<String, String> insertCondition;

    private ErrorInserter(
        Random random,
        ImmutableList<Function<String, String>> replacers,
        ImmutableList<String> insertables,
        BiPredicate<String, String> insertCondition
    ) {
        this.random = random;
        this.replacers = replacers;
        this.insertables = insertables;
        this.insertCondition = insertCondition;
    }

    /**
     * Alter the given test and return the altered version.
     * @param test The path to the test.
     * @return An {@code AlteredTest} that is based on {@code test}.
     */
    public AlteredTest alterTest(Path test) throws IOException {
        AlteredTest alterable = new AlteredTest(test, insertCondition);
        int remainingAlterationAttempts = MAX_ALTERATION_ATTEMPTS;
        while (alterable.getBadLines().isEmpty() && remainingAlterationAttempts-- > 0) {
            if (random.nextBoolean() && !replacers.isEmpty()) {
                alterable.replace(replacers.get(random.nextInt(replacers.size())), random);
            } else if (!insertables.isEmpty()) {
                alterable.insert(insertables.get(random.nextInt(insertables.size())), random);
            }
        }
        alterable.write();
        return alterable;
    }
}
