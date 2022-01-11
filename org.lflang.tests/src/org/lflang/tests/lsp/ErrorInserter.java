package org.lflang.tests.lsp;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.Function;

import org.eclipse.xtext.util.RuntimeIOException;
import org.jetbrains.annotations.NotNull;

import org.lflang.tests.TestRegistry;

import com.google.common.collect.ImmutableList;

/**
 * Insert problems into integration tests.
 */
class ErrorInserter {

    /** A basic error inserter builder on which more specific error inserters can be built. */
    private static final Builder BASE_ERROR_INSERTER = new ErrorInserter.Builder()
        .insertable("    0 = 1;").insertable("some_undeclared_var1524263 = 9;").insertable("        ++;");
    public static final Builder C = BASE_ERROR_INSERTER
        .replacer("SET(", "UNDEFINED_NAME2828376(")
        .replacer("schedule(", "undefined_name15291838(");
    public static final Builder CPP = BASE_ERROR_INSERTER
        .replacer(".get", ".undefined_name15291838")
        .replacer("std::", "undefined_name3286634::");
    public static final Builder PYTHON = new Builder()
        .insertable("+++++").replacer(".", "..");
    public static final Builder RUST = BASE_ERROR_INSERTER
        .replacer("println!", "undefined_name15291838!")
        .replacer("ctx", "undefined_name3286634!");
    public static final Builder TYPESCRIPT = BASE_ERROR_INSERTER
        .replacer("requestErrorStop(", "not_an_attribute_of_util9764(")
        .replacer("const ", "var ");

    static class AlteredTest {

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
        /** The file to which the altered version of the original LF file should be written. */
        private final File file;
        /** The content of this test. */
        private final LinkedList<String> lines;

        /**
         * Initialize a possibly altered copy of {@code originalTest}.
         * @param originalTest A path to an LF file that serves as a test.
         * @throws IOException if the content of {@code originalTest} cannot be read.
         */
        public AlteredTest(Path originalTest) throws IOException {
            this.badLines = new ArrayList<>();
            this.file = tempFileOf(originalTest).toFile();
            this.lines = new LinkedList<>();  // Constant-time insertion during iteration is desired.
            this.lines.addAll(Files.readAllLines(originalTest));
        }

        /** Return the file where the content of {@code this} lives. */
        public File getFile() {
            return file;
        }

        /**
         * Write the altered version of the test to the file system.
         * @throws IOException If an I/O error occurred.
         */
        public void write() throws IOException {
            if (!file.exists()) copyTestDir();
            try (PrintWriter writer = new PrintWriter(file)) {
                lines.forEach(writer::println);
            }
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
                if (onceTrue.get()) {
                    it.remove();
                    it.add(line);
                    it.add(current);
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
                if (current.contains("=}")) inCodeBlock = false;
                if (inCodeBlock && alterer.apply(it, current)) badLines.add(lineNumber);
                if (current.contains("{=")) inCodeBlock = true;
                if (current.contains("{=") && current.contains("=}")) {
                    inCodeBlock = current.lastIndexOf("{=") > current.lastIndexOf("=}");
                }
                lineNumber++;
            }
        }

        /**
         * Return the file location of the temporary copy of {@code test}.
         * @param test An LF file that can be used in tests.
         * @return The file location of the temporary copy of {@code test}.
         */
        private static Path tempFileOf(Path test) {
            return ALTERED_TEST_DIR.resolve(TestRegistry.LF_TEST_PATH.relativize(test));
        }

        /**
         * Initialize the error insertion process by recursively copying the test directory to a temporary directory.
         * @throws IOException If an I/O error occurs.
         */
        private static void copyTestDir() throws IOException {
            Files.walkFileTree(TestRegistry.LF_TEST_PATH, new SimpleFileVisitor<>() {

                private int depth = 0;

                @Override
                public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) throws IOException {
                    if (depth == 2 && !dir.getName(dir.getNameCount() - 1).toString().equals("src")) {
                        return FileVisitResult.SKIP_SUBTREE;
                    }
                    depth++;
                    Files.createDirectories(tempFileOf(dir));
                    return FileVisitResult.CONTINUE;
                }

                @Override
                public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
                    Files.copy(file, ALTERED_TEST_DIR.resolve(TestRegistry.LF_TEST_PATH.relativize(file)));
                    return FileVisitResult.CONTINUE;
                }

                @Override
                public FileVisitResult postVisitDirectory(Path dir, IOException exc) {
                    depth--;
                    return FileVisitResult.CONTINUE;
                }
            });
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

            @NotNull
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

        /** Initializes a builder for error inserters. */
        public Builder() {
            this(null, null);
        }

        /** Construct a builder with the given replacers and insertables. */
        private Builder(Node<Function<String, String>> replacers, Node<String> insertables) {
            this.replacers = replacers;
            this.insertables = insertables;
        }

        /**
         * Record that the resulting {@code ErrorInserter} may replace {@code phrase} with {@code alternativePhrase}.
         * @param phrase A phrase in target language code.
         * @param alternativePhrase A phrase that {@code phrase} may be replaced with in order to introduce an error.
         * @return A {@code Builder} that knows about all the edits that {@code this} knows about, plus the edit that
         * replaces {@code phrase} with {@code alternativePhrase}.
         */
        public Builder replacer(String phrase, String alternativePhrase) {
            return new Builder(new Node<>(replacers, line -> line.replace(phrase, alternativePhrase)), insertables);
        }

        /** Record that {@code} line may be inserted in order to introduce an error. */
        public Builder insertable(String line) {
            return new Builder(replacers, new Node<>(insertables, line));
        }

        /** Get the error inserter generated by {@code this}. */
        public ErrorInserter get(Random random) {
            return new ErrorInserter(random, ImmutableList.copyOf(replacers), ImmutableList.copyOf(insertables));
        }
    }

    private static final int MAX_ALTERATION_ATTEMPTS = 100;
    private static final Path ALTERED_TEST_DIR;

    static {
        try {
            ALTERED_TEST_DIR = Files.createTempDirectory("lingua-franca-altered-tests");
        } catch (IOException e) {
            throw new RuntimeIOException(e);
        }
    }

    private final Random random;
    private final ImmutableList<Function<String, String>> replacers;
    private final ImmutableList<String> insertables;

    private ErrorInserter(
        Random random,
        ImmutableList<Function<String, String>> replacers,
        ImmutableList<String> insertables
    ) {
        this.random = random;
        this.replacers = replacers;
        this.insertables = insertables;
    }

    /**
     * Alter the given test and return the altered version.
     * @param test An LF file that can be used as a test.
     * @return An {@code AlteredTest} that is based on {@code test}.
     */
    public AlteredTest alterTest(Path test) throws IOException {
        AlteredTest alterable = new AlteredTest(test);
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
