package org.lflang.tests.lsp;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;

import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.emf.common.util.URI;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.LanguageServerErrorReporter;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.lsp.ErrorInserter.AlteredTest;

/**
 * Test the code generator features that are required by the language server.
 *
 * @author Peter Donovan
 */
class LspTests {

    /** The test categories that should be excluded from LSP tests. */
    private static final TestCategory[] EXCLUDED_CATEGORIES = {
        TestCategory.SERIALIZATION, TestCategory.DOCKER, TestCategory.DOCKER_FEDERATED
    };
    private static final Predicate<List<Diagnostic>> NOT_SUPPORTED = diagnosticsHaveKeyword("supported");
    private static final Predicate<List<Diagnostic>> MISSING_DEPENDENCY = diagnosticsHaveKeyword("libprotoc")
        .or(diagnosticsHaveKeyword("protoc-c")).or(diagnosticsIncludeText("could not be found"));
    /** The number of samples to take from each test category (with replacement) when doing validation tests. */
    private static final int SAMPLES_PER_CATEGORY_VALIDATION_TESTS = 3;

    /** The {@code IntegratedBuilder} instance whose behavior is to be tested. */
    private static final IntegratedBuilder builder = new LFStandaloneSetup(new LFRuntimeModule())
        .createInjectorAndDoEMFRegistration().getInstance(IntegratedBuilder.class);

    /** Test for false negatives in Python syntax-only validation. */
    @Test
    void pythonValidationTestSyntaxOnly() throws IOException {
        targetLanguageValidationTest(Target.Python, ErrorInserter.PYTHON_SYNTAX_ONLY);
    }

    /** Test for false negatives in C++ validation. */
    @Test
    void cppValidationTest() throws IOException {
        targetLanguageValidationTest(Target.CPP, ErrorInserter.CPP);
    }

    /** Test for false negatives in Python validation. */
    @Test
    void pythonValidationTest() throws IOException {
        targetLanguageValidationTest(Target.Python, ErrorInserter.PYTHON);
    }

    /** Test for false negatives in Rust validation. */
    @Test
    void rustValidationTest() throws IOException {
        targetLanguageValidationTest(Target.Rust, ErrorInserter.RUST);
    }

    /** Test for false negatives in TypeScript validation. */
    @Test
    void typescriptValidationTest() throws IOException {
        targetLanguageValidationTest(Target.TS, ErrorInserter.TYPESCRIPT);
    }

    /**
     * Test for false negatives in the validation of LF files.
     * @param target The target language of the LF files to be validated.
     * @param builder A builder for the error inserter that will be used.
     */
    private void targetLanguageValidationTest(Target target, ErrorInserter.Builder builder) throws IOException {
        long seed = new Random().nextLong();
        System.out.printf("Running validation tests for %s with random seed %d.%n", target.getDisplayName(), seed);
        Random random = new Random(seed);
        int i = SAMPLES_PER_CATEGORY_VALIDATION_TESTS;
        while (i-- > 0) checkDiagnostics(
            target,
            alteredTest -> MISSING_DEPENDENCY.or(diagnostics -> alteredTest.getBadLines().stream().allMatch(
                badLine -> {
                    System.out.print("Expecting an error to be reported at line " + badLine + "...");
                    boolean result = NOT_SUPPORTED.test(diagnostics) || diagnostics.stream().anyMatch(
                        diagnostic -> diagnostic.getRange().getStart().getLine() == badLine
                    );
                    if (result) {
                        System.out.println(" Success.");
                    } else {
                        System.out.println(" but the expected error could not be found.");
                        System.out.printf(
                            "%s failed. Content of altered version of %s:%n%s%n",
                            alteredTest.getSrcFile(),
                            alteredTest.getSrcFile(),
                            TestBase.THIN_LINE
                        );
                        System.out.println(alteredTest + "\n" + TestBase.THIN_LINE);
                    }
                    return result;
                }
            )),
            builder.get(random),
            random
        );
    }

    /**
     * Verify that the diagnostics that result from fully validating tests associated with {@code target} satisfy
     * {@code requirementGetter}.
     * @param target Any target language.
     * @param requirementGetter A map from altered tests to the requirements that diagnostics regarding those tests
     * must meet.
     * @param alterer The means of inserting problems into the tests, or {@code null} if problems are not to be
     * inserted.
     * @param random The {@code Random} instance that determines which tests are selected.
     * @throws IOException upon failure to write an altered copy of some test to storage.
     */
    private void checkDiagnostics(
        Target target,
        Function<AlteredTest, Predicate<List<Diagnostic>>> requirementGetter,
        ErrorInserter alterer,
        Random random
    ) throws IOException {
        MockLanguageClient client = new MockLanguageClient();
        LanguageServerErrorReporter.setClient(client);
        for (LFTest test : selectTests(target, random)) {
            client.clearDiagnostics();
            if (alterer != null) {
                try (AlteredTest altered = alterer.alterTest(test.getSrcPath())) {
                    runTest(altered.getSrcFile());
                    Assertions.assertTrue(requirementGetter.apply(altered).test(client.getReceivedDiagnostics()));
                }
            } else {
                runTest(test.getSrcPath());
                Assertions.assertTrue(requirementGetter.apply(null).test(client.getReceivedDiagnostics()));
            }
        }
    }

    /**
     * Select a test from each test category.
     * @param target The target language of the desired tests.
     * @param random The {@code Random} instance that determines which tests are selected.
     * @return A sample of one integration test per target, per category.
     */
    private Set<LFTest> selectTests(Target target, Random random) {
        Set<LFTest> ret = new HashSet<>();
        for (TestCategory category : selectedCategories()) {
            Set<LFTest> registeredTests = TestRegistry.getRegisteredTests(target, category, false);
            if (registeredTests.size() == 0) continue;
            int relativeIndex = random.nextInt(registeredTests.size());
            for (LFTest t : registeredTests) {
                if (relativeIndex-- == 0) {
                    ret.add(t);
                    break;
                }
            }
        }
        return ret;
    }

    /** Return the non-excluded categories. */
    private Iterable<? extends TestCategory> selectedCategories() {
        return () -> Arrays.stream(TestCategory.values()).filter(
            category -> Arrays.stream(EXCLUDED_CATEGORIES).noneMatch(category::equals)
        ).iterator();
    }

    /**
     * Return the predicate that a list of diagnostics contains the given keyword.
     * @param keyword A keyword that a list of diagnostics should be searched for.
     * @return The predicate, "X mentions {@code keyword}."
     */
    private static Predicate<List<Diagnostic>> diagnosticsHaveKeyword(String keyword) {
        return diagnostics -> diagnostics.stream().anyMatch(
            d -> Arrays.asList(d.getMessage().toLowerCase().split("\\b")).contains(keyword)
        );
    }

    /**
     * Return the predicate that a list of diagnostics contains the given text.
     * @param requiredText A keyword that a list of diagnostics should be searched for.
     * @return The predicate, "X includes {@code requiredText}."
     */
    private static Predicate<List<Diagnostic>> diagnosticsIncludeText(@SuppressWarnings("SameParameterValue") String requiredText) {
        return diagnostics -> diagnostics.stream().anyMatch(
            d -> d.getMessage().toLowerCase().contains(requiredText)
        );
    }

    /**
     * Run the given test.
     * @param test The test b
     */
    private void runTest(Path test) {
        MockReportProgress reportProgress = new MockReportProgress();
        try {
            builder.run(
                URI.createFileURI(test.toString()),
                false, reportProgress,
                () -> false
            );
        } catch (Exception e) {
            e.printStackTrace();
            throw e;
        }
        Assertions.assertFalse(reportProgress.failed());
    }
}
