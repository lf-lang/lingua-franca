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
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.GeneratorResult.Status;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.generator.LanguageServerErrorReporter;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.lsp.ErrorInserter.AlteredTest;

/**
 * Test the code generator features that are required by the language server.
 */
class LspTests {

    /** The {@code Random} whose initial state determines the behavior of the set of all {@code LspTests} instances. */
    private static final Random RANDOM = new Random(2101);
    /** The maximum number of integration tests to validate for each target and category. */
    private static final int MAX_VALIDATIONS_PER_CATEGORY = 1000000;
    /** The test categories that should be excluded from LSP tests. */
    private static final TestCategory[] EXCLUDED_CATEGORIES = {
        TestCategory.EXAMPLE, TestCategory.DOCKER, TestCategory.DOCKER_FEDERATED
    };
    private static final Predicate<List<Diagnostic>> NOT_SUPPORTED = diagnosticsHaveKeyword("supported");
    private static final Predicate<List<Diagnostic>> MISSING_DEPENDENCY = diagnosticsHaveKeyword("libprotoc")
        .or(diagnosticsHaveKeyword("protoc-c")).or(diagnosticsIncludeText("could not be found"));

    /** The {@code IntegratedBuilder} instance whose behavior is to be tested. */
    private static final IntegratedBuilder builder = new LFStandaloneSetup(new LFRuntimeModule())
        .createInjectorAndDoEMFRegistration().getInstance(IntegratedBuilder.class);

    @Test
    void lspWithDependenciesTestC() { buildAndRunTest(Target.C); }
    @Test
    void lspWithDependenciesTestCpp() { buildAndRunTest(Target.CPP); }
    @Test
    void lspWithDependenciesTestPython() { buildAndRunTest(Target.Python); }
    @Test
    void lspWithDependenciesTestTypeScript() { buildAndRunTest(Target.TS); }
    @Test
    void lspWithDependenciesTestRust() { buildAndRunTest(Target.Rust); }

    /** Test for false negatives in C++ validation. */
    @Test
    void cppValidationTest() throws IOException {
        targetLanguageValidationTest(Target.CPP, ErrorInserter.CPP.get(RANDOM));
    }

    /** Test for false negatives in Python validation. */
    @Test
    void pythonValidationTest() throws IOException {
        targetLanguageValidationTest(Target.Python, ErrorInserter.PYTHON.get(RANDOM));
    }

    /** Test for false negatives in Rust validation. */
    @Test
    void rustValidationTest() throws IOException {
        targetLanguageValidationTest(Target.Rust, ErrorInserter.RUST.get(RANDOM));
    }

    /** Test for false negatives in TypeScript validation. */
    @Test
    void typescriptValidationTest() throws IOException {
        targetLanguageValidationTest(Target.TS, ErrorInserter.TYPESCRIPT.get(RANDOM));
    }

    private void targetLanguageValidationTest(Target target, ErrorInserter errorInserter) throws IOException {
        checkDiagnostics(
            target,
            alteredTest -> diagnostics -> alteredTest.getBadLines().stream().allMatch(
                badLine -> {
                    System.out.print("Expecting an error to be reported at line " + badLine + "...");
                    boolean result = diagnostics.stream().anyMatch(
                        diagnostic -> diagnostic.getRange().getStart().getLine() == badLine
                    );
                    System.out.println(result ? " Success." : " but the expected error could not be found.");
                    return result;
                }
            ),
            errorInserter,
            MAX_VALIDATIONS_PER_CATEGORY
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
     * @param count The maximum number of tests to validate from each category.
     * @throws IOException upon failure to write an altered copy of some test to storage.
     */
    private void checkDiagnostics(
        Target target,
        Function<AlteredTest, Predicate<List<Diagnostic>>> requirementGetter,
        ErrorInserter alterer,
        int count
    ) throws IOException {
        MockLanguageClient client = new MockLanguageClient();
        LanguageServerErrorReporter.setClient(client);
        for (LFTest test : selectTests(target, count)) {
            client.clearDiagnostics();
            if (alterer != null) {
                try (AlteredTest altered = alterer.alterTest(test.srcFile)) {
                    runTest(altered.getPath(), false);
                    Assertions.assertTrue(requirementGetter.apply(altered).test(client.getReceivedDiagnostics()));
                }
            } else {
                runTest(test.srcFile, false);
                Assertions.assertTrue(requirementGetter.apply(null).test(client.getReceivedDiagnostics()));
            }
        }
    }

    /** Test the "Build and Run" functionality of the language server. */
    private void buildAndRunTest(Target target) {
        MockLanguageClient client = new MockLanguageClient();
        LanguageServerErrorReporter.setClient(client);
        for (LFTest test : selectTests(target, 1)) {
            MockReportProgress reportProgress = new MockReportProgress();
            GeneratorResult result = runTest(test.srcFile, true);
            if (NOT_SUPPORTED.or(MISSING_DEPENDENCY).test(client.getReceivedDiagnostics())) {
                System.err.println("WARNING: Skipping \"Build and Run\" test due to lack of support or a missing "
                                       + "requirement.");
            } else {
                Assertions.assertFalse(reportProgress.failed());
                Assertions.assertEquals(Status.COMPILED, result.getStatus());
                Assertions.assertNotNull(result.getCommand());
                Assertions.assertEquals(result.getCommand().run(), 0);
            }
        }
    }

    /**
     * Select {@code count} tests from each test category.
     * @param target The target language of the desired tests.
     * @param count The number of tests to select per category.
     * @return A stratified sample of the integration tests for the given target.
     */
    private Set<LFTest> selectTests(Target target, int count) {
        Set<LFTest> ret = new HashSet<>();
        for (
            TestCategory category : (Iterable<? extends TestCategory>) () ->
                Arrays.stream(TestCategory.values()).filter(
                    category -> Arrays.stream(EXCLUDED_CATEGORIES).noneMatch(category::equals)
                ).iterator()
        ) {
            Set<LFTest> registeredTests = TestRegistry.getRegisteredTests(target, category, false);
            if (registeredTests.size() == 0) continue;
            Set<Integer> selectedIndices = RANDOM.ints(0, registeredTests.size())
                .limit(count).collect(HashSet::new, HashSet::add, HashSet::addAll);
            int i = 0;
            for (LFTest t : registeredTests) {
                if (selectedIndices.contains(i)) ret.add(t);
                i++;
            }
        }
        return ret;
    }

    /**
     * Returns the predicate that a list of diagnostics contains the given keyword.
     * @param keyword A keyword that a list of diagnostics should be searched for.
     * @return The predicate, "X mentions {@code keyword}."
     */
    private static Predicate<List<Diagnostic>> diagnosticsHaveKeyword(String keyword) {
        return diagnostics -> diagnostics.stream().anyMatch(
            d -> Arrays.asList(d.getMessage().toLowerCase().split("\\b")).contains(keyword)
        );
    }

    /**
     * Returns the predicate that a list of diagnostics contains the given text.
     * @param requiredText A keyword that a list of diagnostics should be searched for.
     * @return The predicate, "X includes {@code requiredText}."
     */
    private static Predicate<List<Diagnostic>> diagnosticsIncludeText(String requiredText) {
        return diagnostics -> diagnostics.stream().anyMatch(
            d -> d.getMessage().toLowerCase().contains(requiredText)
        );
    }

    /**
     * Run the given test.
     * @param test An integration test.
     * @param mustComplete Whether the build must be complete.
     * @return The result of running the test.
     */
    private GeneratorResult runTest(Path test, boolean mustComplete) {
        MockReportProgress reportProgress = new MockReportProgress();
        GeneratorResult result = builder.run(
            URI.createFileURI(test.toString()),
            mustComplete, reportProgress,
            () -> false
        );
        Assertions.assertFalse(reportProgress.failed());
        return result;
    }
}
