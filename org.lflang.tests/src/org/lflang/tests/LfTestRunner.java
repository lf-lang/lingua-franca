/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.tests;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Properties;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;
import org.junit.jupiter.api.Assertions;
import org.junit.runner.Description;
import org.junit.runner.manipulation.Filter;
import org.junit.runner.manipulation.NoTestsRemainException;
import org.junit.runner.notification.RunNotifier;
import org.junit.runners.ParentRunner;
import org.junit.runners.model.InitializationError;
import org.junit.runners.model.Statement;

import org.lflang.FileConfig;
import org.lflang.generator.StandaloneContext;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Inject;
import com.google.inject.Provider;
import junit.framework.AssertionFailedError;

/**
 *
 */
public class LfTestRunner extends ParentRunner<LfTestDescriptor> {

    /**
     * Execution timeout enforced for all tests.
     */
    private static final long MAX_EXECUTION_TIME_SECONDS = 60;

    @Inject
    IResourceValidator validator;
    @Inject
    GeneratorDelegate generator;
    @Inject
    JavaIoFileSystemAccess fileAccess;
    @Inject
    Provider<ResourceSet> resourceSetProvider;

    private final LfTargetTestBase instance;
    private List<LfTestDescriptor> children;


    public LfTestRunner(Class<? extends LfTargetTestBase> testClass) throws InitializationError {
        super(testClass);
        try {
            this.instance = (LfTargetTestBase) getTestClass().getOnlyConstructor().newInstance();
        } catch (Exception e) {
            throw new InitializationError(e);
        }

        new LFInjectorProvider().getInjector().injectMembers(this);
    }


    @Override
    protected Description describeChild(LfTestDescriptor testCase) {
        return Description.createTestDescription(
            getTestClass().getJavaClass(),
            testCase.getTestName()
        );
    }



    @Override
    public void filter(Filter filter) throws NoTestsRemainException {
        super.filter(filter);
    }


    @Override
    protected List<LfTestDescriptor> getChildren() {
        if (children == null) {
            children = new ArrayList<>();
            for (TestCategory cat : TestCategory.values()) {
                for (LFTest test : TestRegistry.getRegisteredTests(instance.getTarget(), cat, false)) {
                    LfTestDescriptor descriptor = new LfTestDescriptor(
                        test.srcFile,
                        test.name,
                        cat,
                        test.packageRoot
                    );
                    children.add(descriptor);
                }
            }
        }
        return children;
    }


    @Override
    protected void runChild(LfTestDescriptor child, RunNotifier notifier) {
        Statement statement = getStatement(child);
        Description description = describeChild(child);
        runLeaf(statement, description, notifier);
    }


    private Statement getStatement(final LfTestDescriptor test) {
        return new LfTestStatement(test);
    }


    private final class LfTestStatement extends Statement {

        private final LfTestDescriptor test;


        private LfTestStatement(LfTestDescriptor test) {
            this.test = test;
        }


        @Override
        public void evaluate() throws Throwable {
            FileConfig fileConfig = configure();
            generateCode(fileConfig);
            if (test.shouldRun()) {
                execute(test, fileConfig);
            }
        }


        private void generateCode(FileConfig fileConfig) {
            try {
                generator.generate(fileConfig.resource, fileAccess, fileConfig.context);
            } catch (Exception e) {
                throw new AssertionError("Exception during code generation", e);
            }
        }


        private FileConfig configure() throws IOException {
            var context = new StandaloneContext();
            // Update file config, which includes a fresh resource that has not
            // been tampered with using AST transformations.
            context.setCancelIndicator(CancelIndicator.NullImpl);
            context.setArgs(new Properties());
            context.setPackageRoot(test.getPackageRoot());
            context.setHierarchicalBin(true);

            var r = resourceSetProvider.get().getResource(
                URI.createFileURI(test.getSrcFile().toFile().getAbsolutePath()),
                true);

            Assertions.assertEquals(0, r.getErrors().size(),
                                    "Unexpected parsing errors: " + r.getErrors());

            fileAccess.setOutputPath(context.getPackageRoot().resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
            var fileConfig = new FileConfig(r, fileAccess, context);

            // Validate the resource and store issues in the test object.
            List<Issue> issues = validator.validate(
                fileConfig.resource,
                CheckMode.ALL,
                context.getCancelIndicator()
            );
            List<String> testIssues = new ArrayList<>();
            if (issues != null && !issues.isEmpty()) {
                issues = new ArrayList<>(issues);
                issues.sort(Comparator.comparing(Issue::getSeverity));

                issues.stream().map(Objects::toString).forEach(testIssues::add);

                if (issues.stream().anyMatch(it -> it.getSeverity() == Severity.ERROR)) {
                    throw new AssertionFailedError("Unexpected errors:\n" + String.join("\n    ", testIssues));
                }
            }

            // Update the test by applying the configuration. E.g., to carry out an AST transformation.
            instance.configure(test, fileConfig);
            return fileConfig;
        }


        /**
         * Given an indexed test, execute it and label the test as failing if it
         * did not execute, took too long to execute, or executed but exited with
         * an error code.
         */
        private void execute(LfTestDescriptor test, FileConfig fileConfig) throws Exception {
            final ProcessBuilder pb;
            final var nameWithExtension = test.getSrcFile().getFileName().toString();
            final var nameOnly = nameWithExtension.substring(0, nameWithExtension.lastIndexOf('.'));

            switch (instance.getTarget()) {
            case C:
            case CPP:
            case CCPP: {
                var binPath = fileConfig.binPath;
                var binaryName = nameOnly;
                // Adjust binary extension if running on Window
                if (System.getProperty("os.name").startsWith("Windows")) {
                    binaryName = nameOnly + ".exe";
                }

                var fullPath = binPath.resolve(binaryName);
                if (Files.exists(fullPath)) {
                    // Running the command as .\binary.exe does not work on Windows for
                    // some reason... Thus we simply pass the full path here, which
                    // should work across all platforms
                    pb = new ProcessBuilder(fullPath.toString());
                    pb.directory(binPath.toFile());
                    break;
                } else {
                    throw new FileNotFoundException(fullPath.toString());
                }
            }
            case Python: {
                var srcGen = fileConfig.getSrcGenPath();
                var fullPath = srcGen.resolve(nameOnly + ".py");
                if (Files.exists(fullPath)) {
                    pb = new ProcessBuilder("python3", fullPath.toFile().getName());
                    pb.directory(srcGen.toFile());
                    break;
                } else {
                    throw new FileNotFoundException(fullPath.toString());
                }
            }
            case TS: {
                var dist = fileConfig.getSrcGenPath().resolve("dist");
                var file = dist.resolve(nameOnly + ".js");
                if (Files.exists(file)) {
                    pb = new ProcessBuilder("node", file.toString());
                    break;
                } else {
                    throw new FileNotFoundException(file.toString());
                }
            }
            default:
                throw new AssertionError("Unreachable");
            }
            var p = pb.start();
            if (!p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS)) {
                p.destroyForcibly();
                throw new TimeoutException("Timeout during test program execution");
            } else {
                Assertions.assertEquals(0, p.exitValue(), "Exit code");
            }
        }

    }

}
