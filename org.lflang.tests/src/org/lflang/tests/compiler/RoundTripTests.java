package org.lflang.tests.compiler;

import static java.util.Collections.emptyList;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.opentest4j.AssertionFailedError;

import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.ast.FormattingUtils;
import org.lflang.ast.IsEqual;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Injector;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class RoundTripTests {

    @Test
    public void roundTripTest() throws Exception {
        for (Target target : Target.values()) {
            for (TestCategory category : TestCategory.values()) {
                for (LFTest test : TestRegistry.getRegisteredTests(target, category, false)) {
                    run(test.srcFile);
                }
            }
        }
    }

    private void run(Path file) throws Exception {
        Model originalModel = parse(file);
        assertThat(originalModel.eResource().getErrors(), equalTo(emptyList()));
        // TODO: Check that the output is a fixed point
        final int smallLineLength = 20;
        final String squishedTestCase = FormattingUtils.render(originalModel, smallLineLength);
        final Model resultingModel = getResultingModel(file, squishedTestCase);
        Assertions.assertNotNull(resultingModel);
        assertThat(resultingModel.eResource().getErrors(), equalTo(emptyList()));
        if (!new IsEqual(originalModel).doSwitch(resultingModel)) {
            System.out.printf(
                "The following is what %s looks like after applying formatting with the preferred line "
                    + "length set to %d columns:%n%s%n%n",
                file,
                smallLineLength,
                squishedTestCase
            );
            throw new junit.framework.AssertionFailedError(String.format(
                "The reformatted version of %s was not equivalent to the original file.",
                file.getFileName()
            ));
        }
        final String normalTestCase = FormattingUtils.render(originalModel);
        try {
            assertEquals(
                normalTestCase,
                Files.readString(file).replaceAll("\\r\\n?", "\n")
            );
        } catch (AssertionFailedError e) {
            System.err.printf(
                "An assertion failed while checking that the content of %s is the same before and "
                    + "after formatting. Check that %s is formatted according to lff and the "
                    + "formatter provided with the VS Code extension.%n",
                file,
                file.getFileName()
            );
            System.out.printf(
                "The following is what %s looks like after applying formatting normally:%n%s%n%n",
                file.getFileName(),
                normalTestCase
            );
            throw e;
        }
    }

    private Model getResultingModel(
        Path file,
        String reformattedTestCase
    ) throws IOException {
        final Path swap = file.getParent().resolve(file.getFileName().toString() + ".swp");
        Files.move(file, swap, StandardCopyOption.REPLACE_EXISTING);
        try (PrintWriter out = new PrintWriter(file.toFile())) {
            out.println(reformattedTestCase);
        }
        Model resultingModel = parse(file);
        Files.move(swap, file, StandardCopyOption.REPLACE_EXISTING);
        return resultingModel;
    }

    private Model parse(Path file) {
        // Source: https://wiki.eclipse.org/Xtext/FAQ#How_do_I_load_my_model_in_a_standalone_Java_application_.3F
        Injector injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
        XtextResourceSet resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
        Resource resource = resourceSet.getResource(
            URI.createFileURI(file.toFile().getAbsolutePath()),
            true
        );
        return (Model) resource.getContents().get(0);
    }
}
