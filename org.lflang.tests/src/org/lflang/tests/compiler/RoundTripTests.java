package org.lflang.tests.compiler;

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
        System.out.printf("Running formatter on %s...%n", file);
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        // TODO: Check that the output is a fixed point
        final int smallLineLength = 20;
        String squishedTestCase = FormattingUtils.render(originalModel, smallLineLength);
        System.out.printf(
            "The following is what %s looks like after applying formatting with the preferred line "
                + "length set to %d columns:%n%s%n%n",
            file.getFileName(),
            smallLineLength,
            squishedTestCase
        );
        Model resultingModel = getResultingModel(file, squishedTestCase);
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) {
            resultingModel.eResource().getErrors().forEach(System.err::println);
            Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty());
        }
        Assertions.assertTrue(new IsEqual(originalModel).doSwitch(resultingModel));
        String normalTestCase = FormattingUtils.render(originalModel);
        System.out.printf(
            "The following is what %s looks like after applying formatting normally:%n%s%n%n",
            file.getFileName(),
            normalTestCase
        );
        try {
            Assertions.assertEquals(
                Files.readString(file).replaceAll("\\r\\n?", "\n"),
                normalTestCase
            );
        } catch (AssertionFailedError e) {
            System.err.printf(
                "An assertion failed while checking that the content of %s is the same before and "
                    + "after formatting. Check that %s is formatted according to lff and the "
                    + "formatter provided with the VS Code extension.%n",
                file.getFileName(),
                file.getFileName()
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
