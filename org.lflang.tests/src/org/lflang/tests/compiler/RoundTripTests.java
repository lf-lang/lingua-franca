package org.lflang.tests.compiler;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.nio.file.Path;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.ast.IsEqual;
import org.lflang.ast.MalleableString;
import org.lflang.ast.ToLf;
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
        int nonFailures = 0;
        for (Target target : Target.values()) {
            for (TestCategory category : TestCategory.values()) {
                for (LFTest test : TestRegistry.getRegisteredTests(target, category, false)) {
                    run(test.srcFile);
                    System.out.printf("%s non-failures%n", ++nonFailures);
                }
            }
        }
    }

    private void run(Path file) throws Exception {
        Model originalModel = parse(file);
        System.out.printf("Running formatter on %s%n", file);
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        MalleableString ms = new MalleableString.Builder().append(ToLf.instance.doSwitch(originalModel)).get();
        ms.findBestRepresentation(ms::toString, ToLf.astRepresentationComparator(30), 30);
        String reformattedTestCase = ms.toString();
        System.out.printf("Reformatted test case:%n%s%n%n", reformattedTestCase);
        Model resultingModel = getResultingModel(file, reformattedTestCase);
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) {
            resultingModel.eResource().getErrors().forEach(System.err::println);
            Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty());
        }
        Assertions.assertTrue(new IsEqual(originalModel).doSwitch(resultingModel));
    }

    private Model getResultingModel(Path file, String reformattedTestCase) throws FileNotFoundException {
        File swap = file.getParent().resolve(file.getFileName().toString() + ".swp").toFile();
        file.toFile().renameTo(swap); // FIXME: renameTo may fail.
        try (PrintWriter out = new PrintWriter(file.toFile())) {
            out.println(reformattedTestCase);
        }
        Model resultingModel = parse(file);
        swap.renameTo(file.toFile());
        return resultingModel;
    }

    private Model parse(Path file) {
        // Source: https://wiki.eclipse.org/Xtext/FAQ#How_do_I_load_my_model_in_a_standalone_Java_application_.3F
        Injector injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
        XtextResourceSet resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
        Resource resource = resourceSet.getResource(URI.createFileURI(file.toFile().getAbsolutePath()), true);
        return (Model) resource.getContents().get(0);
    }
}
