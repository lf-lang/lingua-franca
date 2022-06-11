package org.lflang.tests.compiler;

import java.nio.file.Path;
import java.util.function.Function;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.LFStandaloneSetup;
import org.lflang.Target;
import org.lflang.ast.ToLf;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Inject;
import com.google.inject.Injector;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class RoundTripTests {
    @Inject
    XtextResourceSet resourceSet;

    @Inject
    ParseHelper<Model> parser;

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
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        String reformattedTestCase = ToLf.instance.doSwitch(originalModel);
        System.out.printf("Reformatted test case:%n%s%n%n", reformattedTestCase);
        Model resultingModel = parser.parse(reformattedTestCase, resourceSet);
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) {
            resultingModel.eResource().getErrors().forEach(System.err::println);
            Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty());
        }
        checkSemanticallyEquivalent(originalModel, resultingModel);
    }

    private Model parse(Path file) {
        // Source: https://wiki.eclipse.org/Xtext/FAQ#How_do_I_load_my_model_in_a_standalone_Java_application_.3F
        Injector injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
        XtextResourceSet resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
        Resource resource = resourceSet.getResource(URI.createFileURI(file.toFile().getAbsolutePath()), true);
        return (Model) resource.getContents().get(0);
    }

    private void checkSemanticallyEquivalent(Model originalModel, Model resultingModel) {
        // FIXME: This is a toy implementation. It is no good.
        Function<TreeIterator<EObject>, Integer> getSize = contents -> {
            int count = 0;
            for (; contents.hasNext(); contents.next()) count++;
            return count;
        };
        Assertions.assertEquals(
            getSize.apply(originalModel.eAllContents()),
            getSize.apply(resultingModel.eAllContents())
        );
    }
}
