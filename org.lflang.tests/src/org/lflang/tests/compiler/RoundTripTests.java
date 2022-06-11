package org.lflang.tests.compiler;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Function;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.EcoreUtil2;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.Target;
import org.lflang.ast.ToText;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

import com.google.inject.Inject;
import com.google.inject.Provider;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class RoundTripTests {
    @Inject
    IResourceValidator validator;

    @Inject
    Provider<ResourceSet> resourceSetProvider;

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
        String testCase = Files.readString(file);
        Resource resource = resourceSetProvider.get().getResource(
            URI.createFileURI(file.toFile().getAbsolutePath()),
            true
        );
        var issues = validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl);
        Assertions.assertTrue(issues.isEmpty());
//        EcoreUtil2.resolveAll(resource);
        EcoreUtil2.resolveLazyCrossReferences(resource, CancelIndicator.NullImpl);
        ResourceSet resourceSet = resource.getResourceSet();
//        EcoreUtil2.resolveAll(resourceSet);
        Model originalModel = parser.parse(testCase, resourceSet);
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        String reformattedTestCase = ToText.instance.doSwitch(originalModel);
        System.out.printf("Reformatted test case:%n%s%n%n", reformattedTestCase);
        Model resultingModel = parser.parse(reformattedTestCase, resourceSet);
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) {
            System.err.println(reformattedTestCase);
            resultingModel.eResource().getErrors().forEach(System.err::println);
            Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty());
        }
        checkSemanticallyEquivalent(originalModel, resultingModel);
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
