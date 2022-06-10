package org.lflang.tests.compiler;

import java.nio.file.Files;
import java.util.function.Function;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
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

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class RoundTripTests {
    @Inject
    ParseHelper<Model> parser;

    @Test
    public void roundTripTest() throws Exception {
        for (Target target : Target.values()) {
            for (TestCategory category : TestCategory.values()) {
                for (LFTest test : TestRegistry.getRegisteredTests(target, category, false)) {
                    run(test);
                }
            }
        }
    }

    private void run(LFTest test) throws Exception {
        String testCase = Files.readString(test.srcFile);
        Model originalModel = parser.parse(testCase);
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        String reformattedTestCase = ToText.instance.doSwitch(originalModel);
        Model resultingModel = parser.parse(reformattedTestCase);
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) System.err.println(reformattedTestCase);
        Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty());
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
