package org.lflang.tests.compiler;

import com.google.inject.Inject;
import com.google.inject.Provider;
import java.nio.file.Files;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.federated.generator.FedFileConfig;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGenerator;
import org.lflang.generator.LFGeneratorContext.Mode;
import org.lflang.generator.MainContext;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)

/**
 * Tests for checking that target properties adequately translate into the target configuration.
 */
class TargetConfigTests {

    @Inject
    ParseHelper<Model> parser;

    @Inject
    LFGenerator generator;

    @Inject
    JavaIoFileSystemAccess fileAccess;

    @Inject
    Provider<ResourceSet> resourceSetProvider;

    private void assertHasTargetProperty(Model model, String name) {
        Assertions.assertNotNull(model);
        Assertions.assertTrue(
            model.getTarget().getConfig().getPairs().stream().anyMatch(
                p -> p.getName().equals(name)
            )
        );
    }

    /**
     * Check that tracing target property affects the target configuration.
     * @throws Exception
     */
    @Test
    public void testParsing() throws Exception {
        assertHasTargetProperty(parser.parse("""
            target C {
              tracing: true
            }
            """), "tracing");
    }

    /**
     * Check that when a federation has the "tracing" target property set, the generated federates
     * will also have it set.
     * @throws Exception
     */
    @Test
    public void testFederation() throws Exception {
        fileAccess.setOutputPath("src-gen");

        Model federation = parser.parse("""
            target C {
              tracing: true
            }
            reactor Foo {
            
            }
            federated reactor {
                a = new Foo()
                b = new Foo()
            }
            """, URI.createFileURI("tmp/src/Federation.lf"), resourceSetProvider.get());
        assertHasTargetProperty(federation, "tracing");

        var resource = federation.eResource();
        var context = new MainContext(Mode.STANDALONE, resource, fileAccess, () -> false);

        if (GeneratorUtils.isHostWindows()) return;

        generator.doGenerate(resource, fileAccess, context);

        String lfSrc = Files.readAllLines(
            ((FedFileConfig)context.getFileConfig()).getSrcPath().resolve("federate__a.lf")
        ).stream().reduce("\n", String::concat);
        Model federate = parser.parse(lfSrc);
        assertHasTargetProperty(federate, "tracing");
    }
}
