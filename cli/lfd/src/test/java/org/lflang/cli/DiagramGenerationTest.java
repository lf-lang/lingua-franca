package org.lflang.cli;

import com.google.inject.Inject;
import com.google.inject.Provider;
import de.cau.cs.kieler.klighd.Klighd;
import de.cau.cs.kieler.klighd.LightDiagramServices;
import de.cau.cs.kieler.klighd.standalone.KlighdStandaloneSetup;
import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import org.lflang.Target;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.TestRegistry;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
@Execution(ExecutionMode.SAME_THREAD)
public class DiagramGenerationTest {
  @Inject private TestRegistry testRegistry;

  @Inject private Provider<ResourceSet> resourceSetProvider;

  @TestFactory
  public Collection<DynamicTest> diagramGenerationTestFactory(@TempDir Path tempDir) {
    KlighdStandaloneSetup.initialize();
    List<DynamicTest> result = new ArrayList<>();
    Path cwd = Paths.get(".").toAbsolutePath();
    int id = 0;
    for (Target target : Target.values()) {
      for (TestRegistry.TestCategory category : TestRegistry.TestCategory.values()) {
        for (LFTest test : testRegistry.getRegisteredTests(target, category, false)) {
          URI testSourceUri = test.getSrcPath().toUri();
          final ResourceSet set = this.resourceSetProvider.get();
          Resource resource =
              set.getResource(
                  org.eclipse.emf.common.util.URI.createFileURI(test.getSrcPath().toString()),
                  true);
          Path outPath = tempDir.resolve("diagram_" + id + ".svg");
          id++;
          result.add(
              DynamicTest.dynamicTest(
                  cwd.relativize(test.getSrcPath()).toString(),
                  testSourceUri,
                  () -> run(resource, outPath.toString())));
        }
      }
    }
    return result;
  }

  private static void run(Resource resource, String outPath) {
    AtomicBoolean errorOccurred = new AtomicBoolean(false);
    Klighd.setStatusManager(
        (status, style) -> {
          if (status.getSeverity() == IStatus.ERROR) {
            System.out.println(status.getMessage());
            if (status.getException() != null) {
              status.getException().printStackTrace();
            }
          }
          errorOccurred.set(true);
        });

    final Model model = (Model) resource.getContents().get(0);
    IStatus status = LightDiagramServices.renderOffScreen(model, "svg", outPath);
    if (!status.isOK()) {
      System.out.println(status.getMessage());
    }
    assert status.isOK();
    assert !errorOccurred.get();
  }
}
