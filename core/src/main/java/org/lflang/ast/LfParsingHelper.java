package org.lflang.ast;

import com.google.inject.Inject;
import com.google.inject.Injector;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.lflang.LFStandaloneSetup;
import org.lflang.lf.Model;

/**
 * Utility to parse LF classes. Not static so that we can reuse the injector, as dependency
 * injection takes a lot of time.
 *
 * @author Clément Fournier
 */
public class LfParsingHelper {

  private final Injector injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
  @Inject XtextResourceSet resourceSet;

  public Model parse(Path file) {
    // Source:
    // https://wiki.eclipse.org/Xtext/FAQ#How_do_I_load_my_model_in_a_standalone_Java_application_.3F
    XtextResourceSet resourceSet =
        this.resourceSet != null ? this.resourceSet : injector.getInstance(XtextResourceSet.class);
    resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    Resource resource =
        resourceSet.getResource(URI.createFileURI(file.toFile().getAbsolutePath()), true);
    return (Model) resource.getContents().get(0);
  }

  public Model parse(String fileContents) {
    Path file = null;
    try {
      file = Files.createTempFile("lftests", ".lf");
      Files.writeString(file, fileContents);
      return parse(file);
    } catch (IOException e) {
      throw new RuntimeException(e);
    } finally {
      if (file != null) {
        try {
          Files.deleteIfExists(file);
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      }
    }
  }

  public Model parseSourceAsIfInDirectory(Path directory, String sourceText) {
    // Use nontrivial number to avoid collisions. This prevents TOCTOU errors
    // which would show up when running tests concurrently.
    int num = sourceText.hashCode();
    while (Files.exists(directory.resolve("file" + num + ".lf"))) {
      num++;
    }
    Path file = directory.resolve("file" + num + ".lf");
    try {
      Files.writeString(file, sourceText);
      return parse(file);
    } catch (IOException e) {
      throw new RuntimeException(e);
    } finally {
      try {
        Files.deleteIfExists(file);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }
}
