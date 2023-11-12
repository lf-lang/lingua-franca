package org.lflang.generator;

import com.google.inject.Inject;
import com.google.inject.Injector;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.RuntimeIOException;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedGenerator;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.generator.c.CFileConfig;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.cpp.CppFileConfig;
import org.lflang.generator.cpp.CppGenerator;
import org.lflang.generator.python.PyFileConfig;
import org.lflang.generator.python.PythonGenerator;
import org.lflang.generator.rust.RustFileConfig;
import org.lflang.generator.rust.RustGenerator;
import org.lflang.generator.ts.TSFileConfig;
import org.lflang.generator.ts.TSGenerator;
import org.lflang.scoping.LFGlobalScopeProvider;
import org.lflang.target.Target;

/** Generates code from your model files on save. */
public class LFGenerator extends AbstractGenerator {

  @Inject private LFGlobalScopeProvider scopeProvider;
  @Inject private Injector injector;

  // Indicator of whether generator errors occurred.
  protected boolean generatorErrorsOccurred = false;

  /**
   * Create a target-specific FileConfig object
   *
   * @return A FileConfig object in Kotlin if the class can be found.
   * @throws RuntimeException If the file config could not be created properly
   */
  public static FileConfig createFileConfig(
      Resource resource, Path srcGenBasePath, boolean useHierarchicalBin) {

    final Target target = Target.fromDecl(ASTUtils.targetDecl(resource));
    assert target != null;

    try {
      if (FedASTUtils.findFederatedReactor(resource) != null) {
        return new FederationFileConfig(resource, srcGenBasePath, useHierarchicalBin);
      }

      return switch (target) {
        case CCPP, C -> new CFileConfig(resource, srcGenBasePath, useHierarchicalBin);
        case Python -> new PyFileConfig(resource, srcGenBasePath, useHierarchicalBin);
        case CPP -> new CppFileConfig(resource, srcGenBasePath, useHierarchicalBin);
        case Rust -> new RustFileConfig(resource, srcGenBasePath, useHierarchicalBin);
        case TS -> new TSFileConfig(resource, srcGenBasePath, useHierarchicalBin);
      };
    } catch (IOException e) {
      throw new RuntimeException(
          "Unable to create FileConfig object for target "
              + target
              + ": "
              + Arrays.toString(e.getStackTrace()));
    }
  }

  /** Create a generator object for the given target. */
  private GeneratorBase createGenerator(LFGeneratorContext context) {
    final Target target = Target.fromDecl(ASTUtils.targetDecl(context.getFileConfig().resource));
    assert target != null;
    return switch (target) {
      case C -> new CGenerator(context, false);
      case CCPP -> new CGenerator(context, true);
      case Python -> new PythonGenerator(context);
      case CPP -> new CppGenerator(context, scopeProvider);
      case TS -> new TSGenerator(context);
      case Rust -> new RustGenerator(context, scopeProvider);
    };
  }

  @Override
  public void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
    assert injector != null;
    final LFGeneratorContext lfContext;
    if (context instanceof LFGeneratorContext) {
      lfContext = (LFGeneratorContext) context;
    } else {
      lfContext = LFGeneratorContext.lfGeneratorContextOf(resource, fsa, context);
    }

    // The fastest way to generate code is to not generate any code.
    if (lfContext.getMode() == LFGeneratorContext.Mode.LSP_FAST) return;

    if (FedASTUtils.findFederatedReactor(resource) != null) {
      try {
        FedGenerator fedGenerator = new FedGenerator(lfContext);
        injector.injectMembers(fedGenerator);
        generatorErrorsOccurred = fedGenerator.doGenerate(resource, lfContext);
      } catch (IOException e) {
        throw new RuntimeIOException("Error during federated code generation", e);
      }

    } else {
      final GeneratorBase generator = createGenerator(lfContext);

      if (generator != null) {
        generator.doGenerate(resource, lfContext);
        generatorErrorsOccurred = generator.errorsOccurred();
      }
    }
    final MessageReporter messageReporter = lfContext.getErrorReporter();
    if (messageReporter instanceof LanguageServerMessageReporter) {
      ((LanguageServerMessageReporter) messageReporter).publishDiagnostics();
    }
  }

  /** Return true if errors occurred in the last call to doGenerate(). */
  public boolean errorsOccurred() {
    return generatorErrorsOccurred;
  }
}
