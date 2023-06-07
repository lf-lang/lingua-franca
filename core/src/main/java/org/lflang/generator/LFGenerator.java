package org.lflang.generator;

import com.google.inject.Inject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Constructor;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;
import java.util.Arrays;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.RuntimeIOException;
import org.lflang.AttributeUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.analyses.uclid.UclidGenerator;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FedGenerator;
import org.lflang.lf.Attribute;
import org.lflang.lf.Reactor;
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

/** Generates code from your model files on save. */
public class LFGenerator extends AbstractGenerator {

  @Inject private LFGlobalScopeProvider scopeProvider;

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
        return new FedFileConfig(resource, srcGenBasePath, useHierarchicalBin);
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
      case TS -> new TSGenerator(context, scopeProvider);
      case Rust -> new RustGenerator(context, scopeProvider);
    };
  }

  @Override
  public void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
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
        generatorErrorsOccurred = (new FedGenerator(lfContext)).doGenerate(resource, lfContext);
      } catch (IOException e) {
        throw new RuntimeIOException("Error during federated code generation", e);
      }

    } else {

      // If "-c" or "--clean" is specified, delete any existing generated directories.
      cleanIfNeeded(lfContext);

      // If @property annotations are used, run the LF verifier.
      runVerifierIfPropertiesDetected(resource, lfContext);

      final GeneratorBase generator = createGenerator(lfContext);

      if (generator != null) {
        generator.doGenerate(resource, lfContext);
        generatorErrorsOccurred = generator.errorsOccurred();
      }
    }
    final ErrorReporter errorReporter = lfContext.getErrorReporter();
    if (errorReporter instanceof LanguageServerErrorReporter) {
      ((LanguageServerErrorReporter) errorReporter).publishDiagnostics();
    }
  }

  /** Return true if errors occurred in the last call to doGenerate(). */
  public boolean errorsOccurred() {
    return generatorErrorsOccurred;
  }

  /**
   * Check if a clean was requested from the standalone compiler and perform
   * the clean step.
   * 
   * FIXME: the signature can be reduced to only take context.
   */
  protected void cleanIfNeeded(LFGeneratorContext context) {
    if (context.getArgs().containsKey("clean")) {
      try {
        context.getFileConfig().doClean();
      } catch (IOException e) {
        System.err.println("WARNING: IO Error during clean");
      }
    }
  }

  /**
   * Check if @property is used. If so, instantiate a UclidGenerator.
   * The verification model needs to be generated before the target code
   * since code generation changes LF program (desugar connections, etc.).
   */
  private void runVerifierIfPropertiesDetected(Resource resource, LFGeneratorContext lfContext) {
    Reactor main = ASTUtils.getMainReactor(resource);
    List<Attribute> properties = AttributeUtils.getAttributes(main)
                                .stream()
                                .filter(attr -> attr.getAttrName().equals("property"))
                                .collect(Collectors.toList());
    if (properties.size() > 0) {

      System.out.println("*** WARNING: @property is an experimental feature. Use it with caution. ***");

      // Check if Uclid5 and Z3 are installed.
      if (execInstalled("uclid", "--help", "uclid 0.9.5")
          && execInstalled("z3", "--version", "Z3 version")) {
        UclidGenerator uclidGenerator = new UclidGenerator(lfContext, properties);
        // Generate uclid files.
        uclidGenerator.doGenerate(resource, lfContext);
        if (!uclidGenerator.targetConfig.noVerify) {
          // Invoke the generated uclid files.
          uclidGenerator.runner.run();
        } else {
          System.out.println("\"no-verify\" is set to true. Skip checking the verification model.");
        }
      } else {
        System.out.println("*** WARNING: Uclid5 or Z3 is not installed. @property is skipped. ***");
      }
    }
  }

  /**
   * A helper function for checking if a dependency is installed on the command line.
   * 
   * @param binaryName The name of the binary
   * @param arg An argument following the binary name
   * @param expectedSubstring An expected substring in the output
   * @return
   */
  public static boolean execInstalled(String binaryName, String arg, String expectedSubstring) {
    ProcessBuilder processBuilder = new ProcessBuilder(binaryName, arg);
    try {
      Process process = processBuilder.start();
      BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
      String line;
      while ((line = reader.readLine()) != null) {
        if (line.contains(expectedSubstring)) {
          return true;
        }
      }
    } catch (IOException e) {
      return false; // binary not present
    }
    return false;
  }
}
