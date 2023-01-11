package org.lflang.generator;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.nio.file.Path;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.RuntimeIOException;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FedGenerator;
import org.lflang.generator.c.CFileConfig;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.python.PyFileConfig;
import org.lflang.generator.python.PythonGenerator;
import org.lflang.scoping.LFGlobalScopeProvider;

import com.google.inject.Inject;

/**
 * Generates code from your model files on save.
 *
 * See
 * https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
public class LFGenerator extends AbstractGenerator {

    @Inject
    private LFGlobalScopeProvider scopeProvider;

    // Indicator of whether generator errors occurred.
    protected boolean generatorErrorsOccurred = false;

    /**
     * Create a target-specific FileConfig object in Kotlin
     * <p>
     * Since the CppFileConfig and TSFileConfig class are implemented in Kotlin, the classes are
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * <p>
     * If the FileConfig class is found, this method returns an instance.
     * Otherwise, it returns an Instance of FileConfig.
     *
     * @return A FileConfig object in Kotlin if the class can be found.
     * @throws IOException If the file config could not be created properly
     */
    public static FileConfig createFileConfig(Resource resource, Path srcGenBasePath,
                                        boolean useHierarchicalBin) {

        final Target target = Target.fromDecl(ASTUtils.targetDecl(resource));
        assert target != null;

        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that FileConfig does not appear as an
        // import. Instead, we look the class up at runtime and instantiate it if
        // found.
        try {
            if (FedASTUtils.findFederatedReactor(resource) != null) {
                return new FedFileConfig(resource, srcGenBasePath, useHierarchicalBin);
            }
            switch (target) {
            case CCPP:
            case C: return new CFileConfig(resource, srcGenBasePath, useHierarchicalBin);
            case Python: return new PyFileConfig(resource, srcGenBasePath, useHierarchicalBin);
            case CPP:
            case Rust:
            case TS:
                String className = "org.lflang.generator." + target.packageName + "." + target.classNamePrefix + "FileConfig";
                try {
                    return (FileConfig) Class.forName(className)
                                             .getDeclaredConstructor(Resource.class, Path.class, boolean.class)
                                             .newInstance(resource, srcGenBasePath, useHierarchicalBin);
                } catch (ReflectiveOperationException e) {
                    throw new RuntimeException(
                        "Exception instantiating " + className, e.getCause());
                }
            default:
                throw new RuntimeException("Could not find FileConfig implementation for target " + target);
            }
        } catch (IOException e) {
            throw new RuntimeException("Unable to create FileConfig object for target " + target + ": " + e.getStackTrace());
        }
    }

    /**
     *  Create a generator object for the given target.
     *  Returns null if the generator could not be created.
     */
    private GeneratorBase createGenerator(LFGeneratorContext context) {
        final Target target = Target.fromDecl(ASTUtils.targetDecl(context.getFileConfig().resource));
        assert target != null;
        return switch (target) {
            case C -> new CGenerator(context, false);
            case CCPP -> new CGenerator(context, true);
            case Python -> new PythonGenerator(context);
            case CPP, TS, Rust ->
                createKotlinBaseGenerator(target, context);
            // If no case matched, then throw a runtime exception.
            default -> throw new RuntimeException("Unexpected target!");
        };
    }


    /**
     * Create a code generator in Kotlin.
     * <p>
     * Since the CppGenerator and TSGenerator class are implemented in Kotlin, the classes are
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * In this case, the method returns null
     *
     * @return A Kotlin Generator object if the class can be found
     */
    private GeneratorBase createKotlinBaseGenerator(Target target, LFGeneratorContext context) {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that Kotlin FileConfig and
        // Kotlin Generator do not appear as an import. Instead, we look the
        // class up at runtime and instantiate it if found.
        String classPrefix = "org.lflang.generator." + target.packageName + "." + target.classNamePrefix;
        try {
            Class<?> generatorClass = Class.forName(classPrefix + "Generator");
            Constructor<?> ctor = generatorClass
                .getDeclaredConstructor(LFGeneratorContext.class, LFGlobalScopeProvider.class);

            return (GeneratorBase) ctor.newInstance(context, scopeProvider);
        } catch (ReflectiveOperationException e) {
            generatorErrorsOccurred = true;
            context.getErrorReporter().reportError(
                "The code generator for the " + target + " target could not be found. "
                    + "This is likely because you built Epoch using "
                    + "Eclipse. The " + target + " code generator is written in Kotlin "
                    + "and, unfortunately, the plugin that Eclipse uses "
                    + "for compiling Kotlin code is broken. "
                    + "Please consider building Epoch using Maven.\n"
                    + "For step-by-step instructions, see: "
                    + "https://github.com/icyphy/lingua-franca/wiki/Running-Lingua-Franca-IDE-%28Epoch%29-with-Kotlin-based-Code-Generators-Enabled-%28without-Eclipse-Environment%29");
            return null;
        }
    }

    @Override
    public void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
        final LFGeneratorContext lfContext;
        if (context instanceof LFGeneratorContext) {
            lfContext = (LFGeneratorContext)context;
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
}
