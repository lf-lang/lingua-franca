package org.lflang.generator;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.generator.c.CGenerator;
import org.lflang.lf.TargetDecl;
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
     * Extract the target as specified in the target declaration from a given
     * resource
     */
    private Target getTarget(Resource resource) {
        // FIXME: is there a better way to do this in plain Java?
        final Iterable<EObject> contentIterable = IteratorExtensions
                .toIterable(resource.getAllContents());
        final Iterable<TargetDecl> targetDeclIterable = IterableExtensions
                .filter(contentIterable, TargetDecl.class);
        final String targetName = IterableExtensions.head(targetDeclIterable)
                .getName();
        return Target.forName(targetName);
    }

    private String getPackageName(Target target) {
        switch (target) {
            case CPP: {
                return "cpp";
            }
            case TS: {
                return "ts";
            }
            default: {
                throw new RuntimeException("Unexpected target!");
            }
        }
    }

    private String getClassNamePrefix(Target target) {
        switch (target) {
            case CPP: {
                return "Cpp";
            }
            case TS: {
                return "TS";
            }
            default: {
                throw new RuntimeException("Unexpected target!");
            }
        }
    }

    /**
     * Create a target-specific FileConfig object in Kotlin
     *
     * Since the CppFileConfig and TypeScriptFileConfig class are implemented in Kotlin, the classes are
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     *
     * If the FileConfig class is found, this method returns an instance.
     * Otherwise, it returns an Instance of FileConfig.
     *
     * @return A FileConfig object in Kotlin if the class can be found.
     * @throws IOException
     */
    private FileConfig createFileConfig(final Target target,
                                              Resource resource,
                                              IFileSystemAccess2 fsa,
                                              IGeneratorContext context)
        throws IOException {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that FileConfig does not appear as an
        // import. Instead we look the class up at runtime and instantiate it if
        // found.
        if (target != Target.CPP && target != Target.TS) {
            return new FileConfig(resource, fsa, context);
        }
        String packageName = getPackageName(target);
        String classNamePrefix = getClassNamePrefix(target);
        try {
            return (FileConfig) Class
                .forName("org.lflang.generator." + packageName + "." + classNamePrefix + "FileConfig")
                .getDeclaredConstructor(Resource.class,
                                        IFileSystemAccess2.class, IGeneratorContext.class)
                .newInstance(resource, fsa, context);
        } catch (InstantiationException | IllegalAccessException
            | IllegalArgumentException | InvocationTargetException
            | NoSuchMethodException | SecurityException
            | ClassNotFoundException e) {
            return new FileConfig(resource, fsa, context);
        }
    }

    /** Create a generator object for the given target */
    private GeneratorBase createGenerator(Target target, FileConfig fileConfig,
            ErrorReporter errorReporter) {
        switch (target) {
            case C: {
                return new CGenerator(fileConfig, errorReporter);
            }
            case CCPP: {
                return new CCppGenerator(fileConfig, errorReporter);
            }
            case Python: {
                return new PythonGenerator(fileConfig, errorReporter);
            }
            default: {
                return createKotlinGenerator(target, fileConfig, errorReporter);
            }
        }
    }

    /**
     * Create a code generator in Kotlin.
     *
     * Since the CppGenerator and TSGenerator class are implemented in Kotlin, the classes are
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * In this case, the method returns null
     *
     * @return A Kotlin Generator object if the class can be found
     */
    private GeneratorBase createKotlinGenerator(Target target, FileConfig fileConfig,
                                            ErrorReporter errorReporter) {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that Kotlin FileConfig and
        // Kotlin Generator do not appear as an import. Instead we look the
        // class up at runtime and instantiate it if found.
        String packageName = getPackageName(target);
        String classNamePrefix = getClassNamePrefix(target);
        try {
            return (GeneratorBase) Class
                .forName("org.lflang.generator." + packageName + "." + classNamePrefix + "Generator")
                .getDeclaredConstructor(
                    Class.forName(
                        "org.lflang.generator." + packageName + "." + classNamePrefix + "FileConfig"),
                    ErrorReporter.class, LFGlobalScopeProvider.class)
                .newInstance(fileConfig, errorReporter, scopeProvider);
        } catch (InstantiationException | IllegalAccessException
            | IllegalArgumentException | InvocationTargetException
            | NoSuchMethodException | SecurityException
            | ClassNotFoundException e) {
            generatorErrorsOccurred = true;
            errorReporter.reportError(
                "The code generator for the " + target + " target could not be found. "
                    + "This is likely because you are running the RCA from"
                    + "Eclipse. The " + target + " code generator is written in Kotlin"
                    + "and, unfortunately, the Eclipse Kotlin plugin is "
                    + "broken, preventing us from loading the generator"
                    + "properly. Please consider building the RCA via Maven.");
            // FIXME: Add a link to the wiki with more information.
            return null;
        }
    }

    @Override
    public void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
        // Determine which target is desired.
        final Target target = getTarget(resource);

        FileConfig fileConfig;
        try {
            fileConfig = createFileConfig(target, resource, fsa, context);
        } catch (IOException e) {
            throw new RuntimeException("Error during FileConfig instaniation");
        }
        final ErrorReporter errorReporter = new EclipseErrorReporter(
                fileConfig);
        final GeneratorBase generator = createGenerator(target, fileConfig,
                errorReporter);

        if (generator != null) {
            generator.doGenerate(resource, fsa, context);
            generatorErrorsOccurred = generator.errorsOccurred();
        }
    }

    /** Return true if errors occurred in the last call to doGenerate(). */
    public boolean errorsOccurred() {
        return generatorErrorsOccurred;
    }
}
