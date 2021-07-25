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

    /** Create a FileConfig object for the given target */
    private FileConfig createFileConfig(final Target target, Resource resource,
            IFileSystemAccess2 fsa, IGeneratorContext context)
            throws IOException {
        switch (target) {
            case CPP: {
                return createCppFileConfig(resource, fsa, context);
            }
            case TS: {
//                return new TypeScriptFileConfig(resource, fsa, context);
                return createTsFileConfig(resource, fsa, context);
            }
            default: {
                return new FileConfig(resource, fsa, context);
            }
        }
    }

    /**
     * Create a C++ specific FileConfig object
     * 
     * Since the CppFileConfig class is implemented in Kotlin, the class is is
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * 
     * If the CppFileConfig class is found, this method returns an instance.
     * Otherwise, it returns an Instance of FileConfig.
     * 
     * @return A CppFileConfig object if the class can be found
     * @throws IOException
     */
    private FileConfig createCppFileConfig(Resource resource,
            IFileSystemAccess2 fsa, IGeneratorContext context)
            throws IOException {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that CppFileConfig does not appear as an
        // import. Instead we look the class up at runtime and instantiate it if
        // found.
        try {
            return (FileConfig) Class
                    .forName("org.lflang.generator.cpp.CppFileConfig")
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

    /**
     * Create a TypeScript specific FileConfig object
     *
     * Since the CppFileConfig class is implemented in Kotlin, the class is is
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     *
     * If the CppFileConfig class is found, this method returns an instance.
     * Otherwise, it returns an Instance of FileConfig.
     *
     * @return A CppFileConfig object if the class can be found
     * @throws IOException
     */
    private FileConfig createTsFileConfig(Resource resource,
                                          IFileSystemAccess2 fsa,
                                          IGeneratorContext context)
        throws IOException {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that CppFileConfig does not appear as an
        // import. Instead we look the class up at runtime and instantiate it if
        // found.
        try {
            return (FileConfig) Class
                .forName("org.lflang.generator.ts.TsFileConfig")
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
            case CPP: {
                return createCppGenerator(fileConfig, errorReporter);
            }
            case TS: {
//                return new TypeScriptGenerator(
//                        (TypeScriptFileConfig) fileConfig, errorReporter);
                return createTsGenerator(fileConfig, errorReporter);
            }
            case Python: {
                return new PythonGenerator(fileConfig, errorReporter);
            }
            default: {
                throw new RuntimeException("Unexpected target!");
            }
        }
    }

    /**
     * Create a C++ code generator
     * 
     * Since the CppGenerator class is implemented in Kotlin, the class is
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * In this case, the method returns null
     * 
     * @return A CppGenerator object if the class can be found
     */
    private GeneratorBase createCppGenerator(FileConfig fileConfig,
            ErrorReporter errorReporter) {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that CppFileConfig and CppGenerator do not
        // appear as an import. Instead we look the class up at runtime and
        // instantiate it if found.
        try {
            return (GeneratorBase) Class
                    .forName("org.lflang.generator.cpp.CppGenerator")
                    .getDeclaredConstructor(
                            Class.forName(
                                    "org.lflang.generator.cpp.CppFileConfig"),
                            ErrorReporter.class, LFGlobalScopeProvider.class)
                    .newInstance(fileConfig, errorReporter, scopeProvider);
        } catch (InstantiationException | IllegalAccessException
                | IllegalArgumentException | InvocationTargetException
                | NoSuchMethodException | SecurityException
                | ClassNotFoundException e) {
            generatorErrorsOccurred = true;
            errorReporter.reportError(
                    "The code generator for the C++ target could not be found. "
                            + "This is likely because you are running the RCA from Eclipse. "
                            + "The C++ code generator is written in Kotlin and, "
                            + "unfortunately, the Eclipse Kotlin plugin is broken, "
                            + "preventing us from loading the generator properly. "
                            + "Please consider building the RCA via Maven.");
            // FIXME: Add a link to the wiki with more information.
            return null;
        }
    }

    /**
     * Create a TypeScript code generator
     *
     * Since the Typeenerator class is implemented in Kotlin, the class is
     * not visible from all contexts. If the RCA is run from within Eclipse via
     * "Run as Eclipse Application", the Kotlin classes are unfortunately not
     * available at runtime due to bugs in the Eclipse Kotlin plugin. (See
     * https://stackoverflow.com/questions/68095816/is-ist-possible-to-build-mixed-kotlin-and-java-applications-with-a-recent-eclips)
     * In this case, the method returns null
     *
     * @return A TypeScriptGenerator object if the class can be found
     */
    private GeneratorBase createTsGenerator(FileConfig fileConfig,
                                            ErrorReporter errorReporter) {
        // Since our Eclipse Plugin uses code injection via guice, we need to
        // play a few tricks here so that TypeScriptFileConfig and
        // TypeScriptGenerator do not appear as an import. Instead we look the
        // class up at runtime and instantiate it if found.
        try {
            return (GeneratorBase) Class
                .forName("org.lflang.generator.ts.TsGenerator")
                .getDeclaredConstructor(
                    Class.forName(
                        "org.lflang.generator.ts.TsFileConfig"),
                    ErrorReporter.class, LFGlobalScopeProvider.class)
                .newInstance(fileConfig, errorReporter, scopeProvider);
        } catch (InstantiationException | IllegalAccessException
            | IllegalArgumentException | InvocationTargetException
            | NoSuchMethodException | SecurityException
            | ClassNotFoundException e) {
            generatorErrorsOccurred = true;
            errorReporter.reportError(
                "The code generator for the TypeScript target could not be found. "
                    + "This is likely because you are running the RCA from"
                    + "Eclipse. The TypeScript code generator is written in Kotlin"
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
