package org.lflang.generator;

import java.io.IOException;

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
import org.lflang.generator.cpp.CppFileConfig;
import org.lflang.generator.cpp.CppGenerator;
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

    private FileConfig createFileConfig(final Target target, Resource resource,
            IFileSystemAccess2 fsa, IGeneratorContext context) throws IOException {
        switch (target) {
            case CPP: {
                return new CppFileConfig(resource, fsa, context);
            }
            case TS: {
                return new TypeScriptFileConfig(resource, fsa, context);
            }
            default: {
                return new FileConfig(resource, fsa, context);
            }
        }
    }
    
    private GeneratorBase createGenerator(Target target, FileConfig fileConfig, ErrorReporter errorReporter) {
        switch (target) {
            case C: {
                return new CGenerator(fileConfig, errorReporter);
            }
            case CCPP: {
                return new CCppGenerator(fileConfig, errorReporter);
            }
            case CPP: {
                return new CppGenerator((CppFileConfig) fileConfig, errorReporter, scopeProvider);
            }
            case TS: {
                return new TypeScriptGenerator((TypeScriptFileConfig) fileConfig, errorReporter);
            }
            case Python: {
                return new PythonGenerator(fileConfig, errorReporter);
            }
            default: {
                throw new RuntimeException("Unexpected target!");
            }
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
        final ErrorReporter errorReporter = new EclipseErrorReporter(fileConfig);
        final GeneratorBase generator = createGenerator(target, fileConfig, errorReporter);

        generator.doGenerate(resource, fsa, context);
        generatorErrorsOccurred = generator.errorsOccurred();
    }

    /** Return true if errors occurred in the last call to doGenerate(). */
    public boolean errorsOccurred() {
        return generatorErrorsOccurred;
    }
}
