package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.Target;
import org.lflang.lf.TargetDecl;

/**
 * Generates code from your model files on save.
 * 
 * See
 * https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
public class LFGenerator extends AbstractGenerator {

    // Indicator of whether generator errors occurred.
    protected boolean generatorErrorsOccurred = false;

    public void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
        // Determine which target is desired.
        GeneratorBase generator = null;

        // first find the target decleration
        // is there a better way to do this in plain Java?
        final Iterable<EObject> contentIterable = IteratorExtensions
                .toIterable(resource.getAllContents());
        final Iterable<TargetDecl> targetDeclIterable = IterableExtensions
                .filter(contentIterable, TargetDecl.class);
        final String targetName = IterableExtensions.head(targetDeclIterable)
                .getName();

        // Get the actual target object
        Target target = Target.forName(targetName);

        switch (target) {
            case C: {
                generator = new CGenerator();
                break;
            }
            case CCPP: {
                generator = new CCppGenerator();
                break;
            }
            case CPP: {
                generator = new CppGenerator();
                break;
            }
            case TS: {
                generator = new TypeScriptGenerator();
                break;
            }
            case Python: {
                generator = new PythonGenerator();
                break;
            }
        }

        generator.doGenerate(resource, fsa, context);
        generatorErrorsOccurred = generator.errorsOccurred();
    }

    /** Return true if errors occurred in the last call to doGenerate(). */
    public boolean errorsOccurred() {
        return generatorErrorsOccurred;
    }
}
