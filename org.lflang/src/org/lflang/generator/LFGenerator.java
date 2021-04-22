package org.lflang.generator;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.Target;
import org.lflang.lf.TargetDecl

/**
 * Generates code from your model files on save.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
public class LFGenerator extends AbstractGenerator {

    // Indicator of whether generator errors occurred.
    protected boolean generatorErrorsOccurred = false;
    
     public void doGenerate(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {
        // Determine which target is desired.
        GeneratorBase generator;

        Target t = Target.forName(
            resource.getAllContents().filter(TargetDecl).head.name);
            
        switch (t) {
            case C: {
                generator = new CGenerator();
            }
            case CCPP: {
                generator = new CCppGenerator();
            }
            case CPP: {
                generator = new CppGenerator();
            }
            case TS: {
                generator = new TypeScriptGenerator();
            }
            case Python: {
                generator = new PythonGenerator();
            }
        }
        
        generator.doGenerate(resource, fsa, context);
        generatorErrorsOccurred = generator.errorsOccurred();
    }

    /**
     * Return true if errors occurred in the last call to doGenerate().
     * @return 
     * @return True if errors occurred.
     */
    public boolean errorsOccurred() {
        return generatorErrorsOccurred;
    }

}