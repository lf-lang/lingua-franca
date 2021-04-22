package org.lflang.generator;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.AbstractGenerator;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;

/**
 * Generates code from your model files on save.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
public class LFGenerator extends AbstractGenerator {

    // Indicator of whether generator errors occurred.
    protected boolean generatorErrorsOccurred = false;
    
     public void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        generatorErrorsOccurred = LFGeneratorImpl.INSTANCE.doGenerate(resource, fsa, context);
    }

    /** Return true if errors occurred in the last call to doGenerate(). */
    public boolean errorsOccurred() {
        return generatorErrorsOccurred;
    }

}
