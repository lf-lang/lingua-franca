package org.lflang.generator;

import java.io.IOException;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.lflang.FileConfig;

public class CppFileConfig extends FileConfig {

    public CppFileConfig(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) throws IOException {
        super(resource, fsa, context);
    }

    /**
     * Clean any artifacts produced by the C++ code generator.
     * 
     * @throws IOException
     */
    public void doClean() throws IOException {
        super.doClean();
        deleteDirectory(outPath.resolve("build"));
        deleteDirectory(outPath.resolve("lib"));
        deleteDirectory(outPath.resolve("include"));
        deleteDirectory(outPath.resolve("share"));
    }

}
