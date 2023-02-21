package org.lflang.generator;

import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;

/**
 * A class that keeps metadata for discovered resources
 * during code generation and the supporting structures
 * associated with that resource.
 * 
 * @author Soroush Bateni
 */
public class LFResource {
    LFResource(Resource resource, FileConfig fileConfig, TargetConfig targetConfig) {
        this.eResource = resource; // FIXME: this is redundant because fileConfig already has the resource.
        this.fileConfig = fileConfig;
        this.targetConfig = targetConfig;
    }
    
    /**
     * Resource associated with a file either from the main .lf
     * file or one of the imported ones.
     */
    Resource eResource;
    public Resource getEResource() { return this.eResource; };
    
    /**
     * The file config associated with 'resource' that can be
     * used to discover files relative to that resource.
     */
    FileConfig fileConfig;
    public FileConfig getFileConfig() { return this.fileConfig; };
    
    /**
     * The target config read from the resource.
     */
    TargetConfig targetConfig;
    public TargetConfig getTargetConfig() { return this.targetConfig; };
}
