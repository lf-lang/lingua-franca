package org.lflang;

import org.eclipse.emf.ecore.resource.ResourceSet;

import com.google.inject.Inject;
import com.google.inject.Provider;

/**
 * Class that provides access to a resource set.
 * 
 * @author Marten Lohstroh
 *
 */
public class LFResourceProvider {
    /**
     * Injected resource set provider.
     */
    @Inject
    private Provider<ResourceSet> resourceSetProvider;
    
    /**
     * Return a resource set obtained from the injected provider.
     * @return
     */
    public ResourceSet getResourceSet() {
        return this.resourceSetProvider.get();
    }
}
