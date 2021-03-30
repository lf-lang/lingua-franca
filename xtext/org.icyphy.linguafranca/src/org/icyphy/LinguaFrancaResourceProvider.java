package org.icyphy;

import org.eclipse.emf.ecore.resource.ResourceSet;

import com.google.inject.Inject;
import com.google.inject.Provider;

/**
 * Class that provides access to a resource set.
 * 
 * @author Marten Lohstroh <marten@berkeley.edu>
 *
 */
public class LinguaFrancaResourceProvider {
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
