/* Custom global scope provider for Lingua Franca. */ package org.icyphy.scoping

import com.google.common.collect.Iterables
import com.google.common.collect.Lists
import com.google.inject.Inject
import java.util.List
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.resource.IContainer
import org.eclipse.xtext.resource.IResourceDescription
import org.eclipse.xtext.resource.IResourceDescription.Event.Source
import org.eclipse.xtext.scoping.impl.DefaultGlobalScopeProvider
import org.eclipse.xtext.scoping.impl.DelegatingEventSource
import org.eclipse.xtext.util.OnChangeEvictingCache

/** Global scope provider designed to limit global scope to the current
 * project directory.
 */
class LinguaFrancaGlobalScopeProvider extends DefaultGlobalScopeProvider {
    // Note: Adapted from Xtext manual.
    // See https://www.eclipse.org/Xtext/documentation/2.6.0/Xtext%20Documentation.pdf Chapter 8.7
    @Inject
    IContainer.Manager containerManager;

    @Inject
    IResourceDescription.Manager descriptionManager;

    /* Access to resources are constrained in Xtext by the container manager, which is language specific.
     * Therefore, the first step in acquiring the resource is to get its corresponding container. 
     * This function returns a list of containers that contain a specific resource.
     * @param resource
     * @return A list of visible containers that have access to the resource. 
     */
    override List<IContainer> getVisibleContainers(Resource resource) {
        // Get the contents of the resource in the form of an ISelectable that holds the imported names
        // from a given resource.
        val description = descriptionManager.getResourceDescription(resource);
        // A flat index that contains all resource descriptions
        val resourceDescriptions = getResourceDescriptions(resource);

        // Cache management for faster retrieval of visible containers
        val cacheKey = getCacheKey("VisibleContainers",
            resource.getResourceSet());
        val cache = new OnChangeEvictingCache().getOrCreate(resource);
        var result = cache.get(cacheKey as Object);
        if (result === null) {
            // If a resource's container is not in cache, actually use the container manager.
            result = containerManager.getVisibleContainers(description,
                resourceDescriptions);
            if (resourceDescriptions instanceof IResourceDescription.Event.Source) {
                val eventSource = resourceDescriptions as Source;
                var delegatingEventSource = new DelegatingEventSource(
                    eventSource);
                delegatingEventSource.addListeners(
                    Lists.newArrayList(
                        Iterables.filter(result,
                            IResourceDescription.Event.Listener)));
                delegatingEventSource.initialize();
                cache.addCacheListener(delegatingEventSource);
            }
            cache.set(cacheKey, result);
        }
        return result;
    }
}
