/* Custom global scope provider for Lingua Franca. */

package org.icyphy.scoping

import com.google.common.base.Splitter
import com.google.inject.Inject
import com.google.inject.Provider
import java.util.LinkedHashSet
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.EcoreUtil2
import org.eclipse.xtext.resource.IResourceDescription
import org.eclipse.xtext.util.IResourceScopeCache
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.LinguaFrancaResourceDescriptionStrategy
import org.eclipse.xtext.scoping.impl.AbstractGlobalScopeProvider
import org.eclipse.xtext.resource.IResourceDescriptions
import com.google.common.collect.Iterables
import org.eclipse.xtext.scoping.impl.DelegatingEventSource
import org.eclipse.xtext.util.OnChangeEvictingCache
import java.util.List
import com.google.common.collect.Lists
import org.eclipse.xtext.resource.IContainer
import org.eclipse.xtext.resource.IResourceDescription.Event.Source
import org.eclipse.xtext.scoping.impl.DefaultGlobalScopeProvider

/** Global scope provider designed limit global scope to 
 *  only those files that were explicitly imported.
 */
class LinguaFrancaGlobalScopeProvider extends DefaultGlobalScopeProvider {
    // NOTE: Adapted from example provided by Itemis.
    // https://blogs.itemis.com/en/in-five-minutes-to-transitive-imports-within-a-dsl-with-xtext 
    
    static final Splitter SPLITTER = Splitter.on(LinguaFrancaResourceDescriptionStrategy.DELIMITER);

    
    @Inject
	IContainer.Manager containerManager;

	@Inject
	IResourceDescription.Manager descriptionManager;
    
    override List<IContainer> getVisibleContainers(Resource resource) {
		val description = descriptionManager.getResourceDescription(resource);
		val resourceDescriptions = getResourceDescriptions(resource);
		val cacheKey = getCacheKey("VisibleContainers", resource.getResourceSet());
		val cache = new OnChangeEvictingCache().getOrCreate(resource);
		var result = cache.get(cacheKey as Object);
		if (result == null) {
			result = containerManager.getVisibleContainers(description, resourceDescriptions);
			// SZ: I'ld like this dependency to be moved to the implementation of the
			// container manager, but it is not aware of a CacheAdapter
			if (resourceDescriptions instanceof IResourceDescription.Event.Source) {
				val eventSource = resourceDescriptions as Source;
				var delegatingEventSource = new DelegatingEventSource(eventSource);
				delegatingEventSource.addListeners(Lists.newArrayList(Iterables.filter(result, IResourceDescription.Event.Listener)));
				delegatingEventSource.initialize();
				cache.addCacheListener(delegatingEventSource);
			}
			cache.set(cacheKey, result);
		}
		return result;
	}
}