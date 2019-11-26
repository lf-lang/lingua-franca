/* Custom global scope provider for Lingua Franca. */

package org.icyphy.scoping

import org.eclipse.xtext.scoping.impl.ImportUriGlobalScopeProvider

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

/** Global scope provider designed limit global scope to 
 *  only those files that were explicitly imported.
 */
class LinguaFrancaGlobalScopeProvider extends ImportUriGlobalScopeProvider {
    // NOTE: Adapted from example provided by Itemis.
    // https://blogs.itemis.com/en/in-five-minutes-to-transitive-imports-within-a-dsl-with-xtext 
    
    static final Splitter SPLITTER = Splitter.on(LinguaFrancaResourceDescriptionStrategy.DELIMITER);

    @Inject
    IResourceDescription.Manager descriptionManager;

    @Inject
    IResourceScopeCache cache;

    override protected getImportedUris(Resource resource) {
        return cache.get(LinguaFrancaGlobalScopeProvider.getSimpleName(), resource, new Provider<LinkedHashSet<URI>>() {
            override get() {
                val uniqueImportURIs = collectImportUris(resource, new LinkedHashSet<URI>(5))

                val uriIter = uniqueImportURIs.iterator()
                while(uriIter.hasNext()) {
                    if (!EcoreUtil2.isValidUri(resource, uriIter.next()))
                        uriIter.remove()
                }
                return uniqueImportURIs
            }

            def LinkedHashSet<URI> collectImportUris(Resource resource, LinkedHashSet<URI> uniqueImportURIs) {
                val resourceDescription = descriptionManager.getResourceDescription(resource)
                val models = resourceDescription.getExportedObjectsByType(LinguaFrancaPackage.Literals.MODEL)
                
                models.forEach[
                    val userData = getUserData(LinguaFrancaResourceDescriptionStrategy.INCLUDES)
                    if(userData !== null) {
                        SPLITTER.split(userData).forEach[uri |
                            var includedUri = URI.createURI(uri)
                            includedUri = includedUri.resolve(resource.URI)
                            if(uniqueImportURIs.add(includedUri)) {
                                collectImportUris(resource.getResourceSet().getResource(includedUri, true), uniqueImportURIs)
                            }
                        ]
                    }
                ]
                
                return uniqueImportURIs
            }
        });
    }
}