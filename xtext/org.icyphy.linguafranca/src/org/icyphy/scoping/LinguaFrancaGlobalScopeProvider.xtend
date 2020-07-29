/* Custom global scope provider for Lingua Franca. */

/*************
 * Copyright (c) 2019, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
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

/**
 * Global scope provider that limits access to only those files that were
 * explicitly imported.
 * 
 * Adapted from from Xtext manual, Chapter 8.7.
 * @see https://www.eclipse.org/Xtext/documentation/2.6.0/Xtext%20Documentation.pdf
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class LinguaFrancaGlobalScopeProvider extends ImportUriGlobalScopeProvider {

    /**
     * Splitter used to process user-data annotations of Model nodes.
     */
    static final Splitter SPLITTER = Splitter.on(
        LinguaFrancaResourceDescriptionStrategy.DELIMITER);

    @Inject
    IResourceDescription.Manager descriptionManager;

    @Inject
    IResourceScopeCache cache;

    /**
     * Return the set of URI objects pointing to the resources that must be
     * included for compilation.
     */
    override protected getImportedUris(Resource resource) {
        return cache.get(LinguaFrancaGlobalScopeProvider.getSimpleName(),
            resource, new Provider<LinkedHashSet<URI>>() {
                /**
                 * Collect unique URIs in case the cache is not populated yet.
                 */
                override get() {
                    val uniqueImportURIs = collectImportUris(resource,
                        new LinkedHashSet<URI>(5))

                    val uriIter = uniqueImportURIs.iterator()
                    while (uriIter.hasNext()) {
                        if (!EcoreUtil2.isValidUri(resource, uriIter.next()))
                            uriIter.remove()
                    }
                    return uniqueImportURIs
                }

                /**
                 * Helper method to recursively collect unique URIs.
                 */
                def LinkedHashSet<URI> collectImportUris(Resource resource,
                    LinkedHashSet<URI> uniqueImportURIs) {
                    for (imported : getImportedResources(resource,
                        uniqueImportURIs)) {
                        collectImportUris(imported, uniqueImportURIs)
                    }
                    return uniqueImportURIs
                }
            });
    }

    def getImportedResources(Resource resource) {
        // FIXME: Use cache like getImportedUris()
        return getImportedResources(resource, null)
    }

    /**
     * Resolve a resource identifier.
     * 
     * @param uriStr resource identifier to resolve.
     * @param resource resource to (initially) resolve it relative to.
     */
    protected def URI resolve(String uriStr, Resource resource) {
        var uriObj = URI.createURI(uriStr)
        val uriExtension = uriObj?.fileExtension
        if (uriExtension !== null && uriExtension.equalsIgnoreCase('lf')) {
            uriObj = uriObj.resolve(resource.URI)
            // FIXME: If this doesn't work, try other things:
            // (1) Look for a .project file up the file structure and try to
            // resolve relative to the directory in which it is found.
            // (2) Look for package description files try to resolve relative
            // to the paths it includes.
            return uriObj
        }
    }

    /**
     * Return the resources imported by a given resource, excluding those
     * already discovered and therefore are present in the given set of
     * import URIs.
     * 
     * @param resource The resource to analyze.
     * @param uniqueImportURIs The set of discovered import URIs 
     */
    protected def getImportedResources(Resource resource,
        LinkedHashSet<URI> uniqueImportURIs) {
        val resourceDescription = descriptionManager.
            getResourceDescription(resource)
        val models = resourceDescription.getExportedObjectsByType(
            LinguaFrancaPackage.Literals.MODEL)
        val resources = new LinkedHashSet<Resource>()
        models.forEach [
            val userData = getUserData(
                LinguaFrancaResourceDescriptionStrategy.INCLUDES)
            if (userData !== null) {
                SPLITTER.split(userData).forEach [ uri |
                    // Attempt to resolve the URI
                    var includedUri = uri.resolve(resource)
                    if (includedUri !== null) {
                        if (uniqueImportURIs === null ||
                            uniqueImportURIs.add(includedUri)) {
                            resources.add(
                                resource.getResourceSet().getResource(
                                    includedUri, true))
                        }
                    }
                ]
            }
        ]
        return resources
    }
}
