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
package org.lflang.scoping

import com.google.common.base.Splitter
import com.google.inject.Inject
import java.util.LinkedHashSet
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.EcoreUtil2
import org.eclipse.xtext.resource.IResourceDescription
import org.eclipse.xtext.scoping.impl.ImportUriGlobalScopeProvider
import org.eclipse.xtext.util.IResourceScopeCache
import org.lflang.lf.LfPackage
import org.lflang.LFResourceDescriptionStrategy

/**
 * Global scope provider that limits access to only those files that were
 * explicitly imported.
 *
 * Adapted from from Xtext manual, Chapter 8.7.
 * @see "https://www.eclipse.org/Xtext/documentation/2.6.0/Xtext%20Documentation.pdf"
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 */
class LFGlobalScopeProvider : ImportUriGlobalScopeProvider() {

    @Inject
    lateinit var descriptionManager: IResourceDescription.Manager

    @Inject
    private lateinit var _cache: IResourceScopeCache


    /**
     * Return the set of URI objects pointing to the resources that must be
     * included for compilation.
     */
    override fun getImportedUris(resource: Resource): LinkedHashSet<URI> =
        _cache.get(IMPORTED_URIS, resource) {
            /** Helper method to recursively collect unique URIs. */
            fun collectImportUris(resource: Resource, uniqueImportURIs: MutableSet<URI>) {
                for (imported in getImportedResources(resource, uniqueImportURIs)) {
                    collectImportUris(imported, uniqueImportURIs)
                }
            }

            val uniqueImportURIs = LinkedHashSet<URI>().also {
                collectImportUris(resource, it)
            }

            uniqueImportURIs.removeIf {
                !EcoreUtil2.isValidUri(resource, it)
            }
            uniqueImportURIs
        }

    /**
     * Return the resources imported by the given resource.
     * @param resource The resource to return the imported resources of.
     */
    fun getImportedResources(resource: Resource): Unit =
        _cache.get(IMPORTED_RESOURCES, resource) {
            getImportedResources(resource, mutableSetOf())
        }


    /**
     * Resolve a resource identifier.
     *
     * @receiver resource identifier to resolve.
     * @param resource resource to (initially) resolve it relative to.
     */
    private fun String.resolve(resource: Resource): URI? {
        var uriObj = URI.createURI(this)
        val uriExtension = uriObj?.fileExtension()
        if (uriExtension?.equals("lf", ignoreCase = true) == true) {
            uriObj = uriObj.resolve(resource.uri)
            // FIXME: If this doesn't work, try other things:
            // (1) Look for a .project file up the file structure and try to
            // resolve relative to the directory in which it is found.
            // (2) Look for package description files try to resolve relative
            // to the paths it includes.
            // FIXME: potentially use a cache here to speed things up.
            // See OnChangeEvictingCache
            return uriObj
        }
        return null
    }

    /**
     * Return the resources imported by a given resource, excluding those
     * already discovered and therefore are present in the given set of
     * import URIs.
     *
     * @param resource The resource to analyze.
     * @param uniqueImportURIs The set of discovered import URIs
     */
    private fun getImportedResources(resource: Resource, uniqueImportURIs: MutableSet<URI>): Set<Resource> {
        val resourceDescription = descriptionManager.getResourceDescription(resource)
        val result = LinkedHashSet<Resource>()
        for (model in resourceDescription.getExportedObjectsByType(LfPackage.Literals.MODEL)) {
            val userData = model.getUserData(LFResourceDescriptionStrategy.INCLUDES) ?: continue
            for (uri in SPLITTER.split(userData)) {
                // Attempt to resolve the URI
                val includedUri = uri.resolve(resource) ?: continue
                if (uniqueImportURIs.add(includedUri)) {
                    try {
                        val loaded = resource.resourceSet.getResource(includedUri, true)
                        result.add(loaded)
                    } catch (e: Exception) {
                        System.err.println("Unable to import $includedUri")
                    }
                }
            }
        }
        return result
    }

    companion object {

        /** Splitter used to process user-data annotations of Model nodes. */
        val SPLITTER: Splitter = Splitter.on(LFResourceDescriptionStrategy.DELIMITER)
        const val IMPORTED_URIS = "IMPORTED_URIS"
        const val IMPORTED_RESOURCES = "IMPORTED_RESOURCES"

    }
}
