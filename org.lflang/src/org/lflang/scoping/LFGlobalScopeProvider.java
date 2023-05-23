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

package org.lflang.scoping;

import com.google.common.base.Splitter;
import com.google.inject.Inject;
import com.google.inject.Provider;
import java.util.LinkedHashSet;
import java.util.Set;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.EcoreUtil2;
import org.eclipse.xtext.resource.IResourceDescription;
import org.eclipse.xtext.scoping.impl.ImportUriGlobalScopeProvider;
import org.eclipse.xtext.util.IResourceScopeCache;
import org.lflang.LFResourceDescriptionStrategy;
import org.lflang.lf.LfPackage;

/**
 * Global scope provider that limits access to only those files that were explicitly imported.
 *
 * <p>Adapted from from Xtext manual, Chapter 8.7.
 *
 * @author Marten Lohstroh
 * @see <a href="https://www.eclipse.org/Xtext/documentation/2.6.0/Xtext%20Documentation.pdf">xtext
 *     doc</a>
 */
public class LFGlobalScopeProvider extends ImportUriGlobalScopeProvider {

  /** Splitter used to process user-data annotations of Model nodes. */
  static final Splitter SPLITTER = Splitter.on(LFResourceDescriptionStrategy.DELIMITER);

  static final String IMPORTED_URIS = "IMPORTED_URIS";

  static final String IMPORTED_RESOURCES = "IMPORTED_RESOURCES";

  @Inject private IResourceDescription.Manager descriptionManager;

  @Inject private IResourceScopeCache cache;

  /**
   * Return the set of URI objects pointing to the resources that must be included for compilation.
   */
  @Override
  protected LinkedHashSet<URI> getImportedUris(Resource resource) {
    return cache.get(
        IMPORTED_URIS,
        resource,
        new Provider<LinkedHashSet<URI>>() {
          /** Collect unique URIs in case the cache is not populated yet. */
          @Override
          public LinkedHashSet<URI> get() {
            var uniqueImportURIs = new LinkedHashSet<URI>(5);
            collectImportUris(resource, uniqueImportURIs);
            uniqueImportURIs.removeIf(uri -> !EcoreUtil2.isValidUri(resource, uri));
            return uniqueImportURIs;
          }

          /** Helper method to recursively collect unique URIs. */
          void collectImportUris(Resource resource, LinkedHashSet<URI> uniqueImportURIs) {
            for (var imported : getImportedResources(resource, uniqueImportURIs)) {
              collectImportUris(imported, uniqueImportURIs);
            }
          }
        });
  }

  /** Return the resources imported by the given resource. */
  public Set<Resource> getImportedResources(Resource resource) {
    return cache.get(IMPORTED_RESOURCES, resource, () -> getImportedResources(resource, null));
  }

  /**
   * Resolve a resource identifier.
   *
   * @param uriStr resource identifier to resolve.
   * @param resource resource to (initially) resolve it relative to.
   */
  protected URI resolve(String uriStr, Resource resource) {
    var uriObj = URI.createURI(uriStr);
    if (uriObj != null && "lf".equalsIgnoreCase(uriObj.fileExtension())) {
      // FIXME: If this doesn't work, try other things:
      // (1) Look for a .project file up the file structure and try to
      // resolve relative to the directory in which it is found.
      // (2) Look for package description files try to resolve relative
      // to the paths it includes.
      // FIXME: potentially use a cache here to speed things up.
      // See OnChangeEvictingCache
      return uriObj.resolve(resource.getURI());
    }
    return null;
  }

  /**
   * Return the resources imported by a given resource, excluding those already discovered and
   * therefore are present in the given set of import URIs.
   *
   * @param resource The resource to analyze.
   * @param uniqueImportURIs The set of discovered import URIs
   */
  protected Set<Resource> getImportedResources(
      Resource resource, LinkedHashSet<URI> uniqueImportURIs) {
    var resourceDescription = descriptionManager.getResourceDescription(resource);
    var models = resourceDescription.getExportedObjectsByType(LfPackage.Literals.MODEL);
    var resources = new LinkedHashSet<Resource>();
    for (var model : models) {
      var userData = model.getUserData(LFResourceDescriptionStrategy.INCLUDES);
      if (userData != null) {
        for (String uri : SPLITTER.split(userData)) { // Attempt to resolve the URI
          var includedUri = this.resolve(uri, resource);
          if (includedUri != null) {
            try {
              if (uniqueImportURIs == null || uniqueImportURIs.add(includedUri)) {
                resources.add(resource.getResourceSet().getResource(includedUri, true));
              }
            } catch (RuntimeException e) {
              System.err.println("Unable to import " + includedUri + ": " + e.getMessage());
            }
          }
        }
      }
    }
    return resources;
  }
}
