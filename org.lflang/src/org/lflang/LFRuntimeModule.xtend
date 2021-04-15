/* Runtime module for Lingua Franca. */
package org.lflang

import org.eclipse.xtext.resource.IDefaultResourceDescriptionStrategy
import org.eclipse.xtext.scoping.IGlobalScopeProvider
import org.eclipse.xtext.validation.INamesAreUniqueValidationHelper
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.validation.LFNamesAreUniqueValidationHelper

/**
 * This class is used to register components to be used at runtime 
 * / without the Equinox extension registry.
 */
class LFRuntimeModule extends AbstractLFRuntimeModule {
    /** Establish a binding to our custom resource description strategy. */
    def Class<? extends IDefaultResourceDescriptionStrategy> bindIDefaultResourceDescriptionStrategy() {
        LFResourceDescriptionStrategy
    }
    
    /** Establish a binding to our custom global scope provider. */
    override Class<? extends IGlobalScopeProvider> bindIGlobalScopeProvider() {
        LFGlobalScopeProvider;
    }
    
    /** Establish a binding to a helper that checks that names are unique. */
    def Class<? extends INamesAreUniqueValidationHelper> bindNamesAreUniqueValidationHelper() {
        LFNamesAreUniqueValidationHelper;
    }
}
