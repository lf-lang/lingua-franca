/* Runtime module for Lingua Franca. */
package org.icyphy

import org.eclipse.xtext.resource.IDefaultResourceDescriptionStrategy
import org.eclipse.xtext.scoping.IGlobalScopeProvider
import org.icyphy.scoping.LinguaFrancaGlobalScopeProvider
import org.eclipse.xtext.validation.INamesAreUniqueValidationHelper
import org.icyphy.validation.LinguaFrancaNamesAreUniqueValidationHelper

/**
 * This class is used to register components to be used at runtime 
 * / without the Equinox extension registry.
 */
class LinguaFrancaRuntimeModule extends AbstractLinguaFrancaRuntimeModule {
    /** Establish a binding to our custom resource description strategy. */
    def Class<? extends IDefaultResourceDescriptionStrategy> bindIDefaultResourceDescriptionStrategy() {
        LinguaFrancaResourceDescriptionStrategy
    }
    
    /** Establish a binding to our custom global scope provider. */
    override Class<? extends IGlobalScopeProvider> bindIGlobalScopeProvider() {
        LinguaFrancaGlobalScopeProvider;
    }
    
    /** Establish a binding to a helper that checks that names are unique. */
    def Class<? extends INamesAreUniqueValidationHelper> bindNamesAreUniqueValidationHelper() {
        LinguaFrancaNamesAreUniqueValidationHelper;
    }
}
