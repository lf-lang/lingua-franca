/* Runtime module for Lingua Franca. */
package org.icyphy

import org.eclipse.xtext.resource.IContainer
import org.eclipse.xtext.resource.IDefaultResourceDescriptionStrategy
import org.eclipse.xtext.resource.containers.IAllContainersState
import org.eclipse.xtext.resource.impl.DefaultResourceDescriptionStrategy
import org.eclipse.xtext.scoping.IGlobalScopeProvider
import org.eclipse.xtext.scoping.impl.DefaultGlobalScopeProvider
import org.eclipse.xtext.validation.INamesAreUniqueValidationHelper
import org.icyphy.scoping.LinguaFrancaContainerManager
import org.icyphy.scoping.LinguaFrancaStateManager
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper
import org.icyphy.validation.LinguaFrancaNamesAreUniqueValidationHelper

import com.google.inject.Provider;

/**
 * This class is used to register components to be used at runtime 
 * / without the Equinox extension registry.
 */
class LinguaFrancaRuntimeModule extends AbstractLinguaFrancaRuntimeModule {
	
	override Class<? extends IContainer.Manager> bindIContainer$Manager() {
        return LinguaFrancaContainerManager;
    }
    
    def Class<? extends IAllContainersState.Provider> provideIAllContainersState() {
        return LinguaFrancaStateManager;
    }
    
		
    /** Establish a binding to our custom resource description strategy. */
    def Class<? extends IDefaultResourceDescriptionStrategy> bindIDefaultResourceDescriptionStrategy() {
        DefaultResourceDescriptionStrategy
    }
    
    /** Establish a binding to our custom global scope provider. */
    override Class<? extends IGlobalScopeProvider> bindIGlobalScopeProvider() {
        DefaultGlobalScopeProvider;
    }
    
    /** Establish a binding to a helper that checks that names are unique. */
    def Class<? extends INamesAreUniqueValidationHelper> bindNamesAreUniqueValidationHelper() {
        LinguaFrancaNamesAreUniqueValidationHelper;
    }
}
