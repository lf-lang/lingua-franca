/* Runtime module for Lingua Franca. */
package org.icyphy

import org.eclipse.xtext.resource.IDefaultResourceDescriptionStrategy
import org.eclipse.xtext.scoping.IGlobalScopeProvider
import org.icyphy.scoping.LinguaFrancaGlobalScopeProvider
import org.eclipse.xtext.scoping.impl.DefaultGlobalScopeProvider
import org.eclipse.xtext.resource.impl.DefaultResourceDescriptionStrategy
import org.eclipse.xtext.resource.IContainer
import org.eclipse.xtext.resource.containers.IAllContainersState;
import org.icyphy.scoping.LinguaFrancaContainerManager
import org.icyphy.scoping.LinguaFrancaStateManager

/**
 * This class is used to register components to be used at runtime 
 * / without the Equinox extension registry.
 */
class LinguaFrancaRuntimeModule extends AbstractLinguaFrancaRuntimeModule {
	
	override Class<? extends IContainer.Manager> bindIContainer$Manager() {
        return LinguaFrancaContainerManager;
    }
    
    def Class<? extends IAllContainersState> provideIAllContainersState() {
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
}
