package org.icyphy.scoping;

import org.eclipse.xtext.resource.containers.StateBasedContainerManager;
import org.eclipse.xtext.resource.IResourceDescriptions
import org.eclipse.xtext.resource.IResourceDescription
import java.util.List
import org.eclipse.xtext.resource.containers.DescriptionAddingContainer
import org.eclipse.xtext.resource.IContainer
import java.util.Collections
import org.eclipse.xtext.resource.containers.ProjectDescriptionBasedContainerManager
import com.google.inject.Inject
import java.util.ArrayList

//import org.eclipse.xtext.resource.containers.IAllContainersState

class LinguaFrancaContainerManager extends StateBasedContainerManager {

    @Inject
	ProjectDescriptionBasedContainerManager delegate;
	
	//@Inject
	//LinguaFrancaStateManager.Provider stateProvider;
	
	/*
	 * Based on a flat resource description index, finds a list of containers that have access to a given resource.
	 * These containers hold descriptions for the resource and can be used to retrieve imported names from the resource.
	 * @param desc The description of a resource that is used to lookup the resource description index.
	 * @param resourceDescriptions The flat list of resource descriptions that is used as a database to lookup desc.
	 */
	override
	List<IContainer> getVisibleContainers(IResourceDescription desc, IResourceDescriptions resourceDescriptions) {
		if (delegate.shouldUseProjectDescriptionBasedContainers(resourceDescriptions)) {
			return delegate.getVisibleContainers(desc, resourceDescriptions);
		}
		val root = internalGetContainerHandle(desc, resourceDescriptions);
		if (root === null) {
			return Collections.emptyList();
		}
		val handles = getState(resourceDescriptions).getVisibleContainerHandles(root);
		val result = getVisibleContainers(handles, resourceDescriptions);
		if (!result.isEmpty()) {
			var first = result.get(0);
			if (!first.hasResourceDescription(desc.getURI())) {
				first = new DescriptionAddingContainer(desc, first);
				result.set(0, first);
			}
		}

		
		
		return result;
	}


}
