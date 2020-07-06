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
//import org.eclipse.xtext.resource.containers.IAllContainersState

class LinguaFrancaContainerManager extends StateBasedContainerManager {

    @Inject
	ProjectDescriptionBasedContainerManager delegate;
	
	//@Inject
	//LinguaFrancaStateManager.Provider stateProvider;
	
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
		val classpath = System.getenv("LF_CLASSPATH")
		if (!classpath.isNullOrEmpty) {
		    System.out.println(classpath)
            val paths = classpath.split(System.getProperty("path.separator"));
            result.addAll(getVisibleContainers(paths, resourceDescriptions))    
		}
		
		return result;
	}


}
