/**
 * Copyright (c) 2015, 2020 itemis AG (http://www.itemis.eu) and others.
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 */
package org.lflang.ui;

import java.util.Objects;

import org.apache.log4j.Logger;
import org.eclipse.core.resources.IMarker;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.Path;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IShouldGenerate;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.workspace.IProjectConfig;
import org.eclipse.xtext.workspace.ProjectConfigAdapter;
import org.lflang.generator.EclipseErrorReporter;

import com.google.inject.Singleton;

/**
 * The class checks if the resource does not contain any error markers, and is contained in an Eclipse project. It returns true if both the
 * conditions are met.
 * 
 * Original: org.eclipse.xtext.ui.generator.EclipseBasedShouldGenerate
 * 
 * @author Sven Efftinge - Initial contribution and API
 * @author Alexander Schulz-Rosengarten - adjustment for LF
 */
@Singleton
public class EclipseBasedShouldGenerateLF implements IShouldGenerate {

    private static final Logger LOG = Logger.getLogger(EclipseBasedShouldGenerateLF.class);

    @Override
    public boolean shouldGenerate(Resource resource, CancelIndicator cancelIndicator) {
        URI uri = resource.getURI();
        if (uri == null || !uri.isPlatformResource()) {
            return false;
        }

        IResource member = ResourcesPlugin.getWorkspace().getRoot().findMember(new Path(uri.toPlatformString(true)));
        if (member != null && member.getType() == IResource.FILE) {
            ProjectConfigAdapter projectConfigAdapter = ProjectConfigAdapter.findInEmfObject(resource.getResourceSet());
            if (projectConfigAdapter != null) {
                IProjectConfig projectConfig = projectConfigAdapter.getProjectConfig();
                if (projectConfig != null && Objects.equals(member.getProject().getName(), projectConfig.getName())) {
                    try {
                        for (IMarker marker : member.findMarkers(null, true, IResource.DEPTH_INFINITE)) {
                            // Only consider markers not created by LF code generation
                            if (marker.getAttribute(IMarker.SEVERITY, IMarker.SEVERITY_INFO) == IMarker.SEVERITY_ERROR 
                                    && !marker.getAttribute(EclipseErrorReporter.class.getName(), false)) {
                                return false; // There is an error that is not reported by a previous compile run, hence, do not build. 
                            }
                        }
                        return true; // build in the absence of errors
                    } catch (CoreException e) {
                        LOG.error("The resource " + member.getName() + " does not exist", e);
                        return false;
                    }
                }
            }
        }
        return false;
    }
}
