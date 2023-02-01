/*************
* Copyright (c) 2020, Kiel University.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.ui.actions;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IFolder;
import org.eclipse.core.resources.IMarker;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.NullProgressMonitor;
import org.eclipse.core.runtime.Status;
import org.eclipse.core.runtime.jobs.Job;
import org.eclipse.emf.common.util.URI;
import org.eclipse.jface.text.ITextSelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.handlers.HandlerUtil;
import org.eclipse.ui.statushandlers.StatusManager;
import org.eclipse.xtext.builder.EclipseResourceFileSystemAccess2;
import org.eclipse.xtext.builder.MonitorBasedCancelIndicator;
import org.eclipse.xtext.generator.GeneratorContext;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.IOutputConfigurationProvider;
import org.eclipse.xtext.generator.OutputConfiguration;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.ui.editor.XtextEditor;
import org.eclipse.xtext.ui.editor.utils.EditorUtils;
import org.eclipse.xtext.ui.editor.validation.MarkerCreator;
import org.eclipse.xtext.ui.editor.validation.MarkerIssueProcessor;
import org.eclipse.xtext.ui.validation.MarkerTypeProvider;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.lflang.ui.LFUiModuleImpl;

import com.google.inject.Inject;
import com.google.inject.Injector;

/**
 * Action handler for invoking compilation.
 * 
 * @author Alexander Schulz-Rosengarten
 */
class CompileActionHandler extends AbstractHandler {

    // -- UI Constants --
    private static final String LF_EXTENSION = "lf";
    private static final String JOB_LABEL = "Lingua Franca Compilation";
    private static final String VALIDATE_LABEL = "Validating %s";
    private static final String VALIDATE_ERROR = "Unexpected exception while validating %s: %s";
    private static final String COMPILE_LABEL = "Compiling %s";
    private static final String COMPILE_ERROR = "Unexpected exception while compiling %s: %s";
    private static final String REFRESH_LABEL = "Refreshing resources";
    
    // -- Xtext classes of the LF language, provided by an Xtext injector.

    @Inject
    private GeneratorDelegate generator;

    @Inject
    private IResourceValidator validator;

    @Inject
    private LFUiModuleImpl.LinguaFrancaAutoEdit config;

    @Inject
    private MarkerCreator markerCreator;

    @Inject
    private MarkerTypeProvider markerTypeProvider;

    @Inject
    private Injector injector;

    /**
     * Internal class for an LF file, possibly open in an editor.
     */
    private class LFFile {
        public final XtextResource resource;
        public final IFile file;
        public final IProject project;
        public final XtextEditor editor;

        LFFile(final XtextResource resource, final IFile file, final IProject project, final XtextEditor editor) {
            this.resource = resource;
            this.file = file;
            this.project = project;
            this.editor = editor;
        }
    }

    @Override
    public Object execute(final ExecutionEvent event) throws ExecutionException {
        var lfFiles = new LinkedList<LFFile>();
        var selection = HandlerUtil.getActiveWorkbenchWindow(event).getActivePage().getSelection();

        // Collect selected LF files
        if (selection instanceof ITextSelection) { // Invoked from context menu of editor
            var xtextEditor = EditorUtils.getActiveXtextEditor(event);
            if (xtextEditor != null) {
                var xtextDocument = xtextEditor.getDocument();
                var resource = xtextEditor.getResource();
                if (xtextDocument != null) {
                    // Save editor
                    if (xtextEditor.isDirty()) {
                        xtextEditor.doSave(new NullProgressMonitor());
                    }
                    if (resource != null) {
                        // Load the resource of this editor based on its associated file (collect).
                        // This does not retrieve the model resource from the editor directly but creates a new one. 
                        // => workaround for issue #746.
                        lfFiles.addAll(collect((IFile) resource));
                    }
                    
                }
            }
        } else if (selection instanceof IStructuredSelection) { // Invoked from context menu in project explorer
            // Collect all LF files recursively 
            for (var element : ((IStructuredSelection) selection)) {
                lfFiles.addAll(collect((IResource) element));
            }
        }

        if (!lfFiles.isEmpty()) {
            config.configureConsole(); // Ensure active stdout forwarding

            new Job(JOB_LABEL) { // Create new background job for compilation
                @Override
                protected IStatus run(IProgressMonitor monitor) {
                    monitor.beginTask(JOB_LABEL, lfFiles.size() * 2);

                    for (var lfFile : lfFiles) {
                        // Clear all error markers
                        try {
                            lfFile.file.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE);
                        } catch (CoreException e) {
                            StatusManager.getManager().handle(
                                    new Status(Status.ERROR, "org.lflang.ui", "Could not delete error markers", e), StatusManager.LOG);
                        }

                        try {
                            // Validate
                            monitor.subTask(String.format(VALIDATE_LABEL, lfFile.file.getProjectRelativePath().toString()));

                            var issues = validator.validate(lfFile.resource, CheckMode.ALL, new CancelIndicator() {
                                @Override
                                public boolean isCanceled() {
                                    return monitor.isCanceled();
                                }
                            });

                            // Create validation markers
                            var issueProcessor = new MarkerIssueProcessor(lfFile.file,
                                    lfFile.editor != null ? lfFile.editor.getInternalSourceViewer().getAnnotationModel() : null,
                                    markerCreator, markerTypeProvider);
                            issueProcessor.processIssues(issues, monitor.slice(1));

                            if (monitor.isCanceled()) {
                                return Status.CANCEL_STATUS;
                            }
                        } catch (Exception e) {
                            StatusManager.getManager().handle(new Status(Status.ERROR, "org.lflang.ui",
                                    String.format(VALIDATE_ERROR, lfFile.file.getProjectRelativePath().toString(), e.getMessage()), e),
                                    StatusManager.LOG);
                        }

                        try {
                            // Compile
                            monitor.subTask(String.format(COMPILE_LABEL, lfFile.file.getProjectRelativePath().toString()));

                            // Create parameters for generator invocation that are usually provided by xtext but need to (artificially) created here.
                            var fsa = injector.getInstance(EclipseResourceFileSystemAccess2.class);
                            fsa.setContext(lfFile.project);
                            fsa.setMonitor(monitor.slice(0));
                            var outputConfigurationMap = new HashMap<String, OutputConfiguration>();
                            for (var config : injector.getInstance(IOutputConfigurationProvider.class).getOutputConfigurations()) {
                                outputConfigurationMap.put(config.getName(), config);
                            }
                            fsa.setOutputConfigurations(outputConfigurationMap);
                            var cancelIndicator = new MonitorBasedCancelIndicator(monitor.slice(0));
                            var generatorContext = new GeneratorContext();
                            generatorContext.setCancelIndicator(cancelIndicator);

                            // Start code generation and compilation
                            generator.generate(lfFile.resource, fsa, generatorContext);

                            monitor.worked(1);
                            if (monitor.isCanceled()) {
                                return Status.CANCEL_STATUS;
                            }
                        } catch (Exception e) {
                            StatusManager.getManager().handle(new Status(Status.ERROR, "org.lflang.ui",
                                    String.format(COMPILE_ERROR, lfFile.file.getProjectRelativePath().toString(), e.getMessage()), e),
                                    StatusManager.SHOW + StatusManager.LOG);
                        }
                    }

                    // Refresh projects to reveal new files
                    monitor.subTask(REFRESH_LABEL);
                    var projects = lfFiles.stream().map(it -> it.project).collect(Collectors.toSet());
                    for (var project : projects) {
                        try {
                            project.refreshLocal(IResource.DEPTH_INFINITE, monitor);
                        } catch (CoreException e) {
                            // ignore
                        }
                    }

                    return Status.OK_STATUS;
                }
            }.schedule(); // Start job
        }
        return this;
    }

    /**
     * Collects all LF file recursively for a given resource.
     */
    protected List<LFFile> collect(final IResource file) {
        try {
            if (file instanceof IFile) {
                return collect((IFile) file);
            } else if (file instanceof IFolder) {
                return collect((IFolder) file);
            } else if (file instanceof IProject) {
                return collect((IProject) file);
            } else {
                return Collections.emptyList();
            }
        } catch (Exception e) {
            return Collections.emptyList();
        }
    }
    
    /**
     * Collects all LF file recursively for a given project.
     */
    protected List<LFFile> collect(final IProject project) throws CoreException {
        var list = new LinkedList<LFFile>();
        for (var member : project.members()) {
            list.addAll(collect(member));
        }
        return list;
    }
    
    /**
     * Collects all LF file recursively for a given folder.
     */
    protected List<LFFile> collect(final IFolder folder) throws CoreException {
        var list = new LinkedList<LFFile>();
        for (var member : folder.members()) {
            list.addAll(collect(member));
        }
        return list;
    }

    /**
     * Returns the LF file if it is one.
     */
    protected List<LFFile> collect(final IFile file) {
        var list = new LinkedList<LFFile>();
        if (((file.exists() && (file.getFileExtension() != null)) && file.getFileExtension().endsWith(CompileActionHandler.LF_EXTENSION))) {
            // Create eResource
            var uri = URI.createPlatformResourceURI(file.getFullPath().toString(), true);
            var resourceSet = injector.getInstance(XtextResourceSet.class);
            var resource = (XtextResource) resourceSet.getResource(uri, true);
            
            list.add(new LFFile(resource, file, file.getProject(), getEditor(file)));
        }
        return list;
    }

    /**
     * Returns the open editor for the given file, if any is open.
     */
    public XtextEditor getEditor(final IFile file) {
        var workbench = PlatformUI.getWorkbench();
        if (workbench != null) {
            var workbenchWindow = workbench.getActiveWorkbenchWindow();
            if (workbenchWindow != null) {
                var page = workbenchWindow.getActivePage();
                if (page != null) {
                    for (var editorRef : page.getEditorReferences()) {
                        var editor = editorRef.getEditor(false);
                        if ((editor instanceof XtextEditor)) {
                            if (file.equals(((XtextEditor) editor).getResource())) {
                                return ((XtextEditor) editor);
                            }
                        }
                    }
                }
            }
        }
        return null;
    }
}
