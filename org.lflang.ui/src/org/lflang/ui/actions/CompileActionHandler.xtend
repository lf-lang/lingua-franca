/** 
 * Copyright (c) 2021, Kiel University.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
 */
package org.lflang.ui.actions

import com.google.inject.Inject
import com.google.inject.Injector
import java.util.List
import org.eclipse.core.commands.AbstractHandler
import org.eclipse.core.commands.ExecutionEvent
import org.eclipse.core.commands.ExecutionException
import org.eclipse.core.resources.IFile
import org.eclipse.core.resources.IFolder
import org.eclipse.core.resources.IProject
import org.eclipse.core.resources.IResource
import org.eclipse.core.runtime.IProgressMonitor
import org.eclipse.core.runtime.Status
import org.eclipse.core.runtime.jobs.Job
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.jface.text.ITextSelection
import org.eclipse.jface.viewers.IStructuredSelection
import org.eclipse.ui.handlers.HandlerUtil
import org.eclipse.ui.statushandlers.StatusManager
import org.eclipse.xtend.lib.annotations.Data
import org.eclipse.xtext.builder.EclipseResourceFileSystemAccess2
import org.eclipse.xtext.builder.MonitorBasedCancelIndicator
import org.eclipse.xtext.generator.GeneratorContext
import org.eclipse.xtext.generator.GeneratorDelegate
import org.eclipse.xtext.generator.IOutputConfigurationProvider
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.resource.XtextResourceSet
import org.eclipse.xtext.ui.editor.XtextEditor
import org.eclipse.xtext.ui.editor.utils.EditorUtils
import org.eclipse.xtext.ui.editor.validation.MarkerCreator
import org.eclipse.xtext.ui.editor.validation.MarkerIssueProcessor
import org.eclipse.xtext.ui.validation.MarkerTypeProvider
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.util.concurrent.IUnitOfWork
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.lflang.ui.LFUiModuleImpl.LinguaFrancaAutoEdit

import static extension java.lang.String.*
import org.eclipse.ui.PlatformUI
import org.eclipse.core.resources.IMarker

/** 
 * Action handler for invoking compilation.
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
package class CompileActionHandler extends AbstractHandler {
    
    static val LF_EXTENSION = "lf"
    static val JOB_LABEL = "Lingua Franca Compilation"
    static val VALIDATE_LABEL = "Validating %s"
    static val VALIDATE_ERROR = "Unexpected exception while validating %s: %s"
    static val COMPILE_LABEL = "Compiling %s"
    static val COMPILE_ERROR = "Unexpected exception while compiling %s: %s"
    static val REFRESH_LABEL = "Refreshing resources"
    
    @Inject GeneratorDelegate generator
    @Inject IResourceValidator validator
    @Inject LinguaFrancaAutoEdit config
    @Inject MarkerCreator markerCreator;
    @Inject MarkerTypeProvider markerTypeProvider;
    @Inject Injector injector

    override Object execute(ExecutionEvent event) throws ExecutionException {        
        val lfFiles = <LFFile>newLinkedList
        val selection = HandlerUtil::getActiveWorkbenchWindow(event).getActivePage().getSelection()
        
        // Collect selected LF files
        if (selection instanceof ITextSelection) {
            var XtextEditor xtextEditor = EditorUtils::getActiveXtextEditor(event)
            if (xtextEditor !== null) {
                val xtextDocument = xtextEditor.getDocument()
                val work = new IUnitOfWork<XtextResource, XtextResource>() {
                    
                    override exec(XtextResource state) throws Exception {
                        return state
                    }
                    
                }
                val resource = xtextDocument?.readOnly(work)
                val file = xtextEditor.resource as IFile
                lfFiles += new LFFile(resource, file, file.project, xtextEditor)
            }
        } else if (selection instanceof IStructuredSelection) {
            for (element : selection.iterator.toIterable) {
                lfFiles.addAll(collect(element as IResource))
            }
        }
        
        if (!lfFiles.empty) {
            config.configureConsole() // Ensure active stdout forwarding 
            
            new Job (JOB_LABEL) {
                override protected run(IProgressMonitor monitor) {
                    monitor.beginTask(JOB_LABEL, lfFiles.size * 2)
                    
                    for (lfFile : lfFiles) {
                        // Clear all error markers
                        lfFile.file.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE)
                        
                        // Save editor
                        if (lfFile.editor !== null && lfFile.editor.isDirty) {
                            lfFile.editor.doSave(monitor)
                        }
                        
                        try {
                            // Validate
                            monitor.subTask(VALIDATE_LABEL.format(lfFile.file.projectRelativePath.toString))
                            
                            val issues = validator.validate(lfFile.resource, CheckMode.ALL, new CancelIndicator() {
                                override isCanceled() {
                                    return monitor.isCanceled();
                                }
                            })
                            
                            // Create validation marker
                            val issueProcessor = new MarkerIssueProcessor(lfFile.file, 
                                lfFile.editor !== null ? lfFile.editor.getInternalSourceViewer().getAnnotationModel() : null, markerCreator, markerTypeProvider);
                            issueProcessor.processIssues(issues, monitor)
                            
                            monitor.worked(1)
                            if (monitor.canceled) return Status.CANCEL_STATUS
                        } catch (Exception e) {
                            StatusManager.getManager().handle(
                                new Status(Status.ERROR, "org.lflang.ui", 
                                    VALIDATE_ERROR.format(lfFile.file.projectRelativePath.toString, e.message), e), 
                                    StatusManager.LOG)
                        }
                        
                        try {
                            // Compile
                            monitor.subTask(COMPILE_LABEL.format(lfFile.file.projectRelativePath.toString))
                        
                            val fsa = injector.getInstance(EclipseResourceFileSystemAccess2)
                            fsa.context = lfFile.project
                            fsa.monitor = monitor
                            fsa.outputConfigurations = injector.getInstance(IOutputConfigurationProvider).outputConfigurations.toMap[it.name]
                            var MonitorBasedCancelIndicator cancelIndicator = new MonitorBasedCancelIndicator(monitor)
                            var GeneratorContext generatorContext = new GeneratorContext()
                            generatorContext.setCancelIndicator(cancelIndicator)
                            
                            generator.generate(lfFile.resource, fsa, generatorContext)
                            
                            monitor.worked(1)
                            if (monitor.canceled) return Status.CANCEL_STATUS
                        } catch (Exception e) {
                            StatusManager.getManager().handle(
                                new Status(Status.ERROR, "org.lflang.ui",
                                    COMPILE_ERROR.format(lfFile.file.projectRelativePath.toString, e.message), e),
                                    StatusManager.LOG)
                        }
                    }
                    
                    // Refresh projects to reveal new files
                    monitor.subTask(REFRESH_LABEL)
                    for (project : lfFiles.map[project].toSet) {
                        project.refreshLocal(IResource.DEPTH_INFINITE, monitor)
                    }
                    
                    return Status::OK_STATUS
                }
            }.schedule()
        }
        return this
    }
    
    dispatch def List<LFFile> collect(IProject project) {
        val list = newLinkedList
        for (member : project.members) {
            list.addAll(collect(member))
        }
        return list
    }

    dispatch def List<LFFile> collect(IFolder folder) {
        val list = newLinkedList
        for (member : folder.members) {
            list.addAll(collect(member))
        }
        return list
    }

    dispatch def List<LFFile> collect(IFile file) {
        if (file.exists && file.fileExtension !== null && file.fileExtension.endsWith(LF_EXTENSION)) {
            val uri = URI.createPlatformResourceURI(file.fullPath.toString, true)
            val resourceSet = injector.getInstance(XtextResourceSet)
            val resource = resourceSet.getResource(uri, true) as XtextResource
            
            return newLinkedList(new LFFile(resource, file, file.project, file.editor))
        }
        return emptyList
    }
    
    def XtextEditor getEditor(IFile file) {
        val workbench = PlatformUI.getWorkbench()
        val workbenchWindow = workbench?.getActiveWorkbenchWindow()
        val activePage = workbenchWindow?.getActivePage()
        for (editorRef : activePage?.editorReferences) {
            val editor = editorRef.getEditor(false)
            if (editor instanceof XtextEditor) {
                if (file.equals(editor.resource)) {
                    return editor
                }
            }
        }
    }
}

@Data
class LFFile {
    val XtextResource resource
    val IFile file
    val IProject project
    val XtextEditor editor
}
