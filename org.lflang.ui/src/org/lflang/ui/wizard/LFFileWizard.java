package org.lflang.ui.wizard;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

import org.eclipse.core.resources.IFile;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.dialogs.WizardNewFileCreationPage;
import org.eclipse.ui.ide.IDE;
import org.eclipse.ui.internal.ide.DialogUtil;
import org.eclipse.ui.internal.wizards.newresource.ResourceMessages;
import org.eclipse.ui.plugin.AbstractUIPlugin;
import org.eclipse.ui.wizards.newresource.BasicNewResourceWizard;

public class LFFileWizard extends BasicNewResourceWizard {

    private static final String FILE_EXT = "lf";
    private static final String CONTENT = "";

    private WizardNewFileCreationPage mainPage;

    @Override
    public void addPages() {
        super.addPages();
        IStructuredSelection selection = this.getSelection();
        this.mainPage = new WizardNewFileCreationPage("newFilePage", selection) {
            @Override
            public InputStream getInitialContents() {
                return new ByteArrayInputStream(CONTENT.getBytes(StandardCharsets.UTF_8));
            }
        };
        this.mainPage.setTitle("Create Lingua Franca File");
        this.mainPage.setDescription("Create a new Lingua Franca file.");
        this.mainPage.setFileExtension(LFFileWizard.FILE_EXT);
        this.addPage(this.mainPage);
    }

    @Override
    public void init(final IWorkbench workbench, final IStructuredSelection currentSelection) {
        super.init(workbench, currentSelection);
        this.setWindowTitle("Create new Lingua Franca File");
        this.setNeedsProgressMonitor(true);
    }

    @Override
    public void initializeDefaultPageImageDescriptor() {
        this.setDefaultPageImageDescriptor(AbstractUIPlugin.imageDescriptorFromPlugin("org.lflang.ui", "icons/new-lf-file-wiz.png"));
    }

    @Override
    public boolean performFinish() {
        final IFile file = this.mainPage.createNewFile();
        if (file == null) {
            return false;
        }
        this.selectAndReveal(file);
        final IWorkbenchWindow dw = this.getWorkbench().getActiveWorkbenchWindow();
        try {
            if ((dw != null)) {
                final IWorkbenchPage page = dw.getActivePage();
                if ((page != null)) {
                    IDE.openEditor(page, file, true);
                }
            }
        } catch (PartInitException e) {
            DialogUtil.openError(dw.getShell(), ResourceMessages.FileResource_errorMessage, e.getMessage(), e);
        }
        return true;
    }

}
