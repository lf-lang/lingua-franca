package org.lflang.product;

import org.eclipse.ui.IFolderLayout;
import org.eclipse.ui.IPageLayout;
import org.eclipse.ui.IPerspectiveFactory;

/**
 * Perspective for modeling with lingua franca.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class LFPerspectiveFactory implements IPerspectiveFactory {
    
    private static final String PACKAGE_EXPLORER_ID = "org.eclipse.jdt.ui.PackageExplorer";
    private static final String KLIGHD_VIEW_ID = "de.cau.cs.kieler.klighd.ui.view.diagram";
    private static final String NEW_FILE_WIZ_ID = "org.eclipse.ui.wizards.new.file";
    
    /** {@inheritDoc} */
    public void createInitialLayout(final IPageLayout layout) {
        final String editor = layout.getEditorArea();
        
        // create general areas relative to editor area
        final IFolderLayout left = layout.createFolder("lf_left", IPageLayout.LEFT, 0.1f, editor);
        final IFolderLayout right = layout.createFolder("lf_right", IPageLayout.RIGHT, 0.4f, editor);

        // add views
        left.addView(PACKAGE_EXPLORER_ID);
        right.addView(KLIGHD_VIEW_ID);

        // activate editor
        layout.setEditorAreaVisible(true);
        
        // add shortcut in case view is closed
        layout.addShowViewShortcut(KLIGHD_VIEW_ID);
        
        // add wizard shortcuts
        layout.addNewWizardShortcut(NEW_FILE_WIZ_ID);
    }
}
