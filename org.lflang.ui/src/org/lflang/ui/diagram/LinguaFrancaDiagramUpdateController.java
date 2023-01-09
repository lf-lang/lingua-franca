/*************
* Copyright (c) 2022, Kiel University.
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
package org.lflang.ui.diagram;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.xtext.ui.editor.LanguageSpecificURIEditorOpener;
import org.eclipse.xtext.ui.editor.XtextEditor;
import org.lflang.lf.Model;
import org.lflang.ui.internal.LflangActivator;

import com.google.inject.Injector;

import de.cau.cs.kieler.klighd.KlighdTreeSelection;
import de.cau.cs.kieler.klighd.piccolo.internal.KlighdCanvas;
import de.cau.cs.kieler.klighd.piccolo.internal.events.KlighdInputManager.KlighdInputEvent;
import de.cau.cs.kieler.klighd.piccolo.internal.nodes.KlighdMainCamera;
import de.cau.cs.kieler.klighd.ui.view.DiagramView;
import de.cau.cs.kieler.klighd.ui.view.controllers.EcoreXtextSaveUpdateController;
import de.cau.cs.kieler.klighd.ui.view.controllers.XtextSelectionHighlighter;
import de.cau.cs.kieler.klighd.util.KlighdSynthesisProperties;
import edu.umd.cs.piccolo.event.PInputEvent;
import edu.umd.cs.piccolo.event.PInputEventListener;

/**
 * Controller that handles the behavior between the diagram view and the Lingua Franca editor.
 * 
 * This class is registered and associated with the LF editor via the plugin.xml, if the class name
 * changes, update the plugin.xml!
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class LinguaFrancaDiagramUpdateController extends EcoreXtextSaveUpdateController implements PInputEventListener {
    
    /** The xtext injector for LF */
    private Injector injector;
    /** Xtext utility class that is normally used for jumpt-to-declaration actions in the editor */
    private LanguageSpecificURIEditorOpener uriOpener;
    /** The camera the key listener is registered on */
    private KlighdMainCamera camera;
    /** Flag to activate code association for diagram elements which source is outside the current editor */
    private boolean jumpToFile = false;
    
    /**
     * {@inheritDoc}
     */
    @Override
    public String getID() {
        return this.getClass().getName();
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public void initialize(final DiagramView parentDiagramView) {
        super.initialize(parentDiagramView);
        
        // Get LF Xtext injector
        injector = LflangActivator.getInstance().getInjector(LflangActivator.ORG_LFLANG_LF);
        uriOpener = injector.getInstance(LanguageSpecificURIEditorOpener.class);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onDiagramUpdate(final Object model, final KlighdSynthesisProperties properties) {
        if (getDiagramView().getViewer() != null && getDiagramView().getViewer().getControl() instanceof KlighdCanvas) {
            var canvas = (KlighdCanvas) getDiagramView().getViewer().getControl();
            var cam = canvas.getCamera();
            if (camera != cam) {
                if (camera != null) {
                    camera.removeInputEventListener(this);
                }
                
                camera = cam;
                jumpToFile = false;
                camera.addInputEventListener(this);
            }
        }
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public void onDispose() {
        if (camera != null) {
            camera.removeInputEventListener(this);
        }
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public void processEvent(PInputEvent event, int type) {
        if (event instanceof KlighdInputEvent) {
            // Check for ALT key to activate cross file association
            jumpToFile = ((KlighdInputEvent) event).isAltDown();
        }
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public void selectionChanged(final SelectionChangedEvent event) {
        if (getEditor() instanceof XtextEditor) {
            if (event.getSelection() instanceof KlighdTreeSelection) {
                var selection = (KlighdTreeSelection) event.getSelection();
                
                // Perform jump to potentially different editor?
                if (jumpToFile && selection.size() == 1) {
                    var source = selection.sourceElementIterator().next();
                    if (source instanceof EObject) { // Source is LF model element
                        var sourceURI = EcoreUtil.getURI((EObject) source);
                        if (uriOpener != null && sourceURI != null) {
                            var editor = uriOpener.open(sourceURI, false);
                            if (editor instanceof XtextEditor) {
                                XtextSelectionHighlighter.highlightSelection((XtextEditor) editor, event.getSelection());
                            }
                            return; // Suppress other behavior
                        }
                    }
                }
                
                // Wrap selection in adjusted one such that elements from imported reactors are associated with the import
                if (getModel() instanceof Model) {
                    selection = new LinguaFrancaAdjustedKlighdTreeSelection(selection, (Model) getModel());
                }
                XtextSelectionHighlighter.highlightSelection((XtextEditor) getEditor(), selection);
            }
        }
    }
    
}
