package org.lflang.diagram.lsp;

import org.eclipse.lsp4j.WorkDoneProgressCancelParams;
import de.cau.cs.kieler.klighd.lsp.KGraphLanguageServerExtension;

import org.eclipse.lsp4j.Hover;
import org.eclipse.lsp4j.HoverParams;
import org.eclipse.xtext.ide.server.hover.IHoverService;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.diagram.lsp.CppLanguageServer;

/**
 * The Lingua Franca language and diagram server.
 *
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
public class LFLanguageServer extends KGraphLanguageServerExtension {
    @Override
    public void cancelProgress(WorkDoneProgressCancelParams params) {
        Progress.cancel(params.getToken().getRight().intValue());
    }
    boolean initialized = false;

    @Override
    protected Hover hover(HoverParams params, CancelIndicator cancelIndicator) {
        // This override is just a hacky little patch that is being applied downstream of the original mistake and
        //  upstream of the ungraceful handling (IndexOutOfBoundsException) of said mistake. This patch is applied here
        //  simply because it is easy. This would be done differently were it not for the fact that we plan to rebuild
        //  this infrastructure from scratch anyway.
        if (!initialized) {
            CppLanguageServer.init();
            initialized = true;
        }
        return CppLanguageServer.hoverRequest(params);
        // return IHoverService.EMPTY_HOVER;  // Fail silently
    }
}
