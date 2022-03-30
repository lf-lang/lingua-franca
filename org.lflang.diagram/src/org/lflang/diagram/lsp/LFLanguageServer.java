package org.lflang.diagram.lsp;

import org.eclipse.lsp4j.WorkDoneProgressCancelParams;
import de.cau.cs.kieler.klighd.lsp.KGraphLanguageServerExtension;

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
}
