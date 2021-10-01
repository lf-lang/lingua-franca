package org.lflang.diagram.lsp
import org.lflang.generator.LanguageServerErrorReporter

import de.cau.cs.kieler.klighd.lsp.KGraphLanguageClient
import de.cau.cs.kieler.klighd.lsp.interactive.layered.LayeredInteractiveLanguageServerExtension
import de.cau.cs.kieler.klighd.lsp.interactive.rectpacking.RectpackingInteractiveLanguageServerExtension
import de.cau.cs.kieler.klighd.lsp.launch.AbstractLanguageServer
import de.cau.cs.kieler.klighd.lsp.launch.AbstractLsCreator
import de.cau.cs.kieler.klighd.lsp.launch.AbstractRegistrationLanguageServerExtension
import de.cau.cs.kieler.klighd.lsp.launch.ILanguageRegistration
import de.cau.cs.kieler.klighd.lsp.launch.Language
import org.lflang.ide.LFIdeSetup

/**
 * Language server with extended diagram communication.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class LanguageDiagramServer extends AbstractLanguageServer {
    
    def static void main(String[] args) {
        new LanguageDiagramServer().start()
    }
    
    def start() {
        this.configureAndRun(new LFLanguageRegistration, new LFLsCreator)
    }
}

class LFLsCreator extends AbstractLsCreator {
    
    LayeredInteractiveLanguageServerExtension constraints
    RectpackingInteractiveLanguageServerExtension rectPack
    LFLanguageServerExtension lfExtension
    
    override getLanguageServerExtensions() {
        constraints = injector.getInstance(LayeredInteractiveLanguageServerExtension)
        rectPack = injector.getInstance(RectpackingInteractiveLanguageServerExtension)
        lfExtension = injector.getInstance(LFLanguageServerExtension)
        return newArrayList(
            injector.getInstance(LFRegistrationLanguageServerExtension), constraints, rectPack, lfExtension
        )
    }
    
    override getRemoteInterface() {
        KGraphLanguageClient
    }
    
    override onConnect() {
        super.onConnect()
        constraints.client = languageClient as KGraphLanguageClient
        rectPack.client = languageClient as KGraphLanguageClient
        LanguageServerErrorReporter.setClient(languageClient)
    }
}

class LFLanguageRegistration implements ILanguageRegistration {
    
    override bindAndRegisterLanguages() {        
        LFIdeSetup.doSetup()
    }
}

class LFRegistrationLanguageServerExtension extends AbstractRegistrationLanguageServerExtension {
    
    override getLanguageExtensions() {
        return newArrayList(new Language("lf", "Lingua Franca", #[]))
    }
}