package org.lflang.diagram.lsp;

import java.util.List;

import org.eclipse.xtext.Constants;
import org.eclipse.xtext.IGrammarAccess;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.LanguageServerImpl;
import org.eclipse.xtext.service.AbstractGenericModule;
import org.eclipse.xtext.util.Modules2;
import org.lflang.generator.LanguageServerErrorReporter;
import org.lflang.ide.LFIdeSetup;

import com.google.inject.Module;
import com.google.inject.name.Names;
import com.google.inject.util.Modules;

import de.cau.cs.kieler.kgraph.text.services.KGraphGrammarAccess;
import de.cau.cs.kieler.klighd.lsp.KGraphLanguageClient;
import de.cau.cs.kieler.klighd.lsp.interactive.layered.LayeredInteractiveLanguageServerExtension;
import de.cau.cs.kieler.klighd.lsp.interactive.rectpacking.RectpackingInteractiveLanguageServerExtension;
import de.cau.cs.kieler.klighd.lsp.launch.AbstractLanguageServer;
import de.cau.cs.kieler.klighd.lsp.launch.AbstractLsCreator;
import de.cau.cs.kieler.klighd.lsp.launch.AbstractRegistrationLanguageServerExtension;
import de.cau.cs.kieler.klighd.lsp.launch.ILanguageRegistration;
import de.cau.cs.kieler.klighd.lsp.launch.Language;

/**
 * Language server with extended diagram communication.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class LanguageDiagramServer extends AbstractLanguageServer {
    
    private static class LFLsCreator extends AbstractLsCreator {

        @Override
        public Module createLSModules(boolean socket) {
            return Modules2.mixin(Modules.override(super.createLSModules(socket)).with(new AbstractGenericModule() {
                public Class<? extends LanguageServerImpl> bindLanguageServerImpl() {
                    return LFLanguageServer.class;
                }
            }), it -> {
                // Temporary fix for an issue of Klighd with Xtext 2.28 (https://github.com/kieler/KLighD/issues/144)
                it.bind(IGrammarAccess.class).to(KGraphGrammarAccess.class);
                it.bind(String.class).annotatedWith(Names.named(Constants.LANGUAGE_NAME)).toInstance("de.cau.cs.kieler.kgraph.text.KGraph");    
            });
        }
        
        LayeredInteractiveLanguageServerExtension constraints;
        RectpackingInteractiveLanguageServerExtension rectPack;
        LFLanguageServerExtension lfExtension;
        
        @Override
        public List<ILanguageServerExtension> getLanguageServerExtensions() {
            constraints = injector.getInstance(LayeredInteractiveLanguageServerExtension.class);
            rectPack = injector.getInstance(RectpackingInteractiveLanguageServerExtension.class);
            lfExtension = injector.getInstance(LFLanguageServerExtension.class);
            return List.of(
                injector.getInstance(LFRegistrationLanguageServerExtension.class), constraints, rectPack, lfExtension
            );
        }
        
        @Override
        public Class<? extends KGraphLanguageClient> getRemoteInterface() {
            return KGraphLanguageClient.class;
        }
        
        @Override
        public void onConnect() {
            super.onConnect();
            constraints.setClient((KGraphLanguageClient) languageClient);
            rectPack.setClient((KGraphLanguageClient) languageClient);
            LanguageServerErrorReporter.setClient(languageClient);
            lfExtension.setClient(languageClient);
            // The following is needed because VS Code treats System.err like System.out and System.out like a shout
            // into the void.
            System.setOut(System.err);
        }
    }
    
    private static class LFLanguageRegistration implements ILanguageRegistration {
        
        @Override
        public void bindAndRegisterLanguages() {
            LFIdeSetup.doSetup();
        }
    }

    private static class LFRegistrationLanguageServerExtension extends AbstractRegistrationLanguageServerExtension {
        
        @Override
        public List<Language> getLanguageExtensions() {
            return List.of(new Language("lf", "Lingua Franca", List.of()));
        }
    }
    
    public static void main(String[] args) {
        new LanguageDiagramServer().start();
    }
    
    public void start() {
        configureAndRun(new LFLanguageRegistration(), new LFLsCreator());
    }
}
