package org.lflang.diagram.synthesis;

import org.lflang.diagram.synthesis.action.CollapseAllReactorsAction;
import org.lflang.diagram.synthesis.action.ExpandAllReactorsAction;
import org.lflang.diagram.synthesis.action.FilterCycleAction;
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction;
import org.lflang.diagram.synthesis.action.ShowCycleAction;
import org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;

import de.cau.cs.kieler.klighd.IKlighdStartupHook;
import de.cau.cs.kieler.klighd.KlighdDataManager;

/**
 * Registration of all diagram synthesis related classes in Klighd.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class SynthesisRegistration implements IKlighdStartupHook {
    
    @Override
    public void execute() {
        KlighdDataManager reg = KlighdDataManager.getInstance();
        
        // Synthesis
        reg.registerDiagramSynthesisClass(LinguaFrancaSynthesis.ID, LinguaFrancaSynthesis.class);
        
        // Actions
        reg.registerAction(MemorizingExpandCollapseAction.ID, new MemorizingExpandCollapseAction());
        reg.registerAction(ExpandAllReactorsAction.ID, new ExpandAllReactorsAction());
        reg.registerAction(CollapseAllReactorsAction.ID, new CollapseAllReactorsAction());
        reg.registerAction(ShowCycleAction.ID, new ShowCycleAction());
        reg.registerAction(FilterCycleAction.ID, new FilterCycleAction());
        
        // Style Mod
        reg.registerStyleModifier(ReactionPortAdjustment.ID, new ReactionPortAdjustment());
        
        // Blacklist LF-specific properties that should be removed when a diagram is sent from the diagram server to a client.
        reg.registerBlacklistedProperty(FilterCycleAction.FILTER_BUTTON);
        reg.registerBlacklistedProperty(LinguaFrancaSynthesis.REACTOR_HAS_BANK_PORT_OFFSET);
        reg.registerBlacklistedProperty(LinguaFrancaSynthesis.REACTOR_INPUT);
        reg.registerBlacklistedProperty(LinguaFrancaSynthesis.REACTOR_OUTPUT);
        reg.registerBlacklistedProperty(LinguaFrancaSynthesis.REACTION_SPECIAL_TRIGGER);
        reg.registerBlacklistedProperty(ReactionPortAdjustment.PROCESSED);
        reg.registerBlacklistedProperty(LinguaFrancaShapeExtensions.REACTOR_CONTENT_CONTAINER);
        reg.registerBlacklistedProperty(LinguaFrancaStyleExtensions.LABEL_PARENT_BACKGROUND);
        reg.registerBlacklistedProperty(NamedInstanceUtil.LINKED_INSTANCE); // Very important since its values can not be synthesized easily!
    }
    
}
