package org.lflang.diagram.synthesis;

import de.cau.cs.kieler.klighd.IKlighdStartupHook;
import de.cau.cs.kieler.klighd.KlighdDataManager;
import org.lflang.diagram.synthesis.action.CollapseAllReactorsAction;
import org.lflang.diagram.synthesis.action.ExpandAllReactorsAction;
import org.lflang.diagram.synthesis.action.FilterCycleAction;
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction;
import org.lflang.diagram.synthesis.action.ShowCycleAction;
import org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment;

/**
 * Registration of all diagram synthesis related classes in Klighd.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
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
    }
    
}
