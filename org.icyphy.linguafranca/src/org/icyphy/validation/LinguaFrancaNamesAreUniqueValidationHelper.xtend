package org.icyphy.validation;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper;
import org.icyphy.linguaFranca.LinguaFrancaPackage

class LinguaFrancaNamesAreUniqueValidationHelper extends
        NamesAreUniqueValidationHelper {

    /**
     * Lump all inputs, outputs, timers, actions, parameters, and 
     * instantiations in the same cluster type. This ensures that
     * names amongst them are checked for uniqueness.
     */
    override getAssociatedClusterType(EClass eClass) {
        if (LinguaFrancaPackage.Literals.INPUT == eClass || 
            LinguaFrancaPackage.Literals.OUTPUT == eClass ||
            LinguaFrancaPackage.Literals.TIMER == eClass ||
            LinguaFrancaPackage.Literals.ACTION == eClass ||
            LinguaFrancaPackage.Literals.PARAMETER == eClass ||
            LinguaFrancaPackage.Literals.INSTANTIATION == eClass
        )  {
            return LinguaFrancaPackage.Literals.VARIABLE;
        }
        return super.getAssociatedClusterType(eClass);
    }
    
}