package org.lflang.validation;

import org.eclipse.emf.ecore.EClass
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper
import org.lflang.lf.LfPackage

class LFNamesAreUniqueValidationHelper extends
        NamesAreUniqueValidationHelper {

    /**
     * Lump all inputs, outputs, timers, actions, parameters, and 
     * instantiations in the same cluster type. This ensures that
     * names amongst them are checked for uniqueness.
     */
    override getAssociatedClusterType(EClass eClass) {
        if (LfPackage.Literals.INPUT == eClass || 
            LfPackage.Literals.OUTPUT == eClass ||
            LfPackage.Literals.TIMER == eClass ||
            LfPackage.Literals.ACTION == eClass ||
            LfPackage.Literals.PARAMETER == eClass ||
            LfPackage.Literals.INSTANTIATION == eClass
        )  {
            return LfPackage.Literals.VARIABLE;
        }
        return super.getAssociatedClusterType(eClass);
    }
    
}