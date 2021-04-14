package org.lflang.validation;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper;
import org.lflang.meta.MetaPackage

class LinguaFrancaNamesAreUniqueValidationHelper extends
        NamesAreUniqueValidationHelper {

    /**
     * Lump all inputs, outputs, timers, actions, parameters, and 
     * instantiations in the same cluster type. This ensures that
     * names amongst them are checked for uniqueness.
     */
    override getAssociatedClusterType(EClass eClass) {
        if (MetaPackage.Literals.INPUT == eClass || 
            MetaPackage.Literals.OUTPUT == eClass ||
            MetaPackage.Literals.TIMER == eClass ||
            MetaPackage.Literals.ACTION == eClass ||
            MetaPackage.Literals.PARAMETER == eClass ||
            MetaPackage.Literals.INSTANTIATION == eClass
        )  {
            return MetaPackage.Literals.VARIABLE;
        }
        return super.getAssociatedClusterType(eClass);
    }
    
}