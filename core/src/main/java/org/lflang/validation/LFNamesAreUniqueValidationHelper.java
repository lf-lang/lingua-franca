package org.lflang.validation;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper;
import org.lflang.lf.LfPackage.Literals;

public class LFNamesAreUniqueValidationHelper extends NamesAreUniqueValidationHelper {

  /**
   * Lump all inputs, outputs, timers, actions, parameters, and instantiations in the same cluster
   * type. This ensures that names amongst them are checked for uniqueness.
   */
  @Override
  public EClass getAssociatedClusterType(EClass eClass) {
    if (Literals.ACTION == eClass
        || Literals.INPUT == eClass
        || Literals.INSTANTIATION == eClass
        || Literals.METHOD == eClass
        || Literals.OUTPUT == eClass
        || Literals.PARAMETER == eClass
        || Literals.REACTION == eClass
        || Literals.STATE_VAR == eClass
        || Literals.TIMER == eClass
        || Literals.WATCHDOG == eClass) {
      return Literals.MEMBER;
    }
    return super.getAssociatedClusterType(eClass);
  }
}
