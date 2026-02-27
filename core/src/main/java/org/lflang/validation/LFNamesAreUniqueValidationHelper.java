package org.lflang.validation;

import java.util.Map;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.xtext.naming.QualifiedName;
import org.eclipse.xtext.resource.IEObjectDescription;
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.LfPackage;

public class LFNamesAreUniqueValidationHelper extends NamesAreUniqueValidationHelper {

  /**
   * Lump all inputs, outputs, timers, actions, parameters, and instantiations in the same cluster
   * type. This ensures that names amongst them are checked for uniqueness.
   */
  @Override
  public EClass getAssociatedClusterType(EClass eClass) {
    if (LfPackage.Literals.INPUT == eClass
        || LfPackage.Literals.OUTPUT == eClass
        || LfPackage.Literals.TIMER == eClass
        || LfPackage.Literals.ACTION == eClass
        || LfPackage.Literals.PARAMETER == eClass
        || LfPackage.Literals.INSTANTIATION == eClass) {
      return LfPackage.Literals.VARIABLE;
    }
    return super.getAssociatedClusterType(eClass);
  }

  /** {@inheritDoc} */
  @Override
  @SuppressWarnings("deprecation")
  protected void checkDescriptionForDuplicatedName(
      IEObjectDescription description,
      Map<EClass, Map<QualifiedName, IEObjectDescription>> clusterTypeToName,
      ValidationMessageAcceptor acceptor) {
    // Special handling for value id in AttrParm
    if (description.getEClass() == LfPackage.eINSTANCE.getAttrParm()) {
      var param = (AttrParm) description.getEObjectOrProxy();
      var clusterType = getAssociatedClusterType(param.eClass());
      if (param.eContainer() instanceof Attribute attribute) {
        // Only check for duplicates in the same attribute
        for (int i = 0; i < attribute.getAttrParms().indexOf(param); i++) {
          var prev = attribute.getAttrParms().get(i);
          if (param.getName() == null
              ? param.getName() == prev.getName()
              : param.getName().equals(prev.getName())) {
            createDuplicateNameError(description, clusterType, acceptor);
            return;
          }
        }
      }
    } else {
      super.checkDescriptionForDuplicatedName(description, clusterTypeToName, acceptor);
    }
  }
}
