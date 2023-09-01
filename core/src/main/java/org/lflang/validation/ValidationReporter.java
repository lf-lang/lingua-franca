package org.lflang.validation;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;

public interface ValidationReporter {
    void error(String message, EObject source, EStructuralFeature feature);

    void warning(String message, EObject source, EStructuralFeature feature);
}