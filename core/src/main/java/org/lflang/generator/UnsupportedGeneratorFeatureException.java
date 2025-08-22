package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;

/**
 * Exception signaling that the code generator does not support a particular feature of the source language.
 *
 * @ingroup Validation
 */
public class UnsupportedGeneratorFeatureException extends GenerationException {

  public UnsupportedGeneratorFeatureException(String feature) {
    super("Unsupported generator feature: " + feature);
  }

  public UnsupportedGeneratorFeatureException(EObject location, String feature) {
    super(location, "Unsupported generator feature: " + feature);
  }
}
