package org.lflang.diagram.synthesis;

import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis;
import org.eclipse.emf.ecore.EObject;

/**
 * Abstract super class for extension classes used in for the diagram synthesis that provides some
 * convince methods.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public abstract class AbstractSynthesisExtensions {

  @Inject private AbstractDiagramSynthesis<?> delegate;

  public boolean getBooleanValue(SynthesisOption option) {
    return delegate.getBooleanValue(option);
  }

  public float getFloatValue(SynthesisOption option) {
    return delegate.getFloatValue(option);
  }

  public Object getObjectValue(final SynthesisOption option) {
    return delegate.getObjectValue(option);
  }

  public <T extends EObject> T associateWith(T derived, Object source) {
    return delegate.associateWith(derived, source);
  }

  @SuppressWarnings("unchecked")
  public <T extends AbstractDiagramSynthesis<?>> T getRootSynthesis() {
    return (T) delegate;
  }
}
