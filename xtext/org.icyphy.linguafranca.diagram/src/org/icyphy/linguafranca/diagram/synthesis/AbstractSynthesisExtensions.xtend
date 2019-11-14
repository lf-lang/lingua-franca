package org.icyphy.linguafranca.diagram.synthesis

import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis
import javax.inject.Inject
import org.eclipse.emf.ecore.EObject

abstract class AbstractSynthesisExtensions {
	
	@Inject AbstractDiagramSynthesis<?> delegate
	
	def boolean getBooleanValue(SynthesisOption option) {
		delegate.getBooleanValue(option)
	}
	
	def <T extends EObject> T associateWith(T derived, Object source) {
		delegate.associateWith(derived, source)
	}
}
