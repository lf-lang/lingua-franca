package org.icyphy.linguafranca.diagram.synthesis

import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit

@ViewSynthesisShared
class LinguaFrancaSynthesisUtilityExtensions extends AbstractSynthesisExtensions {
	
	/**
	 * Converts a timing value into readable text
	 */
	def toText(TimeOrValue tov) {
		if (tov.parameter !== null) {
			return tov.parameter.name
		} else if (tov.value !== null) {
			return tov.value
		} else if (tov.unit === TimeUnit.NONE) {
			return Integer.toString(tov.time)
		} else {
			return tov.time + "" + tov.unit.literal
		}
	}
	
	/**
	 * Returns true if the reactor as has inner reactions or instances
	 */
	def hasContent(Reactor reactor) {
		return !reactor.reactions.empty || !reactor.instantiations.empty
	}

}
