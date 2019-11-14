package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.kgraph.KLabel
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.KText
import de.cau.cs.kieler.klighd.krendering.Underline
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator
import javax.inject.Inject
import org.icyphy.linguafranca.diagram.synthesis.AbstractSynthesisExtensions

@ViewSynthesisShared
class LinguaFrancaStyleExtensions extends AbstractSynthesisExtensions {

	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
	@Inject extension KPolylineExtensions
	@Inject extension KColorExtensions

	def noSelectionStyle(KText text) {
		text.selectionTextStrikeout = false // prevents default selection style
	}
	
	def underlineSelectionStyle(KText text) {
		text.selectionTextUnderline = Underline.SINGLE
	}
	
	def boldLineSelectionStyle(KRendering r) {
		r.selectionLineWidth = 2
	}
	
	static var LabelDecorationConfigurator _inlineLabelConfigurator; // ONLY for use in applyOnEdgeStyle
	def applyOnEdgeStyle(KLabel label) {
		if (_inlineLabelConfigurator === null) {
	        _inlineLabelConfigurator = LabelDecorationConfigurator.create
	        	.withInlineLabels(true)
	            .withLabelTextRenderingProvider([ container, klabel | 
	            	val kText = KRenderingFactory.eINSTANCE.createKText()
	            	kText.fontSize = 9
        			container.children += kText
        			kText
	            ])
		}
		
		_inlineLabelConfigurator.applyTo(label)
	}

}