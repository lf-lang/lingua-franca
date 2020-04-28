package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KLabel
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KPolyline
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.KText
import de.cau.cs.kieler.klighd.krendering.Underline
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY
import de.cau.cs.kieler.klighd.labels.decoration.IDecoratorRenderingProvider
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator.LayoutMode
import javax.inject.Inject
import org.eclipse.elk.core.math.ElkPadding
import org.icyphy.linguafranca.diagram.synthesis.AbstractSynthesisExtensions

/**
 * Extension class that provides styles and coloring for the Lingua France diagram synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaStyleExtensions extends AbstractSynthesisExtensions {

	@Inject extension KRenderingExtensions
	@Inject extension KPolylineExtensions
    extension KRenderingFactory = KRenderingFactory::eINSTANCE

	def noSelectionStyle(KRendering r) {
		r.selectionTextStrikeout = false // prevents default selection style
	}
	
	def underlineSelectionStyle(KRendering r) {
		r.selectionTextUnderline = Underline.SINGLE
	}
	
	def boldLineSelectionStyle(KRendering r) {
		r.selectionLineWidth = 2
	}
		
	def boldTextSelectionStyle(KText t) {
		t.selectionFontBold = true
	}
	
	def errorStyle(KRendering r) {
		r.foreground = Colors.RED
		r.lineWidth = 2
		r.selectionLineWidth = 3
		
		if (r.eContainer instanceof KEdge) { // also color potential arrow heads
			r.background = Colors.RED
			r.background.propagateToChildren = true
			r.foreground.propagateToChildren = true
			r.lineWidth.propagateToChildren = true
		}
	}
	
	def commentStyle(KRendering r) {
		r.foreground = Colors.LIGHT_GOLDENROD
		r.background = Colors.PALE_GOLDENROD
		r.lineWidth = 1
		r.selectionLineWidth = 2
		
		if (r.eContainer instanceof KEdge) { // also color potential arrow heads
			r.background = Colors.LIGHT_GOLDENROD
			r.background.propagateToChildren = true
			r.foreground.propagateToChildren = true
			r.lineWidth.propagateToChildren = true
		}
	}
	
	static var LabelDecorationConfigurator _onEdgeLabelConfigurator; // ONLY for use in applyOnEdgeStyle
	def applyOnEdgeStyle(KLabel label) {
		if (_onEdgeLabelConfigurator === null) {
	        _onEdgeLabelConfigurator = LabelDecorationConfigurator.create
	        	.withInlineLabels(true)
	            .withLabelTextRenderingProvider([ container, klabel | 
	            	val kText = createKText()
	            	kText.fontSize = 9
        			container.children += kText
        			kText
	            ])
		}
		_onEdgeLabelConfigurator.applyTo(label)
	}
	
	static var LabelDecorationConfigurator _onEdgeDelayLabelConfigurator; // ONLY for use in applyOnEdgeDelayStyle
	def applyOnEdgeDelayStyle(KLabel label) {
		if (_onEdgeDelayLabelConfigurator === null) {
	        _onEdgeDelayLabelConfigurator = LabelDecorationConfigurator.create
	        	.withInlineLabels(true)
	            .withLabelTextRenderingProvider([ container, klabel | 
	            	val kText = createKText()
	            	kText.fontSize = 8
	            	kText.boldTextSelectionStyle()
					kText.setProperty(KlighdInternalProperties.MODEL_ELEMEMT, klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT))
	            	
        			container.children += kText
        			kText
	            ])
	            .addDecoratorRenderingProvider(new IDecoratorRenderingProvider() {
				    
					override createDecoratorRendering(KContainerRendering container, KLabel label, LayoutMode layoutMode) {
	        			val padding = new ElkPadding()
						padding.left = 2
						padding.right = 2
						padding.bottom = Math.max(padding.bottom, 1)
	
						container.children += createKPolygon => [
							from(PositionReferenceX.LEFT, -2, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.LEFT, 2, 0, PositionReferenceY.TOP, 0, 0)
							to(PositionReferenceX.RIGHT, -2, 0, PositionReferenceY.TOP, 0, 0)
							to(PositionReferenceX.RIGHT, 2, 0, PositionReferenceY.BOTTOM, 0, 0)
							background = Colors.WHITE
							foreground = Colors.WHITE
						]
						container.children += createKPolyline => [
							from(PositionReferenceX.LEFT, -2, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.LEFT, 2, 0, PositionReferenceY.TOP, 0, 0)
						]
						container.children += createKPolyline => [
							from(PositionReferenceX.RIGHT, 2, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.RIGHT, -2, 0, PositionReferenceY.TOP, 0, 0)
						]
						return padding
					}
	            	
	            })
		}
		_onEdgeDelayLabelConfigurator.applyTo(label)
	}
	
    def KRendering addFixedTailArrowDecorator(KPolyline pl) {
    	val head = pl.addTailArrowDecorator()
		head.placementData = createKDecoratorPlacementData => [
            it.rotateWithLine = true
            it.relative = 0f
            it.absolute = 2f
            it.width = 8
            it.height = 6
            it.setXOffset(-3f)
            it.setYOffset(-4f)
        ]
        return head
    }

}