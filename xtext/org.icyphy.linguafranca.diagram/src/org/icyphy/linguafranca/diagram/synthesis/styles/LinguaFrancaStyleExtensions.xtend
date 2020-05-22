package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KLabel
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KPolyline
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.KText
import de.cau.cs.kieler.klighd.krendering.Underline
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY
import de.cau.cs.kieler.klighd.labels.decoration.IDecoratorRenderingProvider
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator.LayoutMode
import javax.inject.Inject
import org.eclipse.elk.core.math.ElkPadding
import org.eclipse.elk.graph.properties.Property
import org.icyphy.linguafranca.diagram.synthesis.AbstractSynthesisExtensions

/**
 * Extension class that provides styles and coloring for the Lingua France diagram synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaStyleExtensions extends AbstractSynthesisExtensions {
	
	static val LABEL_PARENT_BACKGROUND = new Property<Colors>("org.icyphy.linguafranca.diagram.synthesis.styles.label.parent.background", Colors.WHITE)
	
	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
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
		
		if (r.eContainer instanceof KEdge || r.eContainer instanceof KPort) { // also color potential arrow heads
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
	
	static val CLOUD_WIDTH = 20
	def KContainerRendering addCloudIcon(KContainerRendering parent) {
		return parent.addRectangle() => [
			invisible = true
			
			addRoundedRectangle(CLOUD_WIDTH / 7, CLOUD_WIDTH / 7) => [
				background = Colors.GRAY
				foreground = Colors.GRAY
				setPointPlacementData(LEFT, 2, 0, TOP, 0, 0.5f, H_LEFT, V_TOP, 0, 0, CLOUD_WIDTH, CLOUD_WIDTH / 3)
			]
			addEllipse() => [
				background = Colors.GRAY
				foreground = Colors.GRAY
				setPointPlacementData(LEFT, 0, 0f, TOP, 0, 0.38f, H_LEFT, V_TOP, 0, 0, CLOUD_WIDTH / 2.5f, CLOUD_WIDTH / 2.5f)
			]
			addEllipse() => [
				background = Colors.GRAY
				foreground = Colors.GRAY
				setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0.25f, H_RIGHT, V_TOP, 0, 0, CLOUD_WIDTH / 3f, CLOUD_WIDTH / 3f)
			]
			addEllipse() => [
				background = Colors.GRAY
				foreground = Colors.GRAY
				setPointPlacementData(LEFT, 0, 0.4f, TOP, CLOUD_WIDTH / 10, 0, H_LEFT, V_TOP, 0, 0, CLOUD_WIDTH / 2, CLOUD_WIDTH / 2)
			]
		]
	}
	
	def KRendering addCloudUploadIcon(KContainerRendering parent) {
		return parent.addCloudIcon() => [
			addPolygon() => [
				background = Colors.WHITE
				foreground = Colors.WHITE
				points += #[
					createKPosition(PositionReferenceX.LEFT, -1.5f, 0.5f, PositionReferenceY.TOP, CLOUD_WIDTH / 3, 0.5f),
					createKPosition(PositionReferenceX.LEFT, -1.5f, 0.5f, PositionReferenceY.TOP, 0, 0.58f),
					createKPosition(PositionReferenceX.LEFT, -4, 0.5f, PositionReferenceY.TOP, 0, 0.58f),
					createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0.35f),
					createKPosition(PositionReferenceX.LEFT, 4, 0.5f, PositionReferenceY.TOP, 0, 0.58f),
					createKPosition(PositionReferenceX.LEFT, 1.5f, 0.5f, PositionReferenceY.TOP, 0, 0.58f),
					createKPosition(PositionReferenceX.LEFT, 1.5f, 0.5f, PositionReferenceY.TOP, CLOUD_WIDTH / 3, 0.5f)
				]
			]
		]
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
	
	static var LabelDecorationConfigurator _onEdgePysicalDelayLabelConfigurator; // ONLY for use in applyOnEdgePysicalDelayStyle
	def applyOnEdgePysicalDelayStyle(KLabel label, Colors parentBackgroundColor) {
		if (_onEdgePysicalDelayLabelConfigurator === null) {
	        _onEdgePysicalDelayLabelConfigurator = LabelDecorationConfigurator.create
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
						padding.left = 8
						padding.right = 16
						padding.bottom = Math.max(padding.bottom, 1)
	
						container.children += createKPolygon() => [
							from(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 1, 0.5f)
							to(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 1, 0.5f)
							to(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							background = label.getProperty(LABEL_PARENT_BACKGROUND)
							foreground = label.getProperty(LABEL_PARENT_BACKGROUND)
						]
						container.children += createKSpline() => [
							from(PositionReferenceX.LEFT, -0.66f, 0, PositionReferenceY.BOTTOM, -0.5f , 0.5f)
							to(PositionReferenceX.LEFT, 1, 0, PositionReferenceY.BOTTOM, -0.5f , 0.5f)
							to(PositionReferenceX.LEFT, 3, 0, PositionReferenceY.BOTTOM, 8, 0.5f)
							to(PositionReferenceX.LEFT, 5, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 5.5f, 0, PositionReferenceY.BOTTOM, -1.5f, 0.5f)
						]
						container.children += createKSpline() => [
							from(PositionReferenceX.RIGHT, 15f, 0, PositionReferenceY.BOTTOM, 3.5f , 0.5f)
							to(PositionReferenceX.RIGHT, 14f, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.RIGHT, 11, 0, PositionReferenceY.BOTTOM, -8, 0.5f)
							to(PositionReferenceX.RIGHT, 9, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.RIGHT, 7, 0, PositionReferenceY.BOTTOM, 8, 0.5f)
							to(PositionReferenceX.RIGHT, 4f, 0, PositionReferenceY.BOTTOM, 2, 0.5f)
							to(PositionReferenceX.RIGHT, 1.5f, 0, PositionReferenceY.BOTTOM, 0.5f, 0.5f)
							to(PositionReferenceX.RIGHT, 0.2f, 0, PositionReferenceY.BOTTOM, -0.5f, 0.5f)
							to(PositionReferenceX.RIGHT, -0.7f, 0, PositionReferenceY.BOTTOM, -0.5f, 0.5f)
						]
						container.children += createKPolygon => [
							from(PositionReferenceX.LEFT, 4, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.LEFT, 8, 0, PositionReferenceY.TOP, 0, 0)
							to(PositionReferenceX.RIGHT, 12, 0, PositionReferenceY.TOP, 0, 0)
							to(PositionReferenceX.RIGHT, 16, 0, PositionReferenceY.BOTTOM, 0, 0)
							background = Colors.WHITE
							foreground = Colors.WHITE
						]
						container.children += createKPolyline => [
							from(PositionReferenceX.LEFT, 4, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.LEFT, 8, 0, PositionReferenceY.TOP, 0, 0)
						]
						container.children += createKPolyline => [
							from(PositionReferenceX.RIGHT, 16, 0, PositionReferenceY.BOTTOM, 0, 0)
							to(PositionReferenceX.RIGHT, 12, 0, PositionReferenceY.TOP, 0, 0)
						]
						return padding
					}
	            	
	            })
		}
		label.setProperty(LABEL_PARENT_BACKGROUND, parentBackgroundColor)
		_onEdgePysicalDelayLabelConfigurator.applyTo(label)
	}
	
	static var LabelDecorationConfigurator _onEdgePysicalLabelConfigurator; // ONLY for use in applyOnEdgePysicalStyle
	def applyOnEdgePysicalStyle(KLabel label, Colors parentBackgroundColor) {
		if (_onEdgePysicalLabelConfigurator === null) {
	        _onEdgePysicalLabelConfigurator = LabelDecorationConfigurator.create
	        	.withInlineLabels(true)
	            .withLabelTextRenderingProvider([ container, klabel | 
	            	val kText = createKText()
	            	kText.invisible = true
        			container.children += kText
        			kText
	            ])
	            .addDecoratorRenderingProvider(new IDecoratorRenderingProvider() {
				    
					override createDecoratorRendering(KContainerRendering container, KLabel label, LayoutMode layoutMode) {
	        			val padding = new ElkPadding()
						padding.left = 3
						padding.right = 3
						padding.bottom = Math.max(padding.bottom, 1)
	
						container.children += createKPolygon() => [
							from(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 1, 0.5f)
							to(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 1, 0.5f)
							to(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
							background = label.getProperty(LABEL_PARENT_BACKGROUND)
							foreground = label.getProperty(LABEL_PARENT_BACKGROUND)
						]
						container.children += createKSpline() => [
							from(PositionReferenceX.LEFT, -0.66f, 0, PositionReferenceY.BOTTOM, -0.5f , 0.5f)
							to(PositionReferenceX.LEFT, 1, 0, PositionReferenceY.BOTTOM, -0.5f , 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.1f, PositionReferenceY.BOTTOM, 8, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.2f, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.3f, PositionReferenceY.BOTTOM, -8, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.4f, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.45f, PositionReferenceY.BOTTOM, 4f, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.BOTTOM, 8, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.55f, PositionReferenceY.BOTTOM, 4f, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.6f, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.65f, PositionReferenceY.BOTTOM, -4, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.7f, PositionReferenceY.BOTTOM, -8, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.8f, PositionReferenceY.BOTTOM, -4, 0.5f)
							to(PositionReferenceX.LEFT, 0, 0.9f, PositionReferenceY.BOTTOM, 0, 0.5f)
							to(PositionReferenceX.LEFT, -1, 1, PositionReferenceY.BOTTOM, -0.5f, 0.5f)
							to(PositionReferenceX.LEFT, 0.66f, 1, PositionReferenceY.BOTTOM, -0.5f, 0.5f)
						]
						return padding
					}
	            	
	            })
		}
		label.setProperty(LABEL_PARENT_BACKGROUND, parentBackgroundColor)
		_onEdgePysicalLabelConfigurator.applyTo(label)
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