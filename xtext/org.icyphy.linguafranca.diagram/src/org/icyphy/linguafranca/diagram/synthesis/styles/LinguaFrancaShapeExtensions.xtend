package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.KlighdConstants
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.VerticalAlignment
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY
import javax.inject.Inject
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguafranca.diagram.synthesis.AbstractSynthesisExtensions
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesisUtilityExtensions

import static org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis.*

import static extension de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses.*
import de.cau.cs.kieler.klighd.krendering.KText

@ViewSynthesisShared
class LinguaFrancaShapeExtensions extends AbstractSynthesisExtensions {
	
	public static val float REACTION_POINTINESS = 5 // arrow point length 

	@Inject extension KNodeExtensions
	@Inject extension KEdgeExtensions
	@Inject extension KPortExtensions
	@Inject extension KLabelExtensions
	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
	@Inject extension KPolylineExtensions
	@Inject extension KColorExtensions
	@Inject extension LinguaFrancaStyleExtensions
	@Inject extension LinguaFrancaSynthesisUtilityExtensions

	/**
	 * Creates the visual representation of a reactor node
	 */
	def addReactorFigure(KNode node, Reactor reactor, String instanceName) {
		val figure = node.addRoundedRectangle(8, 8, 1)
		figure.setGridPlacement(1)
		figure.lineWidth = 1
		figure.foreground = Colors.GRAY
		figure.background = Colors.GRAY_95

		// minimal node size is necessary if no text will be added
		node.setMinimalNodeSize(2 * figure.cornerWidth, 2 * figure.cornerHeight)

		val showInstanceName = SHOW_INSTANCE_NAMES.booleanValue && !instanceName.nullOrEmpty
		
		figure.addText(reactor.name) => [
			setGridPlacementData().from(LEFT, 8, 0, TOP, 8f, 0).to(RIGHT, 8, 0, BOTTOM, showInstanceName ? 0 : (reactor.hasContent ? 4 : 8), 0)
			suppressSelectability
		]
		
		if (showInstanceName) {
			figure.addText(instanceName) => [
				fontItalic = true
				setGridPlacementData().from(LEFT, 8, 0, TOP, 2, 0).to(RIGHT, 8, 0, BOTTOM, reactor.hasContent ? 4 : 8, 0)
				suppressSelectability
			]
		}

		return figure
	}

	/**
	 * Creates the visual representation of a timer node
	 */
	def addTimerFigure(KNode node, Timer timer) {
		node.setMinimalNodeSize(40, 40)
		
		val figure = node.addEllipse
		figure.lineWidth = 1
		figure.background = Colors.GRAY_95
		
		figure.addPolyline(1,
			#[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.1f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.7f, PositionReferenceY.TOP, 0 , 0.7f)
			]
		)
		
		if (timer.timing !== null) {
			val labelParts = newArrayList
			if (timer.timing.offset !== null) {
				labelParts += timer.timing.offset.toText
			}
			if (timer.timing.period !== null) {
				labelParts += timer.timing.period.toText
			}
			if (!labelParts.empty) {
				node.addOutsideBottomCenteredNodeLabel(labelParts.join("(", ", ", ")")[it])
			}
		}

		return figure
	}
	
	/**
	 * Creates the visual representation of a reaction node
	 */
	def addReactionFigure(KNode node, Reaction reaction) {
		val minHeight = 15
		node.setMinimalNodeSize(45, minHeight)
		
		val baseShape = node.addPolygon() => [
			associateWith(reaction)
			
			// style
			lineWidth = 1
			foreground = Colors.GRAY_45
			background = Colors.GRAY_65
			boldLineSelectionStyle
			
			points += createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0, 0)
			points += createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.TOP, 0, 0)
			points += createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f)
			points += createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0)
			points += createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
			points += createKPosition(PositionReferenceX.LEFT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
		]
		
		val contentContainer = baseShape.addRectangle() => [
			associateWith(reaction)
			invisible = true
			setLeftTopAlignedPointPlacementData(0, REACTION_POINTINESS, 0, REACTION_POINTINESS)
			gridPlacement = 1
		]

		// optional code content
		if (SHOW_REACTION_CODE.booleanValue && !reaction.code.nullOrEmpty) {
			contentContainer.addText(reaction.code) => [
				associateWith(reaction)
				fontSize = 6
				noSelectionStyle
				horizontalAlignment = HorizontalAlignment.LEFT
				verticalAlignment = VerticalAlignment.TOP
				setGridPlacementData().from(LEFT, 5, 0, TOP, 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
			]
		}
		
		if (reaction.deadline !== null) {
			baseShape.addPolygon() => [
				associateWith(reaction.deadline)
				
				points += createKPosition(PositionReferenceX.LEFT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
				points += createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f)
				points += createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0)
				points += createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
				
				// style
				lineWidth = 1
				foreground = Colors.GRAY_45
				background = Colors.BROWN
				
				// ensure content is rendered on top
				baseShape.children.move(baseShape.children.size - 1, contentContainer)
			]
				
			val deadlineSection = contentContainer.addRectangle() => [
				associateWith(reaction.deadline)
				invisible = true
				verticalAlignment = VerticalAlignment.BOTTOM
				if (!SHOW_REACTION_CODE.booleanValue || reaction.code.nullOrEmpty) {
					setGridPlacementData().from(LEFT, 0, 0, TOP, -3, 0.5f).to(RIGHT, 0, 0, BOTTOM, 0, 0)
				}
				gridPlacement = 1
			]
				
			// delay
			deadlineSection.addText(reaction.deadline.time.toText) => [
				associateWith(reaction.deadline.time)
				fontBold = true
				fontSize = 7
				setGridPlacementData().from(LEFT, 5, 0, TOP, 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
				underlineSelectionStyle
			]
				
			// optional code content
			if (SHOW_REACTION_CODE.booleanValue && !reaction.deadline.deadlineCode.nullOrEmpty) {
				deadlineSection.addText(reaction.code) => [
					associateWith(reaction.deadline)
					fontSize = 6
					setGridPlacementData().from(LEFT, 5, 0, TOP, 0, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
					horizontalAlignment = HorizontalAlignment.LEFT
					noSelectionStyle
				]
			}
		}

		return baseShape
	}
	
	/**
	 * Creates the visual representation of a reactor port.
	 */
	def addTrianglePort(KPort port) {
		port.addPolygon() => [
			lineWidth = 1
			background = Colors.BLACK
			points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0 , 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0 , 0)
			]
		]
	}
	
	/**
	 * Added a text as collapse expand button.
	 */
	def KText addCollapseExpandButton(KContainerRendering container, String text) {
		container.addText(text) => [
			foreground = Colors.BLUE
			fontSize = 8
			addSingleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
			addDoubleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
		]
	}
	
	/**
	 * Creates and adds an error message figure
	 */
	def addErrorMessage(KNode node, String title, String message) {
		node.addRectangle() => [
            invisible = true
            addRoundedRectangle(7, 7) => [
                setGridPlacement(1)
                lineWidth = 2
                // title
                if (title !== null) {
	                addText(title) => [
	                    fontSize = 12
	                    setFontBold = true
	                    foreground = Colors.RED
	                    setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
	                    suppressSelectability()
	                ]
                }
                // message
                if (message !== null) {
	                addText(message) => [
                        if (title !== null) {
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0);
                        } else {
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0);
                        }
	                ]
	            }
            ]
		]
	}

}