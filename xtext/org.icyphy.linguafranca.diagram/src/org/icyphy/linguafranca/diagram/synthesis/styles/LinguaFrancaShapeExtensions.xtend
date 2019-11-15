package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KText
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

@ViewSynthesisShared
class LinguaFrancaShapeExtensions extends AbstractSynthesisExtensions {
	
	public static val float REACTION_POINTINESS = 6 // arrow point length 

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
	 * Creates the main reactor frame.
	 */
	def addMainReactorFigure(KNode node, Reactor reactor) {
		val figure = node.addRoundedRectangle(8, 8, 1) => [
			setGridPlacement(1)
			lineWidth = 1
			foreground = Colors.GRAY
		]
		
		figure.addText(reactor.name) => [
			setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
			suppressSelectability
			underlineSelectionStyle
		]
		
		return figure
	}

	/**
	 * Creates the visual representation of a reactor node
	 */
	def addReactorFigure(KNode node, Reactor reactor, String instanceName) {
		val figure = node.addRoundedRectangle(8, 8, 1) => [
			setGridPlacement(1)
			lineWidth = 1
			foreground = Colors.GRAY
			background = Colors.GRAY_95
		]

		// minimal node size is necessary if no text will be added
		node.setMinimalNodeSize(2 * figure.cornerWidth, 2 * figure.cornerHeight)

		val showInstanceName = SHOW_INSTANCE_NAMES.booleanValue && !instanceName.nullOrEmpty
		
		figure.addText(reactor.name) => [
			setGridPlacementData().from(LEFT, 8, 0, TOP, 8f, 0).to(RIGHT, 8, 0, BOTTOM, showInstanceName ? 0 : (reactor.hasContent ? 4 : 8), 0)
			suppressSelectability
			underlineSelectionStyle
		]
		
		if (showInstanceName) {
			figure.addText(instanceName) => [
				fontItalic = true
				setGridPlacementData().from(LEFT, 8, 0, TOP, 2, 0).to(RIGHT, 8, 0, BOTTOM, reactor.hasContent ? 4 : 8, 0)
				suppressSelectability
				noSelectionStyle
			]
		}

		return figure
	}
	
	/**
	 * Creates the visual representation of a reaction node
	 */
	def addReactionFigure(KNode node, Reaction reaction) {
		val minHeight = 22
		val minWidth = 45
		val reactor = reaction.eContainer as Reactor
		node.setMinimalNodeSize(minWidth, minHeight)
		
		val baseShape = node.addPolygon() => [
			associateWith(reaction)
			
			// style
			lineWidth = 1
			foreground = Colors.GRAY_45
			background = Colors.GRAY_65
			boldLineSelectionStyle()
			
			points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0, 0),
				createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.TOP, 0, 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f),
				createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0),
				createKPosition(PositionReferenceX.LEFT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
			]
		]
		
		val order = if (reactor.reactions.size > 1) {
			baseShape.addText(Integer.toString(reactor.reactions.indexOf(reaction) + 1)) => [
				fontBold = true
				noSelectionStyle
				suppressSelectability
			]
		}
		
		val contentContainer = baseShape.addRectangle() => [
			associateWith(reaction)
			invisible = true
			//setLeftTopAlignedPointPlacementData(0, REACTION_POINTINESS, 0, REACTION_POINTINESS)
			setPointPlacementData(LEFT, REACTION_POINTINESS, 0, TOP, 0, 0, H_LEFT, V_TOP, REACTION_POINTINESS, 0, minWidth - REACTION_POINTINESS * 2, minHeight)
			gridPlacement = 1
		]

		// optional code content
		if (SHOW_REACTION_CODE.booleanValue && !reaction.code.nullOrEmpty) {
			contentContainer.addText(reaction.code) => [
				associateWith(reaction)
				fontSize = 6
				noSelectionStyle()
				horizontalAlignment = HorizontalAlignment.LEFT
				verticalAlignment = VerticalAlignment.TOP
				setGridPlacementData().from(LEFT, 5, 0, TOP, order !== null ? 15 : 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
			]
		}
		
		if (reaction.deadline !== null) {
			baseShape.addPolygon() => [
				associateWith(reaction.deadline)
				
				points += #[
					createKPosition(PositionReferenceX.LEFT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0.5f),
					createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f),
					createKPosition(PositionReferenceX.RIGHT, REACTION_POINTINESS, 0, PositionReferenceY.BOTTOM, 0, 0),
					createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
				]
				
				// style
				lineWidth = 1
				foreground = Colors.GRAY_45
				background = Colors.BROWN
				
				// ensure content is rendered on top
				baseShape.children.move(0, it)
			]
			
			// Move order label if present
			order?.setPointPlacementData(LEFT, 0, 0.5f, TOP, 1, 0, H_CENTRAL, V_TOP, REACTION_POINTINESS, 0, minWidth - REACTION_POINTINESS * 2, 0)
				
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
				setGridPlacementData().from(LEFT, 5, 0, TOP, 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0).setHorizontalAlignment(HorizontalAlignment.CENTER)
				underlineSelectionStyle()
			]

			// optional code content
			if (SHOW_REACTION_CODE.booleanValue && !reaction.deadline.deadlineCode.nullOrEmpty) {
				deadlineSection.addText(reaction.deadline.deadlineCode) => [
					associateWith(reaction.deadline)
					fontSize = 6
					setGridPlacementData().from(LEFT, 5, 0, TOP, 0, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
					horizontalAlignment = HorizontalAlignment.LEFT
					noSelectionStyle()
				]
			}
		}

		return baseShape
	}
	
	/**
	 * Creates the visual representation of a timer node
	 */
	def addTimerFigure(KNode node, Timer timer) {
		node.setMinimalNodeSize(30, 30)
		
		val figure = node.addEllipse => [
			lineWidth = 1
			background = Colors.GRAY_95
			noSelectionStyle()
			boldLineSelectionStyle()
		]
		
		figure.addPolyline(1,
			#[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.1f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.7f, PositionReferenceY.TOP, 0 , 0.7f)
			]
		).boldLineSelectionStyle
		
		if (timer.timing !== null) {
			val labelParts = newArrayList
			if (timer.timing.offset !== null) {
				labelParts += timer.timing.offset.toText
			}
			if (timer.timing.period !== null) {
				labelParts += timer.timing.period.toText
			}
			if (!labelParts.empty) {
				node.addOutsideBottomCenteredNodeLabel(labelParts.join("(", ", ", ")")[it], 8)
			}
		}

		return figure
	}
	
	/**
	 * Creates the visual representation of a startup trigger.
	 */
	def addStartupFigure(KNode node) {
		node.setMinimalNodeSize(25, 25)
		
		val figure = node.addEllipse => [
			lineWidth = 1
			background = Colors.GRAY_65
			noSelectionStyle()
			boldLineSelectionStyle()
		]

		return figure
	}
	
	/**
	 * Creates the visual representation of a shutdown trigger.
	 */
	def addShutdownFigure(KNode node) {
		node.setMinimalNodeSize(25, 25)
		
		val figure = node.addPolygon => [
			lineWidth = 1
			background = Colors.WHITE
			noSelectionStyle
			boldLineSelectionStyle
		]
		figure.points += #[
			createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0),
			createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0 , 0.5f),
			createKPosition(PositionReferenceX.RIGHT, 0, 0.5f, PositionReferenceY.BOTTOM, 0 , 0),
			createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0 , 0.5f)
		]

		return figure
	}
	
	/**
	 * Creates the visual representation of a reactor port.
	 */
	def addTrianglePort(KPort port) {
		port.setSize(8, 8)
		port.addPolygon() => [
			lineWidth = 1
			boldLineSelectionStyle()
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
	def KText addTextButton(KContainerRendering container, String text) {
		container.addText(text) => [
			foreground = Colors.BLUE
			fontSize = 8
			noSelectionStyle()
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
                noSelectionStyle()
                
                // title
                if (title !== null) {
	                addText(title) => [
	                    fontSize = 12
	                    setFontBold = true
	                    foreground = Colors.RED
	                    setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
	                    suppressSelectability()
	                    noSelectionStyle()
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
                        noSelectionStyle()
	                ]
	            }
            ]
		]
	}

}