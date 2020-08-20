package org.icyphy.linguafranca.diagram.synthesis.styles

import de.cau.cs.kieler.klighd.KlighdConstants
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Arc
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KPolyline
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle
import de.cau.cs.kieler.klighd.krendering.KText
import de.cau.cs.kieler.klighd.krendering.LineStyle
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
import java.util.List
import javax.inject.Inject
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.PortSide
import org.eclipse.xtend.lib.annotations.Data
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguafranca.diagram.synthesis.AbstractSynthesisExtensions
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesisUtilityExtensions
import org.icyphy.linguafranca.diagram.synthesis.postprocessor.ReactionPortAdjustment

import static org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis.*

import static extension de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses.*
import static extension org.icyphy.ASTUtils.*

/**
 * Extension class that provides shapes and figures for the Lingua France diagram synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
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
	
    extension KRenderingFactory = KRenderingFactory.eINSTANCE
    
    public static val BANK_FIGURE_X_OFFSET_SUM = 6.0f
    public static val BANK_FIGURE_Y_OFFSET_SUM = 9.0f

	/**
	 * Creates the main reactor frame.
	 */
	def addMainReactorFigure(KNode node, Reactor reactor, String text) {
		val padding = SHOW_HYPERLINKS.booleanValue ? 8 : 6
		val figure = node.addRoundedRectangle(8, 8, 1) => [
			setGridPlacement(1)
			lineWidth = 1
			foreground = Colors.GRAY
			boldLineSelectionStyle
		]
		
		figure.addRectangle() => [
			invisible = true
			setGridPlacementData().from(LEFT, padding, 0, TOP, padding, 0).to(RIGHT, padding, 0, BOTTOM, 4, 0)
			
			addRectangle() => [ // Centered child container
				invisible = true
				setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, 0, 0)
				val placement = setGridPlacement(1)
				
				addText(text) => [
					suppressSelectability
					underlineSelectionStyle
				]
				
				if (reactor.federated) {
					addCloudIcon() => [
						setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
					]
					placement.numColumns = 2
					
					if (reactor.host !== null && SHOW_REACTOR_HOST.booleanValue) {
						addText(reactor.host.toText()) => [
							suppressSelectability
							underlineSelectionStyle
							setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
						]
						placement.numColumns = 3
					}
				}
			]
		]
		
		return figure
	}

	/**
	 * Creates the visual representation of a reactor node
	 */
	def ReactorFigureComponents addReactorFigure(KNode node, Reactor reactor, Instantiation instance, String text) {
		val padding = SHOW_HYPERLINKS.booleanValue ? 8 : 6
		val style = [ KRoundedRectangle r |
            r.lineWidth = 1
            r.foreground = Colors.GRAY
            r.background = Colors.GRAY_95
            r.boldLineSelectionStyle
		]
		val figure = node.addRoundedRectangle(8, 8, 1) => [
			setGridPlacement(1)
			style.apply(it)
		]

		// minimal node size is necessary if no text will be added
		val minSize = #[2 * figure.cornerWidth, 2 * figure.cornerHeight]
		node.setMinimalNodeSize(minSize.get(0), minSize.get(1))

		figure.addRectangle() => [
			invisible = true
			setGridPlacementData().from(LEFT, padding, 0, TOP, padding, 0).to(RIGHT, padding, 0, BOTTOM, reactor.hasContent ? 4 : padding, 0)
			
			addRectangle() => [ // Centered child container
				invisible = true
				setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, 0, 0)
				val placement = setGridPlacement(1)
				
				addText(text) => [
					suppressSelectability
					underlineSelectionStyle
				]
				
				if (instance !== null && instance.host !== null) {
					addCloudUploadIcon() => [
						setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
					]
					placement.numColumns = 2
					
					if (SHOW_REACTOR_HOST.booleanValue) {
						addText(instance.host.toText()) => [
							suppressSelectability
							underlineSelectionStyle
							setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
						]
						placement.numColumns = 3
					}
				}
			]
		]
		
        if (instance.isBank()) {
            val bank = newArrayList
            val container = node.addInvisibleContainerRendering => [
                val bankWidth = instance.widthSpec.width
                addRoundedRectangle(8, 8, 1) => [
                    style.apply(it)
                    setAreaPlacementData().from(LEFT, BANK_FIGURE_X_OFFSET_SUM, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
                ]
                if (bankWidth === 3) {
                    addRoundedRectangle(8, 8, 1) => [
                        style.apply(it)
                        setAreaPlacementData().from(LEFT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM / 2, 0).to(RIGHT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM / 2, 0)
                    ]
                } else if (bankWidth !== 2 && bankWidth !== 3) {
                    addRoundedRectangle(8, 8, 1) => [
                        style.apply(it)
                        setAreaPlacementData().from(LEFT, 2 * BANK_FIGURE_X_OFFSET_SUM / 3, 0, TOP, 2 * BANK_FIGURE_Y_OFFSET_SUM / 3, 0).to(RIGHT, BANK_FIGURE_X_OFFSET_SUM / 3, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM / 3, 0)
                    ]
                    addRoundedRectangle(8, 8, 1) => [
                        style.apply(it)
                        setAreaPlacementData().from(LEFT, BANK_FIGURE_X_OFFSET_SUM / 3, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM / 3, 0).to(RIGHT, 2 * BANK_FIGURE_X_OFFSET_SUM / 3, 0, BOTTOM, 2 * BANK_FIGURE_Y_OFFSET_SUM / 3, 0)
                    ]
                }
                
                children += figure // move figure into invisible container (add last to be on top)
                figure.setAreaPlacementData().from(LEFT, 0, 0, TOP, 0, 0).to(RIGHT, BANK_FIGURE_X_OFFSET_SUM, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM, 0)
                bank.addAll(children)
                
                addRectangle() => [
                    invisible = true
                    setAreaPlacementData().from(LEFT, 12, 0, BOTTOM, 9, 0).to(RIGHT, 6, 0, BOTTOM, 0.5f, 0)
                    addText(instance.widthSpec.toText) => [
                        horizontalAlignment = HorizontalAlignment.LEFT
                        verticalAlignment = VerticalAlignment.BOTTOM
                        fontSize = 6
                        noSelectionStyle
                        associateWith(instance.widthSpec)
                    ]
                ]
            ]
            return new ReactorFigureComponents(container, figure, bank)
        } else {
            return new ReactorFigureComponents(figure, figure, #[figure])
        }
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
			
			styles.head.modifierId = ReactionPortAdjustment.ID // This hack will adjust the port position after layout
		]
		
		val contentContainer = baseShape.addRectangle() => [
			associateWith(reaction)
			invisible = true
			setPointPlacementData(LEFT, REACTION_POINTINESS, 0, TOP, 0, 0, H_LEFT, V_TOP, REACTION_POINTINESS, 0, minWidth - REACTION_POINTINESS * 2, minHeight)
			gridPlacement = 1
		]
		
		if (reactor.reactions.size > 1) {
			contentContainer.addText(Integer.toString(reactor.reactions.indexOf(reaction) + 1)) => [
				fontBold = true
				noSelectionStyle
				suppressSelectability
			]
		}

		// optional code content
		val hasCode = SHOW_REACTION_CODE.booleanValue && !reaction.code.body.nullOrEmpty
		if (hasCode) {
			contentContainer.addText(reaction.code.trimCode) => [
				associateWith(reaction)
				fontSize = 6
				fontName = KlighdConstants.DEFAULT_MONOSPACE_FONT_NAME
				noSelectionStyle()
				horizontalAlignment = HorizontalAlignment.LEFT
				verticalAlignment = VerticalAlignment.TOP
				setGridPlacementData().from(LEFT, 5, 0, TOP, 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
			]
		}
		
		if (reaction.deadline !== null) {
			val hasDeadlineCode = SHOW_REACTION_CODE.booleanValue && !reaction.deadline.code.body.nullOrEmpty
			if (hasCode || hasDeadlineCode) {
				contentContainer.addHorizontalLine(0) => [
					setGridPlacementData().from(LEFT, 5, 0, TOP, 3, 0).to(RIGHT, 5, 0, BOTTOM, 6, 0)
				]
			}
				
			// delay with stopwatch
			val labelContainer = contentContainer.addRectangle() => [
				invisible = true
				setGridPlacementData().from(LEFT, hasDeadlineCode ? 0 : -REACTION_POINTINESS * 0.5f, 0, TOP, 0, reactor.reactions.size > 1 || hasCode || hasDeadlineCode ? 0 : 0.5f).to(RIGHT, 0, 0, BOTTOM, 0, 0).setHorizontalAlignment(HorizontalAlignment.LEFT)
			]
			labelContainer.addStopwatchFigure() => [
				setLeftTopAlignedPointPlacementData(0, 0, 0, 0)
			]
			labelContainer.addText(reaction.deadline.delay.toText) => [
				associateWith(reaction.deadline.delay)
				foreground = Colors.BROWN
				fontBold = true
				fontSize = 7
				underlineSelectionStyle()
				setLeftTopAlignedPointPlacementData(15, 0, 0, 0)
			]

			// optional code content
			if (hasDeadlineCode) {
				contentContainer.addText(reaction.deadline.code.trimCode) => [
					associateWith(reaction.deadline)
					foreground = Colors.BROWN
					fontSize = 6
					fontName = KlighdConstants.DEFAULT_MONOSPACE_FONT_NAME
					setGridPlacementData().from(LEFT, 5, 0, TOP, 0, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
					horizontalAlignment = HorizontalAlignment.LEFT
					noSelectionStyle()
				]
			}
		}

		return baseShape
	}
	
	/**
	 * Stopwatch figure for deadlines.
	 */
	def addStopwatchFigure(KContainerRendering parent) {
		val size = 12
		val container = parent.addRectangle() => [
			invisible = true
			setPointPlacementData(LEFT, 0, 0, TOP, 0, 0, H_LEFT, V_TOP, 0, 0, size, size)
		]
		container.addPolyline(2,
			#[
				createKPosition(PositionReferenceX.LEFT, 3, 0.5f, PositionReferenceY.TOP, -2, 0),
				createKPosition(PositionReferenceX.LEFT, -3, 0.5f, PositionReferenceY.TOP, -2, 0)
			]
		) => [
			foreground = Colors.BROWN
		]
		container.addPolyline(2,
			#[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, -2, 0),
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 1, 0)
			]
		) => [
			foreground = Colors.BROWN
		]
		val body = container.addEllipse() => [
			lineWidth = 1
			foreground = Colors.BROWN
			setPointPlacementData(LEFT, 0, 0, TOP, 0, 0, H_LEFT, V_TOP, 0, 0, size, size)
			noSelectionStyle()
		]
		body.addArc() => [
			startAngle = -20
			arcAngle = 110
			arcType = Arc.PIE
			lineWidth = 0
			background = Colors.BROWN
			setPointPlacementData(LEFT, 2, 0, TOP, 2, 0, H_LEFT, V_TOP, 2, 2, size - 4, size - 4)
			noSelectionStyle()
		]
		
		return container
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
		
		val labelParts = newArrayList
		if (timer.offset !== null) {
			labelParts += timer.offset.toText
		}
		if (timer.period !== null) {
			labelParts += timer.period.toText
		}
		if (!labelParts.empty) {
			node.addOutsideBottomCenteredNodeLabel(labelParts.join("(", ", ", ")")[it], 8)
		}

		return figure
	}
	
	/**
	 * Creates the visual representation of a startup trigger.
	 */
	def addStartupFigure(KNode node) {
		node.setMinimalNodeSize(18, 18)
		
		val figure = node.addEllipse => [
			lineWidth = 1
			background = Colors.WHITE
			noSelectionStyle()
			boldLineSelectionStyle()
		]

		return figure
	}
	
	/**
	 * Creates the visual representation of a shutdown trigger.
	 */
	def addShutdownFigure(KNode node) {
		node.setMinimalNodeSize(18, 18)
		
		val figure = node.addPolygon => [
			lineWidth = 1
			background = Colors.WHITE
			noSelectionStyle()
			boldLineSelectionStyle()
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
	def addTrianglePort(KPort port, boolean multiport) {
	    if (multiport) {
	        port.setSize(7, 7) // compensate for line width
	    } else {
	        port.setSize(8, 8)
	    }
		port.addPolygon() => [
			lineWidth = multiport ? 2.2f : 1
			boldLineSelectionStyle()
			background = multiport ? Colors.WHITE : Colors.BLACK
			points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0 , 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0 , 0)
			]
		]
	}
	
	/**
	 * Reverses the arrow direction of a triangle port.
	 */
	def reverseTrianglePort(KPort port) {
		val poly = port.KRendering
		if (poly instanceof KPolyline) {
			poly.points.clear
			poly.points += #[
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0 , 0),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0 , 0)
			]
		}
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
	 * Creates the triangular line decorator with text.
	 */
	def addActionDecorator(KPolyline line, String text) {
		val float size = 18
        line.addPolygon() => [
            background = Colors.WHITE
            
            points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
			]
            
            placementData = createKDecoratorPlacementData => [
                relative = 0.5f
                absolute = -size / 2
                width = size
                height = size
                setYOffset(-size * 0.66f)
                rotateWithLine = true
            ]
            
            addText(text) => [
				fontSize = 8
				noSelectionStyle()
				suppressSelectability()
				
				setPointPlacementData(LEFT, 0, 0.5f, TOP, size * 0.15f, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, size, size)
			]
        ]
	}
	
	/**
	 * Creates the triangular action node with text and ports.
	 */
	def Pair<KPort, KPort> addActionFigureAndPorts(KNode node, String text) {
		val float size = 18
		node.setMinimalNodeSize(size, size)
		
		val figure = node.addPolygon() => [
            background = Colors.WHITE
            boldLineSelectionStyle
            
            points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
			]
            
            addText(text) => [
				fontSize = 8
				noSelectionStyle()
				suppressSelectability()
				
				setPointPlacementData(LEFT, 0, 0.5f, TOP, size * 0.15f, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, size, size)
			]
		]
		
		val in = createPort
		node.ports += in
		in.setSize(0, 0) // invisible
		in.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.WEST)
		in.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, - size / 4 as double)
		
		val out = createPort
		node.ports += out
		out.setSize(0, 0) // invisible
		out.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.EAST)
		out.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, - size / 4 as double)

		return new Pair(in, out)
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
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
                        } else {
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
                        }
                        noSelectionStyle()
	                ]
	            }
            ]
		]
	}
	
    def KRoundedRectangle addCommentFigure(KNode node, String message) {
        node.addRoundedRectangle(1, 1, 1) => [
        	gridPlacement = 1
            addText(message) => [
            	fontSize = 6
            	setGridPlacementData().from(LEFT, 3, 0, TOP, 3, 0).to(RIGHT, 3, 0, BOTTOM, 3, 0)
            	noSelectionStyle()
            ]
        ]
    }
    
    def KPolyline addCommentPolyline(KEdge edge) {
        edge.addPolyline => [
            lineWidth = 1
        	lineStyle = LineStyle.DOT
        ]
    }

}

@Data
class ReactorFigureComponents {
    val KContainerRendering outer
    val KContainerRendering reactor
    val List<KRendering> figures
}