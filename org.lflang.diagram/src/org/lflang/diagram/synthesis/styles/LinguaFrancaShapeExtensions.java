/*************
* Copyright (c) 2020, Kiel University.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.diagram.synthesis.styles;

import com.google.common.collect.Iterables;
import de.cau.cs.kieler.klighd.KlighdConstants;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.Arc;
import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment;
import de.cau.cs.kieler.klighd.krendering.KArc;
import de.cau.cs.kieler.klighd.krendering.KAreaPlacementData;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KDecoratorPlacementData;
import de.cau.cs.kieler.klighd.krendering.KEllipse;
import de.cau.cs.kieler.klighd.krendering.KGridPlacement;
import de.cau.cs.kieler.klighd.krendering.KGridPlacementData;
import de.cau.cs.kieler.klighd.krendering.KPolygon;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KPosition;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle;
import de.cau.cs.kieler.klighd.krendering.KStyle;
import de.cau.cs.kieler.klighd.krendering.KText;
import de.cau.cs.kieler.klighd.krendering.LineStyle;
import de.cau.cs.kieler.klighd.krendering.VerticalAlignment;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX;
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import javax.inject.Inject;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.emf.common.util.EList;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.ObjectExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure1;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.TimeValue;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment;
import org.lflang.diagram.synthesis.util.UtilityExtensions;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;

/**
 * Extension class that provides shapes and figures for the Lingua France diagram synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class LinguaFrancaShapeExtensions extends AbstractSynthesisExtensions {
	
    public static final float REACTION_POINTINESS = 6; // arrow point length 
    // Property for marking the KContainterRendering in Reactor figures that is supposed to hold the content
    public static final Property<Boolean> REACTOR_CONTENT_CONTAINER = new Property(
            "org.lflang.diagram.synthesis.shapes.reactor.content", false);
    
    @Inject
    @Extension
    private KNodeExtensions _kNodeExtensions;
    @Inject
    @Extension
    private KEdgeExtensions _kEdgeExtensions;
    @Inject
    @Extension
    private KPortExtensions _kPortExtensions;
    @Inject
    @Extension
    private KLabelExtensions _kLabelExtensions;
    @Inject
    @Extension
    private KRenderingExtensions _kRenderingExtensions;
    @Inject
    @Extension
    private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject
    @Extension
    private KPolylineExtensions _kPolylineExtensions;
    @Inject
    @Extension
    private KColorExtensions _kColorExtensions;
    @Inject
    @Extension
    private LinguaFrancaStyleExtensions _linguaFrancaStyleExtensions;
    @Inject
    @Extension
    private UtilityExtensions _utilityExtensions;
    @Extension
    private KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;
    
    public static final float BANK_FIGURE_X_OFFSET_SUM = 6.0f;
    public static final float BANK_FIGURE_Y_OFFSET_SUM = 9.0f;

	/**
	 * Creates the main reactor frame.
	 */
	def addMainReactorFigure(KNode node, ReactorInstance reactorInstance, String text) {
		val padding = SHOW_HYPERLINKS.booleanValue ? 8 : 6
		val figure = node.addRoundedRectangle(8, 8, 1) => [
			setGridPlacement(1)
			lineWidth = 1
			foreground = Colors.GRAY
			background = Colors.WHITE
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
				
				if (reactorInstance.reactorDefinition.federated) {
					addCloudIcon() => [
						setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
					]
					placement.numColumns = 2
					
					if (reactorInstance.reactorDefinition.host !== null && SHOW_REACTOR_HOST.booleanValue) {
						addText(reactorInstance.reactorDefinition.host.toText()) => [
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
	def ReactorFigureComponents addReactorFigure(KNode node, ReactorInstance reactorInstance, String text) {
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
			setProperty(REACTOR_CONTENT_CONTAINER, true)
		]

		// minimal node size is necessary if no text will be added
		val minSize = #[2 * figure.cornerWidth, 2 * figure.cornerHeight]
		node.setMinimalNodeSize(minSize.get(0), minSize.get(1))

		figure.addRectangle() => [
			invisible = true
			setGridPlacementData().from(LEFT, padding, 0, TOP, padding, 0).to(RIGHT, padding, 0, BOTTOM, reactorInstance.hasContent ? 4 : padding, 0)
			
			addRectangle() => [ // Centered child container
				invisible = true
				setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, 0, 0)
				val placement = setGridPlacement(1)
				
				addText(text) => [
					suppressSelectability
					underlineSelectionStyle
				]
				
				if (!reactorInstance.isRoot && reactorInstance.definition.host !== null) {
					addCloudUploadIcon() => [
						setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
					]
					placement.numColumns = 2
					
					if (SHOW_REACTOR_HOST.booleanValue) {
						addText(reactorInstance.definition.host.toText()) => [
							suppressSelectability
							underlineSelectionStyle
							setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
						]
						placement.numColumns = 3
					}
				}
			]
		]
		
        if (reactorInstance.isBank()) {
            val bank = newArrayList
            val container = node.addInvisibleContainerRendering => [
                // TODO handle unresolved width
                addRoundedRectangle(8, 8, 1) => [
                    style.apply(it)
                    setAreaPlacementData().from(LEFT, BANK_FIGURE_X_OFFSET_SUM, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
                ]
                if (reactorInstance.width === 3) {
                    addRoundedRectangle(8, 8, 1) => [
                        style.apply(it)
                        setAreaPlacementData().from(LEFT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM / 2, 0).to(RIGHT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM / 2, 0)
                    ]
                } else if (reactorInstance.width !== 2 && reactorInstance.width !== 3) {
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
                    // Handle unresolved width.
                    val widthLabel = (reactorInstance.width >= 0)?
                            Integer.toString(reactorInstance.width)
                            : "?"
                    // addText(instance.widthSpec.toText) => [
                    addText(widthLabel) => [
                        horizontalAlignment = HorizontalAlignment.LEFT
                        verticalAlignment = VerticalAlignment.BOTTOM
                        fontSize = 6
                        noSelectionStyle
                        associateWith(reactorInstance.definition.widthSpec)
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
	def addReactionFigure(KNode node, ReactionInstance reaction) {
		val minHeight = 22
		val minWidth = 45
		val reactor = reaction.parent
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

        // optional reaction level
        if (SHOW_REACTION_LEVEL.booleanValue) {
            // Force calculation of levels for reactions. This calculation
            // will only be done once. Note that if this fails due to a causality loop,
            // then some reactions will have level -1.
            try {
                val levels = reaction.getLevels().join(", ");
                contentContainer.addText("level: " + levels) => [
                    fontBold = false
                    noSelectionStyle
                    suppressSelectability
                ]
            } catch (Exception ex) {
                // If the graph has cycles, the above fails. Continue without showing levels.
            }
        }

		// optional code content
		val hasCode = SHOW_REACTION_CODE.booleanValue && !reaction.definition.code.body.nullOrEmpty
		if (hasCode) {
			contentContainer.addText(reaction.definition.code.trimCode) => [
				associateWith(reaction)
				fontSize = 6
				fontName = KlighdConstants.DEFAULT_MONOSPACE_FONT_NAME
				noSelectionStyle()
				horizontalAlignment = HorizontalAlignment.LEFT
				verticalAlignment = VerticalAlignment.TOP
				setGridPlacementData().from(LEFT, 5, 0, TOP, 5, 0).to(RIGHT, 5, 0, BOTTOM, 5, 0)
			]
		}
		
		// TODO improve default check
		if (reaction.declaredDeadline !== null) {
			val hasDeadlineCode = SHOW_REACTION_CODE.booleanValue && !reaction.definition.deadline.code.body.nullOrEmpty
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
			labelContainer.addText(reaction.declaredDeadline.maxDelay.toString) => [
				associateWith(reaction.definition.deadline.delay)
				foreground = Colors.BROWN
				fontBold = true
				fontSize = 7
				underlineSelectionStyle()
				setLeftTopAlignedPointPlacementData(15, 0, 0, 0)
			]

			// optional code content
			if (hasDeadlineCode) {
				contentContainer.addText(reaction.definition.deadline.code.trimCode) => [
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
	public KRectangle addStopwatchFigure(KContainerRendering parent) {
	    final int size = 12;
	    KRectangle container = _kContainerRenderingExtensions.addRectangle(parent);
	    _kRenderingExtensions.setInvisible(container, true);
	    _kRenderingExtensions.setPointPlacementData(container, 
	            _kRenderingExtensions.LEFT, 0, 0, 
	            _kRenderingExtensions.TOP, 0, 0, 
	            _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 0, 
	            0, size, size);
	    
	    KPolyline polyline = _kContainerRenderingExtensions.addPolyline(container, 2, 
	        List.of(
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 3, 0.5f, PositionReferenceY.TOP, (-2), 0),
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, (-3), 0.5f, PositionReferenceY.TOP, (-2), 0)
            )
	    );
	    _kRenderingExtensions.setForeground(polyline, Colors.BROWN);
	    
	    polyline = _kContainerRenderingExtensions.addPolyline(container, 2, 
            List.of(
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, (-2), 0),
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 1, 0)
            )
        );
        _kRenderingExtensions.setForeground(polyline, Colors.BROWN);

        KEllipse body = _kContainerRenderingExtensions.addEllipse(container);
        _kRenderingExtensions.setLineWidth(body, 1);
        _kRenderingExtensions.setForeground(body, Colors.BROWN);
        _kRenderingExtensions.<KEllipse>setPointPlacementData(body, 
                _kRenderingExtensions.LEFT, 0, 0, 
                _kRenderingExtensions.TOP, 0, 0, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 0, 
                0, size, size);
        _linguaFrancaStyleExtensions.noSelectionStyle(body);
        
        KArc arc = _kContainerRenderingExtensions.addArc(body);
        arc.setStartAngle((-20));
        arc.setArcAngle(110);
        arc.setArcType(Arc.PIE);
        _kRenderingExtensions.setLineWidth(arc, 0);
        _kRenderingExtensions.setBackground(arc, Colors.BROWN);
        _kRenderingExtensions.setPointPlacementData(arc, 
                _kRenderingExtensions.LEFT, 2, 0, 
                _kRenderingExtensions.TOP, 2, 0, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 2, 
                2, size - 4, size - 4);
        _linguaFrancaStyleExtensions.noSelectionStyle(arc);
	
		return container;
	}
	
	/**
	 * Creates the visual representation of a timer node
	 */
	public KEllipse addTimerFigure(KNode node, TimerInstance timer) {
	    _kNodeExtensions.setMinimalNodeSize(node, 30, 30);
	    
	    KEllipse figure = _kRenderingExtensions.addEllipse(node);
	    _kRenderingExtensions.setBackground(figure, Colors.GRAY_95);
	    _linguaFrancaStyleExtensions.noSelectionStyle(figure);
	    _kRenderingExtensions.setLineWidth(figure, 1);
	    _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
		
	    List<KPosition> polylinePoints = List.of(
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0.1f),
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0.5f),
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.7f, PositionReferenceY.TOP, 0, 0.7f)
        );
	    KPolyline polyline = _kContainerRenderingExtensions.addPolyline(figure, 1, polylinePoints);
	    _linguaFrancaStyleExtensions.boldLineSelectionStyle(polyline);
	    
	    List<String> labelParts = new ArrayList<>();
	    if (timer.getOffset() != TimerInstance.DEFAULT_OFFSET && timer.getOffset() != null) {
	        labelParts.add(timer.getOffset().toString());
	    }
	    if (timer.getPeriod() != TimerInstance.DEFAULT_PERIOD && timer.getPeriod() != null) {
	        if (timer.getOffset() == TimerInstance.DEFAULT_OFFSET) {
	            labelParts.add(timer.getOffset().toString());
	        }
	        labelParts.add(timer.getPeriod().toString());
	    }
	    if (!labelParts.isEmpty()) {
	        _kLabelExtensions.addOutsideBottomCenteredNodeLabel(node, 
                "(" + String.join(", ", labelParts) + ")", 8);
	    }
		return figure;
	}
	
	/**
	 * Creates the visual representation of a startup trigger.
	 */
	public KEllipse addStartupFigure(KNode node) {
	    _kNodeExtensions.setMinimalNodeSize(node, 18, 18);
	    KEllipse figure = _kRenderingExtensions.addEllipse(node);
	    _kRenderingExtensions.setLineWidth(figure, 1);
	    _kRenderingExtensions.setBackground(figure, Colors.WHITE);
	    _linguaFrancaStyleExtensions.noSelectionStyle(figure);
	    _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
		return figure;
	}
	
	/**
	 * Creates the visual representation of a shutdown trigger.
	 */
	public KPolygon addShutdownFigure(KNode node) {
	    _kNodeExtensions.setMinimalNodeSize(node, 18, 18);
	    KPolygon figure = _kRenderingExtensions.addPolygon(node);
	    _kRenderingExtensions.setLineWidth(figure, 1);
        _kRenderingExtensions.setBackground(figure, Colors.WHITE);
        _linguaFrancaStyleExtensions.noSelectionStyle(figure);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
        
        List<KPosition> pointsToAdd = List.of(
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f),
            _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0.5f, PositionReferenceY.BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0.5f)
        );
        
        figure.getPoints().addAll(pointsToAdd);
		return figure;
	}
	
	/**
	 * Creates the visual representation of a reactor port.
	 */
	public KPolygon addTrianglePort(KPort port, boolean multiport) {
	    port.setSize(8, 8);
	    
	    // Create triangle port
	    KPolygon trianglePort = this._kRenderingExtensions.addPolygon(port);
	    
	    // Set line width and background color according to multiport or not
	    float lineWidth = multiport ? 2.2f : 1;
	    this._kRenderingExtensions.setLineWidth(trianglePort, lineWidth);
        this._linguaFrancaStyleExtensions.boldLineSelectionStyle(trianglePort);
	    Colors background = multiport ? Colors.WHITE : Colors.BLACK;
	    
	    List<KPosition> pointsToAdd;
	    if (multiport) {
            // Compensate for line width by making triangle smaller
            // Do not adjust by port size because this will affect port distribution and cause offsets between parallel connections 
	        pointsToAdd = List.of(
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0.6f, 0),
                _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 1.2f, 0, PositionReferenceY.TOP, 0, 0.5f),
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0.6f, 0)
            );
	    } else {
	        pointsToAdd = List.of(
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0, 0),
                _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0, 0.5f),
                _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
            );
	    }
	    trianglePort.getPoints().addAll(pointsToAdd);
	    return trianglePort;
	}
	
	/**
	 * Added a text as collapse expand button.
	 */
	public KText addTextButton(KContainerRendering container, String text) {
	    KText textToAdd = this._kContainerRenderingExtensions.addText(container, text);
	    _kRenderingExtensions.setForeground(textToAdd, Colors.BLUE);
	    _kRenderingExtensions.setFontSize(textToAdd, 8);
	    _linguaFrancaStyleExtensions.noSelectionStyle(textToAdd);
	    return textToAdd;
	}
	
	/** 
	 * Creates the triangular line decorator with text.
	 */
	public KPolygon addActionDecorator(KPolyline line, String text) {
	    final float size = 18;
	    
	    // Create action decorator
	    KPolygon actionDecorator = _kContainerRenderingExtensions.addPolygon(line);
	    _kRenderingExtensions.setBackground(actionDecorator, Colors.WHITE);
	    List<KPosition> pointsToAdd = List.of(
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
        );
        actionDecorator.getPoints().addAll(pointsToAdd);
        
        // Set placement data of the action decorator
        KDecoratorPlacementData placementData = _kRenderingFactory.createKDecoratorPlacementData();
        placementData.setRelative(0.5f);
        placementData.setAbsolute(-size / 2);
        placementData.setWidth(size);
        placementData.setHeight(size);
        placementData.setYOffset(-size * 0.66f);
        placementData.setRotateWithLine(true);
        actionDecorator.setPlacementData(placementData);
        
        // Add text to the action decorator
        KText textToAdd = _kContainerRenderingExtensions.addText(actionDecorator, text);
        _kRenderingExtensions.setFontSize(textToAdd, 8);
        _linguaFrancaStyleExtensions.noSelectionStyle(textToAdd);
        DiagramSyntheses.suppressSelectability(textToAdd);
        _kRenderingExtensions.setPointPlacementData(textToAdd, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, size * 0.15f, 0.5f, 
                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0,
                0, size, size);
        
        return actionDecorator;
	}
	
	/**
	 * Creates the triangular action node with text and ports.
	 */
	public Pair<KPort, KPort> addActionFigureAndPorts(KNode node, String text) {
		final float size = 18;
		_kNodeExtensions.setMinimalNodeSize(node, size, size);
		KPolygon figure = _kRenderingExtensions.addPolygon(node);
		_kRenderingExtensions.setBackground(figure, Colors.WHITE);
	    _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
	  
	    List<KPosition> pointsToAdd = List.of(
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0, 0)
        );
	    figure.getPoints().addAll(pointsToAdd);
	    
	    // Add text to the action figure
	    KText textToAdd = _kContainerRenderingExtensions.addText(figure, text);
	    _kRenderingExtensions.setFontSize(textToAdd, 8);
        _linguaFrancaStyleExtensions.noSelectionStyle(textToAdd);
        DiagramSyntheses.suppressSelectability(textToAdd);
        _kRenderingExtensions.setPointPlacementData(textToAdd, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, (size * 0.15f), 0.5f, 
                _kRenderingExtensions.H_CENTRAL, 
                _kRenderingExtensions.V_CENTRAL, 0, 0, size, size);
		
        // Add input port
        KPort in = _kPortExtensions.createPort();
        node.getPorts().add(in);
        in.setSize(0, 0);
        DiagramSyntheses.setLayoutOption(in, CoreOptions.PORT_SIDE, PortSide.WEST);
        DiagramSyntheses.setLayoutOption(in, CoreOptions.PORT_BORDER_OFFSET, -size / ((double) 4));

        // Add output port
        KPort out = _kPortExtensions.createPort();
        node.getPorts().add(out);
        DiagramSyntheses.setLayoutOption(out, CoreOptions.PORT_SIDE, PortSide.EAST);
        DiagramSyntheses.setLayoutOption(out, CoreOptions.PORT_BORDER_OFFSET, -size / ((double) 4));
        return new Pair<KPort, KPort>(in, out);
	}
	
	/**
	 * Creates and adds an error message figure
	 */
	public KRectangle addErrorMessage(KNode node, String title, String message) {
	    // Create figure for error message
	    KRectangle figure = _kRenderingExtensions.addRectangle(node);
	    _kRenderingExtensions.setInvisible(figure, true);
	    
	    // Add error message box
	    KRoundedRectangle errMsgBox = _kContainerRenderingExtensions.addRoundedRectangle(figure, 7, 7);
	    _kContainerRenderingExtensions.setGridPlacement(errMsgBox, 1);
	    _kRenderingExtensions.setLineWidth(errMsgBox, 2);
        _linguaFrancaStyleExtensions.noSelectionStyle(errMsgBox);
        
        if (title != null) {
            // Add title to error message box
            KText titleText = _kContainerRenderingExtensions.addText(errMsgBox, title);
            _kRenderingExtensions.setFontSize(titleText, 12);
            _kRenderingExtensions.setFontBold(titleText, true);
            _kRenderingExtensions.setForeground(titleText, Colors.RED);
            setGridPlacementDataFromPointToPoint(titleText,
                    _kRenderingExtensions.LEFT, 8, 0, 
                    _kRenderingExtensions.TOP, 8, 0,
                    _kRenderingExtensions.RIGHT, 8, 0, 
                    _kRenderingExtensions.BOTTOM, 4, 0);
            DiagramSyntheses.suppressSelectability(titleText);
            _linguaFrancaStyleExtensions.noSelectionStyle(titleText);
        }
        
        if (message != null) {
            // Add message to error message box
            KText msgText = _kContainerRenderingExtensions.addText(errMsgBox, message);
            if (title != null) {
                setGridPlacementDataFromPointToPoint(msgText,
                        _kRenderingExtensions.LEFT, 8, 0, 
                        _kRenderingExtensions.TOP, 0, 0,
                        _kRenderingExtensions.RIGHT, 8, 0, 
                        _kRenderingExtensions.BOTTOM, 4, 0);
            } else {
                setGridPlacementDataFromPointToPoint(msgText,
                        _kRenderingExtensions.LEFT, 8, 0, 
                        _kRenderingExtensions.TOP, 8, 0,
                        _kRenderingExtensions.RIGHT, 8, 0, 
                        _kRenderingExtensions.BOTTOM, 8, 0);
            }
            _linguaFrancaStyleExtensions.noSelectionStyle(msgText);
        }
	    return figure;
	}
	
    public KRoundedRectangle addCommentFigure(KNode node, String message) {
        // Create rectangle for comment figure
        KRoundedRectangle commentFigure = _kRenderingExtensions.addRoundedRectangle(node, 1, 1, 1);
        _kContainerRenderingExtensions.setGridPlacement(commentFigure, 1);
        
        // Add message
        KText text = _kContainerRenderingExtensions.addText(commentFigure, message);
        _kRenderingExtensions.setFontSize(text, 6);
        setGridPlacementDataFromPointToPoint(text,
                _kRenderingExtensions.LEFT, 3, 0,
                _kRenderingExtensions.TOP, 3, 0,
                _kRenderingExtensions.RIGHT, 3, 0, 
                _kRenderingExtensions.BOTTOM, 3, 0);
        _linguaFrancaStyleExtensions.noSelectionStyle(text);
        return commentFigure;
    }
    
    private void setGridPlacementDataFromPointToPoint(KRendering rendering,
            PositionReferenceX fPx, float fAbsoluteLR, float fRelativeLR,
            PositionReferenceY fPy, float fAbsoluteTB, float fRelativeTB,
            PositionReferenceX tPx, float tAbsoluteLR, float tRelativeLR,
            PositionReferenceY tPy, float tAbsoluteTB, float tRelativeTB) {
        KAreaPlacementData fromPoint = _kRenderingExtensions.from(
                _kRenderingExtensions.setGridPlacementData(rendering), 
                fPx, fAbsoluteLR, fRelativeLR,
                fPy, fAbsoluteTB, fRelativeTB);
        _kRenderingExtensions.to(fromPoint, 
                tPx, tAbsoluteLR, tRelativeLR, 
                tPy, tAbsoluteTB, tRelativeTB);
    }
    
    public KPolyline addCommentPolyline(KEdge edge) {
        KPolyline polyline = _kEdgeExtensions.addPolyline(edge);
        _kRenderingExtensions.setLineWidth(polyline, 1);
        _kRenderingExtensions.setLineStyle(polyline, LineStyle.DOT);
        return polyline;
    }

}