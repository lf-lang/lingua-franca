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

import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX.LEFT;
import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX.RIGHT;
import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY.BOTTOM;
import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY.TOP;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;

import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.ASTUtils;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.util.UtilityExtensions;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;
import org.lflang.lf.Parameter;
import org.lflang.lf.StateVar;

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
import de.cau.cs.kieler.klighd.krendering.KPolygon;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KPosition;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle;
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

/**
 * Extension class that provides shapes and figures for the Lingua France diagram synthesis.
 * 
 * @author Alexander Schulz-Rosengarten
 */
@ViewSynthesisShared
public class LinguaFrancaShapeExtensions extends AbstractSynthesisExtensions {
    
    public static final float REACTION_POINTINESS = 6; // arrow point length 
    // Property for marking the KContainterRendering in Reactor figures that is supposed to hold the content
    public static final Property<Boolean> REACTOR_CONTENT_CONTAINER = new Property<>(
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
    public KRoundedRectangle addMainReactorFigure(KNode node, ReactorInstance reactorInstance, String text) {
        int padding = getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS) ? 8 : 6;
        KRoundedRectangle figure = _kRenderingExtensions.addRoundedRectangle(node, 8, 8, 1);
        _kContainerRenderingExtensions.setGridPlacement(figure, 1);
        _kRenderingExtensions.setLineWidth(figure, 1);
        _kRenderingExtensions.setForeground(figure, Colors.GRAY);
        _kRenderingExtensions.setBackground(figure, Colors.WHITE);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
        
        // Create parent container
        KRectangle parentContainer = _kContainerRenderingExtensions.addRectangle(figure);
        _kRenderingExtensions.setInvisible(parentContainer, true);
        setGridPlacementDataFromPointToPoint(parentContainer,
            LEFT, padding, 0, TOP, padding, 0,
            RIGHT, padding, 0, BOTTOM, 4, 0
        );
        
        // Create child container
        KRectangle childContainer = _kContainerRenderingExtensions.addRectangle(parentContainer);
        _kRenderingExtensions.setInvisible(childContainer, true);
        _kRenderingExtensions.setPointPlacementData(childContainer, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0, 
                0, 0, 0);
        KGridPlacement placement = _kContainerRenderingExtensions.setGridPlacement(childContainer, 1);
        
        // Add text to the child container
        KText childText = _kContainerRenderingExtensions.addText(childContainer, text);
        DiagramSyntheses.suppressSelectability(childText);
        _linguaFrancaStyleExtensions.underlineSelectionStyle(childText);
        
        if (reactorInstance.reactorDefinition.isFederated()) {
            KContainerRendering cloudIcon = _linguaFrancaStyleExtensions.addCloudIcon(childContainer);
            setGridPlacementDataFromPointToPoint(cloudIcon,
                LEFT, 3, 0, TOP, 0, 0,
                RIGHT, 0, 0, BOTTOM, 0, 0
            );
            placement.setNumColumns(2);
            
            if (reactorInstance.reactorDefinition.getHost() != null && 
                    getBooleanValue(LinguaFrancaSynthesis.SHOW_REACTOR_HOST)) {
                KText hostNameText = _kContainerRenderingExtensions.addText(childContainer, 
                        ASTUtils.toOriginalText(reactorInstance.reactorDefinition.getHost()));
                DiagramSyntheses.suppressSelectability(hostNameText);
                _linguaFrancaStyleExtensions.underlineSelectionStyle(hostNameText);
                setGridPlacementDataFromPointToPoint(hostNameText,
                    LEFT, 3, 0, TOP, 0, 0,
                    RIGHT, 0, 0, BOTTOM, 0, 0
                );
                placement.setNumColumns(3);
            }
        }        
        return figure;
    }

    /**
     * Creates the visual representation of a reactor node
     */
    public ReactorFigureComponents addReactorFigure(KNode node, ReactorInstance reactorInstance, String text) {
        int padding = getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS) ? 8 : 6;
       
        Function1<KRoundedRectangle, KRendering> style = r -> {
            _kRenderingExtensions.setLineWidth(r, 1);
            _kRenderingExtensions.setForeground(r, Colors.GRAY);
            _kRenderingExtensions.setBackground(r, Colors.GRAY_95);
            return _linguaFrancaStyleExtensions.boldLineSelectionStyle(r);
        };
        
        KRoundedRectangle figure = _kRenderingExtensions.addRoundedRectangle(node, 8, 8, 1);
        _kContainerRenderingExtensions.setGridPlacement(figure, 1);
        style.apply(figure);
        figure.setProperty(REACTOR_CONTENT_CONTAINER, true);

        // minimal node size is necessary if no text will be added
        List<Float> minSize = List.of(2 * figure.getCornerWidth(), 2 * figure.getCornerHeight());
        _kNodeExtensions.setMinimalNodeSize(node, minSize.get(0), minSize.get(1));
        
        // Add parent container
        KRectangle parentContainer = _kContainerRenderingExtensions.addRectangle(figure);
        _kRenderingExtensions.setInvisible(parentContainer, true);
        setGridPlacementDataFromPointToPoint(parentContainer,
                LEFT, padding, 0, 
                TOP, padding, 0,
                RIGHT, padding, 0, BOTTOM, 
                _utilityExtensions.hasContent(reactorInstance) ? 4 : padding, 0
        );
        
        // Add centered child container
        KRectangle childContainer = _kContainerRenderingExtensions.addRectangle(parentContainer);
        _kRenderingExtensions.setInvisible(childContainer, true);
        _kRenderingExtensions.setPointPlacementData(childContainer, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0, 
                0, 0, 0);
        KGridPlacement placement = _kContainerRenderingExtensions.setGridPlacement(childContainer, 1);
        
        KText childText = _kContainerRenderingExtensions.addText(childContainer, text);
        DiagramSyntheses.suppressSelectability(childText);
        _linguaFrancaStyleExtensions.underlineSelectionStyle(childText);
        
        if (!_utilityExtensions.isRoot(reactorInstance) && 
                reactorInstance.getDefinition().getHost() != null) {
            KRendering cloudUploadIcon = _linguaFrancaStyleExtensions.addCloudUploadIcon(childContainer);
            setGridPlacementDataFromPointToPoint(cloudUploadIcon,
                    LEFT, 3, 0, TOP, 0, 0,
                    RIGHT, 0, 0, BOTTOM, 0, 0
            );
            placement.setNumColumns(2);
            
            if (getBooleanValue(LinguaFrancaSynthesis.SHOW_REACTOR_HOST)) {
                KText reactorHostText = _kContainerRenderingExtensions.addText(childContainer, 
                        ASTUtils.toOriginalText(reactorInstance.getDefinition().getHost()));
                DiagramSyntheses.suppressSelectability(reactorHostText);
                _linguaFrancaStyleExtensions.underlineSelectionStyle(reactorHostText);
                setGridPlacementDataFromPointToPoint(reactorHostText,
                        LEFT, 3, 0, TOP, 0, 0,
                        RIGHT, 0, 0, BOTTOM, 0, 0
                );
                placement.setNumColumns(3);
            }
        }
        
        if (reactorInstance.isBank()) {
            List<KRendering> bank = new ArrayList<>();
            KContainerRendering container = _kRenderingExtensions.addInvisibleContainerRendering(node);
            // TODO handle unresolved width
            KRoundedRectangle banks;
            banks = _kContainerRenderingExtensions.addRoundedRectangle(container, 8, 8, 1);
            style.apply(banks);
            setGridPlacementDataFromPointToPoint(banks,
                LEFT, BANK_FIGURE_X_OFFSET_SUM, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM, 0,
                RIGHT, 0, 0, BOTTOM, 0, 0
            );
            if (reactorInstance.getWidth() == 3) {
                banks = _kContainerRenderingExtensions.addRoundedRectangle(container, 8, 8, 1);
                style.apply(banks);
                setGridPlacementDataFromPointToPoint(banks,
                    LEFT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM / 2, 0,
                    RIGHT, BANK_FIGURE_X_OFFSET_SUM / 2, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM / 2, 0
                );
            } else if (reactorInstance.getWidth() != 2 && reactorInstance.getWidth() != 3) {
                banks = _kContainerRenderingExtensions.addRoundedRectangle(container, 8, 8, 1);
                style.apply(banks);
                setGridPlacementDataFromPointToPoint(banks,
                    LEFT, 2 * BANK_FIGURE_X_OFFSET_SUM / 3, 0, TOP, 2 * BANK_FIGURE_Y_OFFSET_SUM / 3, 0,
                    RIGHT, BANK_FIGURE_X_OFFSET_SUM / 3, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM / 3, 0
                );
                
                banks = _kContainerRenderingExtensions.addRoundedRectangle(container, 8, 8, 1);
                style.apply(banks);
                setGridPlacementDataFromPointToPoint(banks,
                    LEFT, BANK_FIGURE_X_OFFSET_SUM / 3, 0, TOP, BANK_FIGURE_Y_OFFSET_SUM / 3, 0,
                    RIGHT, 2 * BANK_FIGURE_X_OFFSET_SUM / 3, 0, BOTTOM, 2 * BANK_FIGURE_Y_OFFSET_SUM / 3, 0
                );
            }
            
            container.getChildren().add(figure);
            setGridPlacementDataFromPointToPoint(figure,
                LEFT, 0, 0, TOP, 0, 0,
                RIGHT, BANK_FIGURE_X_OFFSET_SUM, 0, BOTTOM, BANK_FIGURE_Y_OFFSET_SUM, 0
            );
            bank.addAll(container.getChildren());
            
            KRectangle widthLabelContainer = _kContainerRenderingExtensions.addRectangle(container);
            _kRenderingExtensions.setInvisible(widthLabelContainer, true);
            setGridPlacementDataFromPointToPoint(widthLabelContainer,
                LEFT, 12, 0, BOTTOM, 9, 0,
                RIGHT, 6, 0, BOTTOM, 0.5f, 0
            );
            // Handle unresolved width.
            String widthLabel = reactorInstance.getWidth() >= 0 ? Integer.toString(reactorInstance.getWidth()) : "?";
            KText widthLabelText = _kContainerRenderingExtensions.addText(widthLabelContainer, widthLabel);
            _kRenderingExtensions.setHorizontalAlignment(widthLabelText, HorizontalAlignment.LEFT);
            _kRenderingExtensions.setVerticalAlignment(widthLabelText, VerticalAlignment.BOTTOM);
            _kRenderingExtensions.setFontSize(widthLabelText, 6);
            _linguaFrancaStyleExtensions.noSelectionStyle(widthLabelText);
            associateWith(widthLabelText, reactorInstance.getDefinition().getWidthSpec());
            return new ReactorFigureComponents(container, figure, bank);
        } else {
            return new ReactorFigureComponents(figure, figure, List.of(figure));
        }
    }
    
    /**
     * Creates the visual representation of a reaction node
     */
    public KPolygon addReactionFigure(KNode node, ReactionInstance reaction) {
        int minHeight = 22;
        int minWidth = 45;
        ReactorInstance reactor = reaction.getParent();
        _kNodeExtensions.setMinimalNodeSize(node, minWidth, minHeight);
        
        // Create base shape
        KPolygon baseShape = _kRenderingExtensions.addPolygon(node);
        associateWith(baseShape, reaction);
        _kRenderingExtensions.setLineWidth(baseShape, 1);
        _kRenderingExtensions.setForeground(baseShape, Colors.GRAY_45);
        _kRenderingExtensions.setBackground(baseShape, Colors.GRAY_65);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(baseShape);
        baseShape.getPoints().addAll(
            List.of(
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, TOP, 0, 0),
                _kRenderingExtensions.createKPosition(RIGHT, REACTION_POINTINESS, 0, TOP, 0, 0),
                _kRenderingExtensions.createKPosition(RIGHT, 0, 0, TOP, 0, 0.5f),
                _kRenderingExtensions.createKPosition(RIGHT, REACTION_POINTINESS, 0, BOTTOM, 0, 0),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0, 0),
                _kRenderingExtensions.createKPosition(LEFT, REACTION_POINTINESS, 0, BOTTOM, 0, 0.5f)
            )
        );
                
        KRectangle contentContainer = _kContainerRenderingExtensions.addRectangle(baseShape);
        associateWith(contentContainer, reaction);
        _kRenderingExtensions.setInvisible(contentContainer, true);
        _kRenderingExtensions.<KRectangle>setPointPlacementData(contentContainer, 
                _kRenderingExtensions.LEFT, REACTION_POINTINESS, 0, 
                _kRenderingExtensions.TOP, 0, 0, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, REACTION_POINTINESS, 
                0, minWidth - REACTION_POINTINESS * 2, minHeight);
        _kContainerRenderingExtensions.setGridPlacement(contentContainer, 1);
        
        if (reactor.reactions.size() > 1) {
            KText textToAdd = _kContainerRenderingExtensions.addText(contentContainer, 
                    Integer.toString(reactor.reactions.indexOf(reaction) + 1));
            _kRenderingExtensions.setFontBold(textToAdd, true);
            _linguaFrancaStyleExtensions.noSelectionStyle(textToAdd);
            DiagramSyntheses.suppressSelectability(textToAdd);
        }

        // optional reaction level
        if (getBooleanValue(LinguaFrancaSynthesis.SHOW_REACTION_LEVEL)) {
            // Force calculation of levels for reactions. This calculation
            // will only be done once. Note that if this fails due to a causality loop,
            // then some reactions will have level -1.
            try {
                String levels = IterableExtensions.join(reaction.getLevels(), ", ");
                KText levelsText = _kContainerRenderingExtensions.addText(contentContainer, ("level: " + levels));
                _kRenderingExtensions.setFontBold(levelsText, false);
                _linguaFrancaStyleExtensions.noSelectionStyle(levelsText);
                DiagramSyntheses.suppressSelectability(levelsText);
            } catch (Exception ex) {
                // If the graph has cycles, the above fails. Continue without showing levels.
            }
        }

        // optional code content
        boolean hasCode = getBooleanValue(LinguaFrancaSynthesis.SHOW_REACTION_CODE) && 
                !StringExtensions.isNullOrEmpty(reaction.getDefinition().getCode().getBody());
        if (hasCode) {
            KText hasCodeText = _kContainerRenderingExtensions.addText(contentContainer, 
                    _utilityExtensions.trimCode(reaction.getDefinition().getCode()));
            associateWith(hasCodeText, reaction);
            _kRenderingExtensions.setFontSize(hasCodeText, 6);
            _kRenderingExtensions.setFontName(hasCodeText, KlighdConstants.DEFAULT_MONOSPACE_FONT_NAME);
            _linguaFrancaStyleExtensions.noSelectionStyle(hasCodeText);
            _kRenderingExtensions.setHorizontalAlignment(hasCodeText, HorizontalAlignment.LEFT);
            _kRenderingExtensions.setVerticalAlignment(hasCodeText, VerticalAlignment.TOP);
            setGridPlacementDataFromPointToPoint(hasCodeText,
                    _kRenderingExtensions.LEFT, 5, 0, 
                    _kRenderingExtensions.TOP, 5, 0,
                    _kRenderingExtensions.RIGHT, 5, 0, 
                    _kRenderingExtensions.BOTTOM, 5, 0
            );
        }
        
        if (reaction.declaredDeadline != null) {
            boolean hasDeadlineCode = getBooleanValue(LinguaFrancaSynthesis.SHOW_REACTION_CODE) && 
                    !StringExtensions.isNullOrEmpty(reaction.getDefinition().getDeadline().getCode().getBody());
            if (hasCode || hasDeadlineCode) {
                KPolyline line = _kContainerRenderingExtensions.addHorizontalLine(contentContainer, 0);
                setGridPlacementDataFromPointToPoint(line,
                    _kRenderingExtensions.LEFT, 5, 0, 
                    _kRenderingExtensions.TOP, 3, 0,
                    _kRenderingExtensions.RIGHT, 5, 0, 
                    _kRenderingExtensions.BOTTOM, 6, 0
                );
            }
            
            // delay with stopwatch
            KRectangle labelContainer = _kContainerRenderingExtensions.addRectangle(contentContainer);
            _kRenderingExtensions.setInvisible(labelContainer, true);
            KRendering placement = setGridPlacementDataFromPointToPoint(labelContainer,
                _kRenderingExtensions.LEFT, hasDeadlineCode ? 0 : -REACTION_POINTINESS * 0.5f, 0,
                _kRenderingExtensions.TOP, 0, reactor.reactions.size() > 1 || hasCode || hasDeadlineCode ? 0 : 0.5f,
                _kRenderingExtensions.RIGHT, 0, 0,
                _kRenderingExtensions.BOTTOM, 0, 0
            );
            _kRenderingExtensions.setHorizontalAlignment(placement, HorizontalAlignment.LEFT);
            
            KRectangle stopWatchFigure = addStopwatchFigure(labelContainer);
            _kRenderingExtensions.setLeftTopAlignedPointPlacementData(stopWatchFigure, 0, 0, 0, 0);
            
            KText stopWatchText = _kContainerRenderingExtensions.addText(labelContainer, 
                    reaction.declaredDeadline.maxDelay.toString());
            associateWith(stopWatchText, reaction.getDefinition().getDeadline().getDelay());
            _kRenderingExtensions.setForeground(stopWatchText, Colors.BROWN);
            _kRenderingExtensions.setFontBold(stopWatchText, true);
            _kRenderingExtensions.setFontSize(stopWatchText, 7);
            _linguaFrancaStyleExtensions.underlineSelectionStyle(stopWatchText);
            _kRenderingExtensions.setLeftTopAlignedPointPlacementData(stopWatchText, 15, 0, 0, 0);
            
            // optional code content
            if (hasDeadlineCode) {
                KText contentContainerText = _kContainerRenderingExtensions.addText(contentContainer, 
                        _utilityExtensions.trimCode(reaction.getDefinition().getDeadline().getCode()));
                associateWith(contentContainerText, reaction.declaredDeadline);
                _kRenderingExtensions.setForeground(contentContainerText, Colors.BROWN);
                _kRenderingExtensions.setFontSize(contentContainerText, 6);
                _kRenderingExtensions.setFontName(contentContainerText, KlighdConstants.DEFAULT_MONOSPACE_FONT_NAME);
                setGridPlacementDataFromPointToPoint(contentContainerText,
                        _kRenderingExtensions.LEFT, 5, 0, 
                        _kRenderingExtensions.TOP, 0, 0,
                        _kRenderingExtensions.RIGHT, 5, 0, 
                        _kRenderingExtensions.BOTTOM, 5, 0
                );
                _kRenderingExtensions.setHorizontalAlignment(contentContainerText, HorizontalAlignment.LEFT);
                _linguaFrancaStyleExtensions.noSelectionStyle(contentContainerText);
            }
        }

        return baseShape;
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
                _kRenderingExtensions.createKPosition(LEFT, 3, 0.5f, TOP, (-2), 0),
                _kRenderingExtensions.createKPosition(LEFT, (-3), 0.5f, TOP, (-2), 0)
            )
        );
        _kRenderingExtensions.setForeground(polyline, Colors.BROWN);
        
        polyline = _kContainerRenderingExtensions.addPolyline(container, 2, 
            List.of(
                _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, (-2), 0),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 1, 0)
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
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0.1f),
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0.5f),
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.7f, TOP, 0, 0.7f)
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
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0),
            _kRenderingExtensions.createKPosition(RIGHT, 0, 0, TOP, 0, 0.5f),
            _kRenderingExtensions.createKPosition(RIGHT, 0, 0.5f, BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0, 0.5f)
        );
        
        figure.getPoints().addAll(pointsToAdd);
        return figure;
    }
    
    /**
     * Creates the visual representation of a shutdown trigger.
     */
    public KEllipse addResetFigure(KNode node) {
        _kNodeExtensions.setMinimalNodeSize(node, 18, 18);
        KEllipse figure = _kRenderingExtensions.addEllipse(node);
        _kRenderingExtensions.setLineWidth(figure, 1);
        _kRenderingExtensions.setBackground(figure, Colors.WHITE);
        _linguaFrancaStyleExtensions.noSelectionStyle(figure);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
        
        KEllipse resetCircle = _kContainerRenderingExtensions.addEllipse(figure);
        _kRenderingExtensions.setSurroundingSpace(resetCircle, 3f, 0);
        _kRenderingExtensions.setLineWidth(resetCircle, 1.5f);
        _kRenderingExtensions.setBackground(resetCircle, Colors.WHITE);
        _linguaFrancaStyleExtensions.noSelectionStyle(resetCircle);
        
        var resetCycleGap = _kContainerRenderingExtensions.addPolygon(resetCircle);
        resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0.0f, 0));
        resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0.0f, 0));
        resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 1.1f, 0, PositionReferenceY.BOTTOM, 0, 0));
        resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 1.1f, 0, PositionReferenceY.BOTTOM, 0, 0));
        _kRenderingExtensions.setLineWidth(resetCycleGap, 0.3f);
        _kRenderingExtensions.setForeground(resetCycleGap, Colors.WHITE);
        _kRenderingExtensions.setBackground(resetCycleGap, Colors.WHITE);
        _linguaFrancaStyleExtensions.noSelectionStyle(resetCycleGap);
        _kRenderingExtensions.setPointPlacementData(resetCycleGap, 
                _kRenderingExtensions.LEFT, -2, 0.5f, 
                _kRenderingExtensions.TOP, 1.5f, 0,
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 4.0f, 3.0f);
        
        var resetArrow = _kContainerRenderingExtensions.addPolygon(resetCircle);
        resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0.0f, 0.1f));
        resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0.0f, 0.3f, PositionReferenceY.BOTTOM, 0, 0));
        resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0.0f, 0, PositionReferenceY.TOP, 0.0f, 0.0f));
        _kRenderingExtensions.setLineWidth(resetArrow, 0.3f);
        _kRenderingExtensions.setBackground(resetArrow, Colors.BLACK);
        _linguaFrancaStyleExtensions.noSelectionStyle(resetArrow);
        _kRenderingExtensions.setPointPlacementData(resetArrow, 
                _kRenderingExtensions.LEFT, 1.0f, 0.5f, 
                _kRenderingExtensions.TOP, 1.8f, 0, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 3.2f, 3.2f);
        
        return figure;
    }
    
    /**
     * Creates the visual representation of a reactor port.
     */
    public KPolygon addTrianglePort(KPort port, boolean multiport) {
        port.setSize(8, 8);
        
        // Create triangle port
        KPolygon trianglePort = _kRenderingExtensions.addPolygon(port);
        
        // Set line width and background color according to multiport or not
        float lineWidth = multiport ? 2.2f : 1;
        _kRenderingExtensions.setLineWidth(trianglePort, lineWidth);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(trianglePort);
        Colors background = multiport ? Colors.WHITE : Colors.BLACK;
        _kRenderingExtensions.setBackground(trianglePort, background);
        
        List<KPosition> pointsToAdd;
        if (multiport) {
            // Compensate for line width by making triangle smaller
            // Do not adjust by port size because this will affect port distribution and cause offsets between parallel connections 
            pointsToAdd = List.of(
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, TOP, 0.6f, 0),
                _kRenderingExtensions.createKPosition(RIGHT, 1.2f, 0, TOP, 0, 0.5f),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0.6f, 0)
            );
        } else {
            pointsToAdd = List.of(
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, TOP, 0, 0),
                _kRenderingExtensions.createKPosition(RIGHT, 0, 0, TOP, 0, 0.5f),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0, 0)
            );
        }
        trianglePort.getPoints().addAll(pointsToAdd);
        return trianglePort;
    }
    
    /**
     * Added a text as collapse expand button.
     */
    public KText addTextButton(KContainerRendering container, String text) {
        KText textToAdd = _kContainerRenderingExtensions.addText(container, text);
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
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0),
            _kRenderingExtensions.createKPosition(RIGHT, 0, 0, BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0, 0)
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
            _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0),
            _kRenderingExtensions.createKPosition(RIGHT, 0, 0, BOTTOM, 0, 0),
            _kRenderingExtensions.createKPosition(LEFT, 0, 0, BOTTOM, 0, 0)
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
    
    private KRendering setGridPlacementDataFromPointToPoint(KRendering rendering,
            PositionReferenceX fPx, float fAbsoluteLR, float fRelativeLR,
            PositionReferenceY fPy, float fAbsoluteTB, float fRelativeTB,
            PositionReferenceX tPx, float tAbsoluteLR, float tRelativeLR,
            PositionReferenceY tPy, float tAbsoluteTB, float tRelativeTB) {
        KAreaPlacementData fromPoint = _kRenderingExtensions.from(
                _kRenderingExtensions.setGridPlacementData(rendering), 
                fPx, fAbsoluteLR, fRelativeLR,
                fPy, fAbsoluteTB, fRelativeTB);
        return _kRenderingExtensions.to(fromPoint, 
                tPx, tAbsoluteLR, tRelativeLR, 
                tPy, tAbsoluteTB, tRelativeTB);
    }
    
    
    public KPolyline addCommentPolyline(KEdge edge) {
        KPolyline polyline = _kEdgeExtensions.addPolyline(edge);
        _kRenderingExtensions.setLineWidth(polyline, 1);
        _kRenderingExtensions.setLineStyle(polyline, LineStyle.DOT);
        return polyline;
    }
    
    public KContainerRendering addParameterEntry(KContainerRendering parent, Parameter associate, String text) {
        KRectangle container = _kContainerRenderingExtensions.addRectangle(parent);
         _kRenderingExtensions.setInvisible(container, true);

        var ktext = _kContainerRenderingExtensions.addText(container, text);
        _kRenderingExtensions.setFontSize(ktext, 8);
        _kRenderingExtensions.setPointPlacementData(ktext, 
                _kRenderingExtensions.LEFT, 10, 0, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 0, 0);
        associateWith(ktext, associate);
        
        var dot = _kContainerRenderingExtensions.addEllipse(container);
        _kRenderingExtensions.setLineWidth(dot, 1);
        _linguaFrancaStyleExtensions.noSelectionStyle(dot);
        _kRenderingExtensions.setPointPlacementData(dot, 
                _kRenderingExtensions.LEFT, 2, 0, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 5, 5);

        return container;
    }
    
    
    public KContainerRendering addStateEntry(KContainerRendering parent, StateVar associate, String text, boolean reset) {
        KRectangle container = _kContainerRenderingExtensions.addRectangle(parent);
         _kRenderingExtensions.setInvisible(container, true);

        var ktext = _kContainerRenderingExtensions.addText(container, text);
        _kRenderingExtensions.setFontSize(ktext, 8);
        _kRenderingExtensions.setPointPlacementData(ktext, 
                _kRenderingExtensions.LEFT, 10, 0, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 0, 0);
        associateWith(ktext, associate);
        
        KEllipse outerCircle;
        
        if (reset) {
            outerCircle = _kContainerRenderingExtensions.addEllipse(container);
            _kRenderingExtensions.setLineWidth(outerCircle, 0.9f);
            _kRenderingExtensions.setBackground(outerCircle, Colors.WHITE);
            _linguaFrancaStyleExtensions.noSelectionStyle(outerCircle);
            _kRenderingExtensions.setPointPlacementData(outerCircle, 
                    _kRenderingExtensions.LEFT, 1.5f, 0, 
                    _kRenderingExtensions.TOP, 0, 0.5f, 
                    _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                    0, 0, 6.3f, 6.3f);
            
            var resetCycleGap = _kContainerRenderingExtensions.addPolygon(outerCircle);
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0.26f, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.2f, PositionReferenceY.TOP, 0.1f, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0.0f, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0.2f, PositionReferenceY.TOP, 0.1f, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0.26f, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0.5f, 0, PositionReferenceY.BOTTOM, 0, 0));
            resetCycleGap.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0.5f, 0, PositionReferenceY.BOTTOM, 0, 0));
            _kRenderingExtensions.setLineWidth(resetCycleGap, 0.3f);
            _kRenderingExtensions.setForeground(resetCycleGap, Colors.WHITE);
            _kRenderingExtensions.setBackground(resetCycleGap, Colors.WHITE);
            _linguaFrancaStyleExtensions.noSelectionStyle(resetCycleGap);
            _kRenderingExtensions.setPointPlacementData(resetCycleGap, 
                    _kRenderingExtensions.LEFT, -1.2f, 0.5f, 
                    _kRenderingExtensions.TOP, 0.75f, 0,
                    _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                    0, 0, 2.5f, 1.3f);
            
            var resetArrow = _kContainerRenderingExtensions.addPolygon(outerCircle);
            resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0.0f, 0.1f));
            resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.LEFT, 0.0f, 0.3f, PositionReferenceY.BOTTOM, 0, 0));
            resetArrow.getPoints().add(_kRenderingExtensions.createKPosition(PositionReferenceX.RIGHT, 0.0f, 0, PositionReferenceY.TOP, 0.0f, 0.0f));
            _kRenderingExtensions.setLineWidth(resetArrow, 0.3f);
            _kRenderingExtensions.setBackground(resetArrow, Colors.BLACK);
            _linguaFrancaStyleExtensions.noSelectionStyle(resetArrow);
            _kRenderingExtensions.setPointPlacementData(resetArrow, 
                    _kRenderingExtensions.LEFT, 0.8f, 0.5f, 
                    _kRenderingExtensions.TOP, 1.1f, 0, 
                    _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                    0, 0, 1.5f, 1.5f);
        } else {
            outerCircle = _kContainerRenderingExtensions.addEllipse(container);
            _kRenderingExtensions.setLineWidth(outerCircle, 1);
            _kRenderingExtensions.setBackground(outerCircle, Colors.WHITE);
            _linguaFrancaStyleExtensions.noSelectionStyle(outerCircle);
            _kRenderingExtensions.setPointPlacementData(outerCircle, 
                    _kRenderingExtensions.LEFT, 1.5f, 0, 
                    _kRenderingExtensions.TOP, 0, 0.5f, 
                    _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_CENTRAL, 
                    0, 0, 6, 6);
        }
        
        var innerDot = _kContainerRenderingExtensions.addEllipse(outerCircle);
        _kRenderingExtensions.setLineWidth(innerDot, 0.5f);
        _kRenderingExtensions.setBackground(innerDot, Colors.BLACK);
        _linguaFrancaStyleExtensions.noSelectionStyle(innerDot);
        _kRenderingExtensions.setPointPlacementData(innerDot, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 
                0, 0, 2.5f, 2.5f);

        return container;
    }

}
