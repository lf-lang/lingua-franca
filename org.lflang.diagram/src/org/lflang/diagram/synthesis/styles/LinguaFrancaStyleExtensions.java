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

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KLabel;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KDecoratorPlacementData;
import de.cau.cs.kieler.klighd.krendering.KEllipse;
import de.cau.cs.kieler.klighd.krendering.KPolygon;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle;
import de.cau.cs.kieler.klighd.krendering.KSpline;
import de.cau.cs.kieler.klighd.krendering.KText;
import de.cau.cs.kieler.klighd.krendering.Underline;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.labels.decoration.IDecoratorRenderingProvider;
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator;
import java.util.List;

import javax.inject.Inject;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;

import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX.*;
import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY.*;

/**
 * Extension class that provides styles and coloring for the Lingua France diagram synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class LinguaFrancaStyleExtensions extends AbstractSynthesisExtensions {
    
    private static final Property<Colors> LABEL_PARENT_BACKGROUND = new Property<>(
            "org.lflang.linguafranca.diagram.synthesis.styles.label.parent.background", Colors.WHITE);
    
    @Inject
    @Extension
    private KRenderingExtensions _kRenderingExtensions;
    @Inject
    @Extension
    private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject
    @Extension
    private KPolylineExtensions _kPolylineExtensions;
    @Extension
    private KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;

    public KRendering noSelectionStyle(KRendering r) {
        return _kRenderingExtensions.setSelectionTextStrikeout(r, false);
    }
  
    public KRendering underlineSelectionStyle(KRendering r) {
        return _kRenderingExtensions.setSelectionTextUnderline(r, Underline.SINGLE);
    }
  
    public KRendering boldLineSelectionStyle(KRendering r) {
        float lineWidthValue = _kRenderingExtensions.getLineWidthValue(r);
        return _kRenderingExtensions.setSelectionLineWidth(r, lineWidthValue * 2);
    }
  
    public KText boldTextSelectionStyle(KText t) {
        return _kRenderingExtensions.setSelectionFontBold(t, true);
    }
    
    public void errorStyle(KRendering r) {
        _kRenderingExtensions.setForeground(r, Colors.RED);
        _kRenderingExtensions.setLineWidth(r, 2);
        _kRenderingExtensions.setSelectionLineWidth(r, 3);
        
        // Set background color the body if its a port or an line decorator
        if (r.eContainer() instanceof KPort || r.eContainer() instanceof KPolyline) {
            _kRenderingExtensions.setBackground(r, Colors.RED);
            _kRenderingExtensions.getBackground(r).setPropagateToChildren(true);
            _kRenderingExtensions.getForeground(r).setPropagateToChildren(true);
            _kRenderingExtensions.getLineWidth(r).setPropagateToChildren(true);
        } else if (r.eContainer() instanceof KEdge && r instanceof KPolyline) {
            // As a workaround for a rendering issue in Klighd VSCode, the style is applied to polyline
            // children directly because a propagated background would lead to a filled edge area.
            // See https://github.com/kieler/klighd-vscode/issues/67
            // If fixed this commit can be reverted
            ((KPolyline) r).getChildren().stream().forEach(c -> errorStyle(c));
        }
    }
    
    public void commentStyle(KRendering r) {
        _kRenderingExtensions.setForeground(r, Colors.LIGHT_GOLDENROD);
        _kRenderingExtensions.setBackground(r, Colors.PALE_GOLDENROD);
        _kRenderingExtensions.setLineWidth(r, 1);
        _kRenderingExtensions.setSelectionLineWidth(r, 2);

        if (r.eContainer() instanceof KEdge) {  // also color potential arrow heads
            _kRenderingExtensions.setBackground(r, Colors.LIGHT_GOLDENROD);
            _kRenderingExtensions.getBackground(r).setPropagateToChildren(true);
            _kRenderingExtensions.getForeground(r).setPropagateToChildren(true);
            _kRenderingExtensions.getLineWidth(r).setPropagateToChildren(true);
        }
    }
    
    private static final int CLOUD_WIDTH = 20;
    public KContainerRendering addCloudIcon(final KContainerRendering parent) {
        KRectangle figure = _kContainerRenderingExtensions.addRectangle(parent);
        _kRenderingExtensions.setInvisible(figure, true);
        
        KRoundedRectangle roundRectangle = _kContainerRenderingExtensions.addRoundedRectangle(
                figure, 
                CLOUD_WIDTH / 7, 
                CLOUD_WIDTH / 7
        );
        _kRenderingExtensions.setBackground(roundRectangle, Colors.GRAY);
        _kRenderingExtensions.setForeground(roundRectangle, Colors.GRAY);
        _kRenderingExtensions.setPointPlacementData(roundRectangle, 
                _kRenderingExtensions.LEFT, 2, 0, 
                _kRenderingExtensions.TOP, 0, 0.5f, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 0, 
                0, CLOUD_WIDTH, CLOUD_WIDTH / 3);
        
        KEllipse childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
        _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setPointPlacementData(childEllipse, 
                _kRenderingExtensions.LEFT, 0, 0f,
                _kRenderingExtensions.TOP, 0, 0.38f, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 0, 
                0, CLOUD_WIDTH / 2.5f, CLOUD_WIDTH / 2.5f);
        
        childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
        _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setPointPlacementData(childEllipse, 
                _kRenderingExtensions.LEFT, 0, 0.5f, 
                _kRenderingExtensions.TOP, 0, 0.25f, 
                _kRenderingExtensions.H_RIGHT, _kRenderingExtensions.V_TOP, 0, 
                0, CLOUD_WIDTH / 3f, CLOUD_WIDTH / 3f);
        
        childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
        _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
        _kRenderingExtensions.setPointPlacementData(childEllipse, 
                _kRenderingExtensions.LEFT, 0, 0.4f,
                _kRenderingExtensions.TOP, CLOUD_WIDTH / 10, 0, 
                _kRenderingExtensions.H_LEFT, _kRenderingExtensions.V_TOP, 0, 
                0, CLOUD_WIDTH / 2, CLOUD_WIDTH / 2);
        
        return figure;
    }
    
    public KRendering addCloudUploadIcon(KContainerRendering parent) {
        KContainerRendering cloudIcon = addCloudIcon(parent);
        KPolygon cloudPolygon = _kContainerRenderingExtensions.addPolygon(cloudIcon);
        _kRenderingExtensions.setBackground(cloudPolygon, Colors.WHITE);
        _kRenderingExtensions.setForeground(cloudPolygon, Colors.WHITE);
        cloudPolygon.getPoints().addAll(
            List.of(
                _kRenderingExtensions.createKPosition(LEFT, -1.5f, 0.5f, TOP, CLOUD_WIDTH / 3, 0.5f),
                _kRenderingExtensions.createKPosition(LEFT, -1.5f, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, (-4), 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0.35f),
                _kRenderingExtensions.createKPosition(LEFT, 4, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, 1.5f, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, 1.5f, 0.5f, TOP, CLOUD_WIDTH / 3, 0.5f)
            )
        );
        return cloudIcon;
    }
    
    private static LabelDecorationConfigurator _onEdgeLabelConfigurator; // ONLY for use in applyOnEdgeStyle
    public void applyOnEdgeStyle(KLabel label) {
        if (_onEdgeLabelConfigurator == null) {
            LabelDecorationConfigurator configurator = LabelDecorationConfigurator.create().withInlineLabels(true);
            _onEdgeLabelConfigurator = configurator.withLabelTextRenderingProvider(
                    (KContainerRendering container, KLabel klabel) -> {
                KText kText = _kRenderingFactory.createKText();
                _kRenderingExtensions.setFontSize(kText, 9);
                container.getChildren().add(kText);
                return kText;
            });
        }
        _onEdgeLabelConfigurator.applyTo(label);
    }
   
    private static LabelDecorationConfigurator _onEdgeDelayLabelConfigurator; // ONLY for use in applyOnEdgeDelayStyle
    public void applyOnEdgeDelayStyle(KLabel label) {
        if (_onEdgeDelayLabelConfigurator == null) {
            LabelDecorationConfigurator configurator = LabelDecorationConfigurator.create().withInlineLabels(true);
            configurator = configurator.withLabelTextRenderingProvider(
                (KContainerRendering container, KLabel klabel) -> {
                    KText kText = _kRenderingFactory.createKText();
                    _kRenderingExtensions.setFontSize(kText, 8);
                    boldTextSelectionStyle(kText);
                    kText.setProperty(KlighdInternalProperties.MODEL_ELEMEMT, 
                            klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT));
                    container.getChildren().add(kText);
                    return kText;
                }
            );
            configurator = configurator.addDecoratorRenderingProvider(new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(
                        KContainerRendering container, KLabel label, 
                        LabelDecorationConfigurator.LayoutMode layoutMode) {
                    ElkPadding padding =  new ElkPadding();
                    padding.top = 1;
                    padding.bottom = 1;
                    padding.left = 2;
                    padding.right = 2;
                    
                    KPolygon polygon = _kRenderingFactory.createKPolygon();
                    _kRenderingExtensions.from(polygon, LEFT, (-2), 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polygon, LEFT, 2, 0, TOP, 0, 0);
                    _kRenderingExtensions.to(polygon, RIGHT, (-2), 0, TOP, 0, 0);
                    _kRenderingExtensions.to(polygon, RIGHT, 2, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.setBackground(polygon, Colors.WHITE);
                    _kRenderingExtensions.setForeground(polygon, Colors.WHITE);
                    container.getChildren().add(polygon);
                    
                    KPolyline polyline = _kRenderingFactory.createKPolyline();
                    _kRenderingExtensions.from(polyline, LEFT, (-2), 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polyline, LEFT, 2, 0, TOP, 0, 0);
                    container.getChildren().add(polyline);
                    
                    polyline = _kRenderingFactory.createKPolyline();
                    _kRenderingExtensions.from(polyline, RIGHT, 2, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polyline, RIGHT, (-2), 0, TOP, 0, 0);
                    container.getChildren().add(polyline);
                    
                    return padding;
                }
            });
            _onEdgeDelayLabelConfigurator = configurator;
        }
        _onEdgeDelayLabelConfigurator.applyTo(label);
    }

    private static LabelDecorationConfigurator _onEdgePysicalDelayLabelConfigurator; // ONLY for use in applyOnEdgePysicalDelayStyle
    public void applyOnEdgePysicalDelayStyle(KLabel label, Colors parentBackgroundColor) {
        if (_onEdgePysicalDelayLabelConfigurator == null) {
            LabelDecorationConfigurator configurator = LabelDecorationConfigurator.create().withInlineLabels(true);
            configurator = configurator.withLabelTextRenderingProvider(
                (KContainerRendering container, KLabel klabel) -> {
                    KText kText = _kRenderingFactory.createKText();
                    _kRenderingExtensions.setFontSize(kText, 8);
                    boldTextSelectionStyle(kText);
                    kText.setProperty(KlighdInternalProperties.MODEL_ELEMEMT,
                            klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT));
                    container.getChildren().add(kText);
                    return kText;
                }
            );
            configurator = configurator.addDecoratorRenderingProvider(new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(
                        KContainerRendering container, 
                        KLabel label, 
                        LabelDecorationConfigurator.LayoutMode layoutMode) {
                    ElkPadding padding = new ElkPadding();
                    padding.top = 1;
                    padding.bottom = 1;
                    padding.left = 8;
                    padding.right = 16;
                    
                    KPolygon polygon = _kRenderingFactory.createKPolygon();
                    _kRenderingExtensions.from(polygon, LEFT, 0, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(polygon, LEFT, 0, 0, TOP, 1, 0.5f);
                    _kRenderingExtensions.to(polygon, RIGHT, 0, 0, TOP, 1, 0.5f);
                    _kRenderingExtensions.to(polygon, RIGHT, 0, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.setBackground(polygon, label.getProperty(LABEL_PARENT_BACKGROUND));
                    _kRenderingExtensions.setForeground(polygon, label.getProperty(LABEL_PARENT_BACKGROUND));
                    container.getChildren().add(polygon);
                    
                    KSpline kSpline = _kRenderingFactory.createKSpline();
                    _kRenderingExtensions.from(kSpline, LEFT, -0.66f, 0, BOTTOM, -0.5f, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 1, 0, BOTTOM, -0.5f, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 3, 0, BOTTOM, 8, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 5, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 5.5f, 0, BOTTOM, -1.5f, 0.5f);
                    container.getChildren().add(kSpline);
                    
                    kSpline = _kRenderingFactory.createKSpline();
                    _kRenderingExtensions.from(kSpline, RIGHT, 15f, 0, BOTTOM, 3.5f, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 14f, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 11, 0, BOTTOM, -8, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 9, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 7, 0, BOTTOM, 8, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 4f, 0, BOTTOM, 2, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 1.5f, 0, BOTTOM, 0.5f, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, 0.2f, 0, BOTTOM, -0.5f, 0.5f);
                    _kRenderingExtensions.to(kSpline, RIGHT, -0.7f, 0, BOTTOM, -0.5f, 0.5f);
                    container.getChildren().add(kSpline);
                    
                    polygon = _kRenderingFactory.createKPolygon();
                    _kRenderingExtensions.from(polygon, LEFT, 4, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polygon, LEFT, 8, 0, TOP, 0, 0);
                    _kRenderingExtensions.to(polygon, RIGHT, 12, 0, TOP, 0, 0);
                    _kRenderingExtensions.to(polygon, RIGHT, 16, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.setBackground(polygon, Colors.WHITE);
                    _kRenderingExtensions.setForeground(polygon, Colors.WHITE);
                    container.getChildren().add(polygon);
                    
                    KPolyline polyline = _kRenderingFactory.createKPolyline();
                    _kRenderingExtensions.from(polyline, LEFT, 4, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polyline, LEFT, 8, 0, TOP, 0, 0);
                    container.getChildren().add(polyline);
                    
                    polyline = _kRenderingFactory.createKPolyline();
                    _kRenderingExtensions.from(polyline, RIGHT, 16, 0, BOTTOM, 0, 0);
                    _kRenderingExtensions.to(polyline, RIGHT, 12, 0, TOP, 0, 0);
                    container.getChildren().add(polyline);
                    
                    return padding;
                }
            });
            _onEdgePysicalDelayLabelConfigurator = configurator;
        }
        label.setProperty(LABEL_PARENT_BACKGROUND, parentBackgroundColor);
        _onEdgePysicalDelayLabelConfigurator.applyTo(label);
    }
    
    private static LabelDecorationConfigurator _onEdgePysicalLabelConfigurator; // ONLY for use in applyOnEdgePysicalStyle
    public void applyOnEdgePysicalStyle(KLabel label, Colors parentBackgroundColor) {
        if (_onEdgePysicalLabelConfigurator == null) {
            LabelDecorationConfigurator configurator = LabelDecorationConfigurator.create().withInlineLabels(true);
            configurator = configurator.withLabelTextRenderingProvider(
                (KContainerRendering container, KLabel klabel) -> {
                    KText kText = _kRenderingFactory.createKText();
                    _kRenderingExtensions.setInvisible(kText, true);
                    container.getChildren().add(kText);
                    return kText;
                }
            );
            configurator = configurator.addDecoratorRenderingProvider(new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(final KContainerRendering container, final KLabel label, final LabelDecorationConfigurator.LayoutMode layoutMode) {
                    ElkPadding padding = new ElkPadding();
                    padding.top = 1;
                    padding.bottom = 1;
                    padding.left = 3;
                    padding.right = 3;
                    
                    KPolygon polygon = _kRenderingFactory.createKPolygon();
                    _kRenderingExtensions.from(polygon, LEFT, 0, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(polygon, LEFT, 0, 0, TOP, 1, 0.5f);
                    _kRenderingExtensions.to(polygon, RIGHT, 0, 0, TOP, 1, 0.5f);
                    _kRenderingExtensions.to(polygon, RIGHT, 0, 0, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.setBackground(polygon, label.getProperty(LABEL_PARENT_BACKGROUND));
                    _kRenderingExtensions.setForeground(polygon, label.getProperty(LABEL_PARENT_BACKGROUND));
                    container.getChildren().add(polygon);
                    
                    KSpline kSpline = _kRenderingFactory.createKSpline();
                    _kRenderingExtensions.from(kSpline, LEFT, (-0.66f), 0, BOTTOM, (-0.5f), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 1, 0, BOTTOM, (-0.5f), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.1f, BOTTOM, 8, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.2f, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.3f, BOTTOM, (-8), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.4f, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.45f, BOTTOM, 4f, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.5f, BOTTOM, 8, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.55f, BOTTOM, 4f, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.6f, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.65f, BOTTOM, (-4), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.7f, BOTTOM, (-8), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.8f, BOTTOM, (-4), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0, 0.9f, BOTTOM, 0, 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, (-1), 1, BOTTOM, (-0.5f), 0.5f);
                    _kRenderingExtensions.to(kSpline, LEFT, 0.66f, 1, BOTTOM, (-0.5f), 0.5f);
                    container.getChildren().add(kSpline);
                    return padding;
                }
            });
            _onEdgePysicalLabelConfigurator = configurator;
        }
        label.setProperty(LABEL_PARENT_BACKGROUND, parentBackgroundColor);
        _onEdgePysicalLabelConfigurator.applyTo(label);
    }

    public KRendering addFixedTailArrowDecorator(KPolyline pl) {
        KRendering head = _kPolylineExtensions.addTailArrowDecorator(pl);
        KDecoratorPlacementData placement = _kRenderingFactory.createKDecoratorPlacementData();
        placement.setRotateWithLine(true);
        placement.setRelative(0f);
        placement.setAbsolute(2f);
        placement.setWidth(8);
        placement.setHeight(6);
        placement.setXOffset(-3f);
        placement.setYOffset(-4f);
        head.setPlacementData(placement);
        return head;
    }
    
    public void addArrayDecorator(KEdge edge, Integer size) {
        final KRendering line = _kRenderingExtensions.getKRendering(edge);
        if (line instanceof KPolyline) {
            KDecoratorPlacementData placement = _kRenderingFactory.createKDecoratorPlacementData();
            placement.setRotateWithLine(true);
            placement.setRelative(0f);
            placement.setAbsolute(6f);
            
            KPolyline slash = _kContainerRenderingExtensions.addChild(
                    (KContainerRendering) line, 
                    _kRenderingFactory.createKPolyline());
            slash.getPoints().add(
            _kRenderingExtensions.createKPosition(
                _kRenderingExtensions.RIGHT, 0, 0, 
                _kRenderingExtensions.TOP, 0, 0)
            );
            slash.getPoints().add(
            _kRenderingExtensions.createKPosition(
                _kRenderingExtensions.LEFT, 0, 0, 
                _kRenderingExtensions.BOTTOM, 0, 0)
            );
            KDecoratorPlacementData slashPlacement = EcoreUtil.copy(placement);
            slashPlacement.setWidth(5);
            slashPlacement.setHeight(10);
            slashPlacement.setYOffset(-5f);
            slash.setPlacementData(slashPlacement);
            
            if (size != null) {
                KText num = _kContainerRenderingExtensions.addChild(
                    (KContainerRendering) line, 
                    _kRenderingFactory.createKText()
                );
                num.setText(size.toString());
                _kRenderingExtensions.setFontSize(num, 5);
                noSelectionStyle(num);
                KDecoratorPlacementData numPlacement = EcoreUtil.copy(placement);
                numPlacement.setXOffset(2f);
                num.setPlacementData(numPlacement);
            }
        }
    }
}