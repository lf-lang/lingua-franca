package org.lflang.diagram.synthesis.styles;

import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX.*;
import static de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY.*;

import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.SynthesisOption;
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
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;

/**
 * Extension class that provides styles and coloring for the Lingua Franca diagram synthesis.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
@ViewSynthesisShared
public class LinguaFrancaStyleExtensions extends AbstractSynthesisExtensions {

  /** INTERNAL property to communicate a node's background color. */
  public static final Property<Colors> LABEL_PARENT_BACKGROUND =
      new Property<>(
          "org.lflang.linguafranca.diagram.synthesis.styles.label.parent.background", Colors.WHITE);

  @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
  @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
  @Inject @Extension private KPolylineExtensions _kPolylineExtensions;
  @Extension private KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;

  public static final String SELECTION_HIGHLIGHTING_COLOR_LABEL = "Selection Coloring";

  public static final SynthesisOption SELECTION_HIGHLIGHTING_COLOR =
      SynthesisOption.createCheckOption(SELECTION_HIGHLIGHTING_COLOR_LABEL, false)
          .setCategory(LinguaFrancaSynthesis.APPEARANCE);

  public KRendering noSelectionStyle(KRendering r) {
    return _kRenderingExtensions.setSelectionTextStrikeout(r, false);
  }

  public KRendering underlineSelectionStyle(KRendering r) {
    return _kRenderingExtensions.setSelectionTextUnderline(r, Underline.SINGLE);
  }

  public KRendering boldLineSelectionStyle(KRendering r) {
    float lineWidthValue = _kRenderingExtensions.getLineWidthValue(r);
    // Improve this with content from https://github.com/lf-lang/rfcs/pull/3
    boolean selectionColor = getBooleanValue(SELECTION_HIGHLIGHTING_COLOR);
    if (selectionColor) {
      _kRenderingExtensions.setSelectionForeground(r, Colors.ORANGE_1);
    }
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
    _kRenderingExtensions.setForeground(r, Colors.GOLDENROD); // Formerly LIGHT_GOLDENROD
    _kRenderingExtensions.setBackground(r, Colors.WHITE); // Formerly PALE_GOLDENROD
    _kRenderingExtensions.setLineWidth(r, 1);
    _kRenderingExtensions.setSelectionLineWidth(r, 2);

    if (r.eContainer() instanceof KEdge) { // also color potential arrow heads
      _kRenderingExtensions.setBackground(r, Colors.LIGHT_GOLDENROD);
      _kRenderingExtensions.getBackground(r).setPropagateToChildren(true);
      _kRenderingExtensions.getForeground(r).setPropagateToChildren(true);
      _kRenderingExtensions.getLineWidth(r).setPropagateToChildren(true);
    }
  }

  public void maxWaitCommentStyle(KRendering r) {
    _kRenderingExtensions.setForeground(r, Colors.BLUE);
    _kRenderingExtensions.setBackground(r, Colors.WHITE);
    _kRenderingExtensions.setLineWidth(r, 1);
    _kRenderingExtensions.setSelectionLineWidth(r, 2);

    if (r.eContainer() instanceof KEdge) { // also color potential arrow heads
      _kRenderingExtensions.setBackground(r, Colors.LIGHT_BLUE);
      _kRenderingExtensions.getBackground(r).setPropagateToChildren(true);
      _kRenderingExtensions.getForeground(r).setPropagateToChildren(true);
      _kRenderingExtensions.getLineWidth(r).setPropagateToChildren(true);
    }
  }

  private static final int CLOUD_WIDTH = 20;

  public KContainerRendering addCloudIcon(final KContainerRendering parent) {
    KRectangle figure = _kContainerRenderingExtensions.addRectangle(parent);
    _kRenderingExtensions.setInvisible(figure, true);

    KRoundedRectangle roundRectangle =
        _kContainerRenderingExtensions.addRoundedRectangle(
            figure, CLOUD_WIDTH / 7, CLOUD_WIDTH / 7);
    _kRenderingExtensions.setBackground(roundRectangle, Colors.GRAY);
    _kRenderingExtensions.setForeground(roundRectangle, Colors.GRAY);
    _kRenderingExtensions.setPointPlacementData(
        roundRectangle,
        _kRenderingExtensions.LEFT,
        2,
        0,
        _kRenderingExtensions.TOP,
        0,
        0.5f,
        _kRenderingExtensions.H_LEFT,
        _kRenderingExtensions.V_TOP,
        0,
        0,
        CLOUD_WIDTH,
        CLOUD_WIDTH / 3);

    KEllipse childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
    _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setPointPlacementData(
        childEllipse,
        _kRenderingExtensions.LEFT,
        0,
        0f,
        _kRenderingExtensions.TOP,
        0,
        0.38f,
        _kRenderingExtensions.H_LEFT,
        _kRenderingExtensions.V_TOP,
        0,
        0,
        CLOUD_WIDTH / 2.5f,
        CLOUD_WIDTH / 2.5f);

    childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
    _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setPointPlacementData(
        childEllipse,
        _kRenderingExtensions.LEFT,
        0,
        0.5f,
        _kRenderingExtensions.TOP,
        0,
        0.25f,
        _kRenderingExtensions.H_RIGHT,
        _kRenderingExtensions.V_TOP,
        0,
        0,
        CLOUD_WIDTH / 3f,
        CLOUD_WIDTH / 3f);

    childEllipse = _kContainerRenderingExtensions.addEllipse(figure);
    _kRenderingExtensions.setBackground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setForeground(childEllipse, Colors.GRAY);
    _kRenderingExtensions.setPointPlacementData(
        childEllipse,
        _kRenderingExtensions.LEFT,
        0,
        0.4f,
        _kRenderingExtensions.TOP,
        CLOUD_WIDTH / 10,
        0,
        _kRenderingExtensions.H_LEFT,
        _kRenderingExtensions.V_TOP,
        0,
        0,
        CLOUD_WIDTH / 2,
        CLOUD_WIDTH / 2);

    return figure;
  }

  public KRendering addCloudUploadIcon(KContainerRendering parent) {
    KContainerRendering cloudIcon = addCloudIcon(parent);
    KPolygon cloudPolygon = _kContainerRenderingExtensions.addPolygon(cloudIcon);
    _kRenderingExtensions.setBackground(cloudPolygon, Colors.WHITE);
    _kRenderingExtensions.setForeground(cloudPolygon, Colors.WHITE);
    cloudPolygon
        .getPoints()
        .addAll(
            List.of(
                _kRenderingExtensions.createKPosition(
                    LEFT, -1.5f, 0.5f, TOP, CLOUD_WIDTH / 3, 0.5f),
                _kRenderingExtensions.createKPosition(LEFT, -1.5f, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, (-4), 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, 0, 0.5f, TOP, 0, 0.35f),
                _kRenderingExtensions.createKPosition(LEFT, 4, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(LEFT, 1.5f, 0.5f, TOP, 0, 0.58f),
                _kRenderingExtensions.createKPosition(
                    LEFT, 1.5f, 0.5f, TOP, CLOUD_WIDTH / 3, 0.5f)));
    return cloudIcon;
  }

  private static LabelDecorationConfigurator
      _onEdgeLabelConfigurator; // ONLY for use in applyOnEdgeStyle

  public void applyOnEdgeStyle(KLabel label) {
    if (_onEdgeLabelConfigurator == null) {
      LabelDecorationConfigurator configurator =
          LabelDecorationConfigurator.create().withInlineLabels(true);
      _onEdgeLabelConfigurator =
          configurator.withLabelTextRenderingProvider(
              (KContainerRendering container, KLabel klabel) -> {
                KText kText = _kRenderingFactory.createKText();
                _kRenderingExtensions.setFontSize(kText, 9);
                container.getChildren().add(kText);
                return kText;
              });
    }
    _onEdgeLabelConfigurator.applyTo(label);
  }

  private static LabelDecorationConfigurator
      _onEdgeDelayLabelConfigurator; // ONLY for use in applyOnEdgeDelayStyle

  public void applyOnEdgeDelayStyle(KLabel label) {
    if (_onEdgeDelayLabelConfigurator == null) {
      LabelDecorationConfigurator configurator =
          LabelDecorationConfigurator.create().withInlineLabels(true);
      configurator =
          configurator.withLabelTextRenderingProvider(
              (KContainerRendering container, KLabel klabel) -> {
                KText kText = _kRenderingFactory.createKText();
                _kRenderingExtensions.setFontSize(kText, 8);
                boldTextSelectionStyle(kText);
                kText.setProperty(
                    KlighdInternalProperties.MODEL_ELEMEMT,
                    klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT));
                container.getChildren().add(kText);
                return kText;
              });
      configurator =
          configurator.addDecoratorRenderingProvider(
              new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(
                    KContainerRendering container,
                    KLabel label,
                    LabelDecorationConfigurator.LayoutMode layoutMode) {
                  ElkPadding padding = new ElkPadding();
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

  /**
   * Draws the physical-connection "squiggle" directly on the edge's line as a decorator instead of
   * as an inline center label. Unlike an inline label, an on-line decorator follows the routed edge
   * path, so the squiggle stays on the connection line even when the edge is routed around a node
   * (such as a self-loop / feedback connection from a bank's output back to its own input, where
   * ELK does not honor inline label placement). The {@code parentBackgroundColor} is used to erase
   * the underlying straight line so the squiggle visually replaces it.
   *
   * @param edge the connection edge to decorate
   * @param parentBackgroundColor the background color behind the edge, used to mask the line
   * @param relative the position along the edge path (0..1) at which to center the squiggle
   */
  public void addPhysicalConnectionSquiggle(
      KEdge edge, Colors parentBackgroundColor, float relative) {
    final KRendering rendering = _kRenderingExtensions.getKRendering(edge);
    if (!(rendering instanceof KContainerRendering)) {
      return;
    }
    final KContainerRendering line = (KContainerRendering) rendering;

    // Erase the straight line underneath the squiggle so the squiggle replaces it rather than
    // being drawn on top of a visible straight segment.
    KPolygon erase = _kRenderingFactory.createKPolygon();
    _kRenderingExtensions.from(erase, LEFT, 0, 0, BOTTOM, (-3), 0.5f);
    _kRenderingExtensions.to(erase, LEFT, 0, 0, BOTTOM, 3, 0.5f);
    _kRenderingExtensions.to(erase, RIGHT, 0, 0, BOTTOM, 3, 0.5f);
    _kRenderingExtensions.to(erase, RIGHT, 0, 0, BOTTOM, (-3), 0.5f);
    _kRenderingExtensions.setBackground(erase, parentBackgroundColor);
    _kRenderingExtensions.setForeground(erase, parentBackgroundColor);
    line.getChildren().add(erase);
    erase.setPlacementData(createSquigglePlacement(relative));

    // The wavy line. Coordinates are interpreted relative to the decorator box (see
    // createSquigglePlacement); the vertical center (relative 0.5 from BOTTOM) lies on the edge
    // line, and the wave spans the box width.
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
    line.getChildren().add(kSpline);
    kSpline.setPlacementData(createSquigglePlacement(relative));
  }

  /**
   * Creates placement data that positions a squiggle decorator centered on a point along the edge
   * path and rotated to follow the line. A fresh instance is required per decorated rendering
   * because placement data is a containment feature.
   */
  private KDecoratorPlacementData createSquigglePlacement(float relative) {
    KDecoratorPlacementData placement = _kRenderingFactory.createKDecoratorPlacementData();
    placement.setRotateWithLine(true);
    placement.setRelative(relative);
    placement.setAbsolute(0f);
    placement.setWidth(14);
    placement.setHeight(18);
    // Center the box on the placement point: along the line (xOffset = -width/2) and perpendicular
    // to it (yOffset = -height/2), so the edge line passes through the box center.
    placement.setXOffset(-7f);
    placement.setYOffset(-9f);
    return placement;
  }

  private static LabelDecorationConfigurator
      _onEdgeLabelStyleConfigurator; // ONLY for use in applyOnEdgeLabelStyle

  public void applyOnEdgeLabelStyle(KLabel label) {
    if (_onEdgeLabelStyleConfigurator == null) {
      LabelDecorationConfigurator configurator =
          LabelDecorationConfigurator.create().withInlineLabels(true);
      configurator =
          configurator.withLabelTextRenderingProvider(
              (KContainerRendering container, KLabel klabel) -> {
                KText kText = _kRenderingFactory.createKText();
                _kRenderingExtensions.setFontSize(kText, 8);
                _kRenderingExtensions.setForeground(kText, Colors.GOLDENROD);
                boldTextSelectionStyle(kText);
                kText.setProperty(
                    KlighdInternalProperties.MODEL_ELEMEMT,
                    klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT));
                container.getChildren().add(kText);
                return kText;
              });
      configurator =
          configurator.addDecoratorRenderingProvider(
              new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(
                    KContainerRendering container,
                    KLabel label,
                    LabelDecorationConfigurator.LayoutMode layoutMode) {
                  ElkPadding padding = new ElkPadding();
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
                  _kRenderingExtensions.setForeground(polygon, Colors.GOLDENROD);
                  container.getChildren().add(polygon);

                  KPolyline polyline = _kRenderingFactory.createKPolyline();
                  _kRenderingExtensions.from(polyline, LEFT, (-2), 0, BOTTOM, 0, 0);
                  _kRenderingExtensions.to(polyline, LEFT, 2, 0, TOP, 0, 0);
                  _kRenderingExtensions.setForeground(polyline, Colors.GOLDENROD);
                  container.getChildren().add(polyline);

                  polyline = _kRenderingFactory.createKPolyline();
                  _kRenderingExtensions.from(polyline, RIGHT, 2, 0, BOTTOM, 0, 0);
                  _kRenderingExtensions.to(polyline, RIGHT, (-2), 0, TOP, 0, 0);
                  _kRenderingExtensions.setForeground(polyline, Colors.GOLDENROD);
                  container.getChildren().add(polyline);

                  return padding;
                }
              });
      _onEdgeLabelStyleConfigurator = configurator;
    }
    _onEdgeLabelStyleConfigurator.applyTo(label);
  }

  private static LabelDecorationConfigurator
      _onEdgeAbsentAfterStyleConfigurator; // ONLY for use in applyOnEdgeAbsentAfterStyle

  public void applyOnEdgeAbsentAfterStyle(KLabel label) {
    if (_onEdgeAbsentAfterStyleConfigurator == null) {
      LabelDecorationConfigurator configurator =
          LabelDecorationConfigurator.create().withInlineLabels(true);
      configurator =
          configurator.withLabelTextRenderingProvider(
              (KContainerRendering container, KLabel klabel) -> {
                KText kText = _kRenderingFactory.createKText();
                _kRenderingExtensions.setFontSize(kText, 6);
                // Text color defaults to black, matching comment box text style
                boldTextSelectionStyle(kText);
                kText.setProperty(
                    KlighdInternalProperties.MODEL_ELEMEMT,
                    klabel.getProperty(KlighdInternalProperties.MODEL_ELEMEMT));
                container.getChildren().add(kText);
                return kText;
              });
      configurator =
          configurator.addDecoratorRenderingProvider(
              new IDecoratorRenderingProvider() {
                @Override
                public ElkPadding createDecoratorRendering(
                    KContainerRendering container,
                    KLabel label,
                    LabelDecorationConfigurator.LayoutMode layoutMode) {
                  ElkPadding padding = new ElkPadding();
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
                  _kRenderingExtensions.setForeground(polygon, Colors.BLUE);
                  container.getChildren().add(polygon);

                  KPolyline polyline = _kRenderingFactory.createKPolyline();
                  _kRenderingExtensions.from(polyline, LEFT, (-2), 0, BOTTOM, 0, 0);
                  _kRenderingExtensions.to(polyline, LEFT, 2, 0, TOP, 0, 0);
                  _kRenderingExtensions.setForeground(polyline, Colors.BLUE);
                  container.getChildren().add(polyline);

                  polyline = _kRenderingFactory.createKPolyline();
                  _kRenderingExtensions.from(polyline, RIGHT, 2, 0, BOTTOM, 0, 0);
                  _kRenderingExtensions.to(polyline, RIGHT, (-2), 0, TOP, 0, 0);
                  _kRenderingExtensions.setForeground(polyline, Colors.BLUE);
                  container.getChildren().add(polyline);

                  return padding;
                }
              });
      _onEdgeAbsentAfterStyleConfigurator = configurator;
    }
    _onEdgeAbsentAfterStyleConfigurator.applyTo(label);
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

      KPolyline slash =
          _kContainerRenderingExtensions.addChild(
              (KContainerRendering) line, _kRenderingFactory.createKPolyline());
      slash
          .getPoints()
          .add(
              _kRenderingExtensions.createKPosition(
                  _kRenderingExtensions.RIGHT, 0, 0, _kRenderingExtensions.TOP, 0, 0));
      slash
          .getPoints()
          .add(
              _kRenderingExtensions.createKPosition(
                  _kRenderingExtensions.LEFT, 0, 0, _kRenderingExtensions.BOTTOM, 0, 0));
      KDecoratorPlacementData slashPlacement = EcoreUtil.copy(placement);
      slashPlacement.setWidth(5);
      slashPlacement.setHeight(10);
      slashPlacement.setYOffset(-5f);
      slash.setPlacementData(slashPlacement);

      if (size != null) {
        KText num =
            _kContainerRenderingExtensions.addChild(
                (KContainerRendering) line, _kRenderingFactory.createKText());
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
