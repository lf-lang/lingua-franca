package org.lflang.diagram.synthesis.action;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.KAreaPlacementData;
import de.cau.cs.kieler.klighd.krendering.KGridPlacement;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KPosition;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle;
import de.cau.cs.kieler.klighd.krendering.KText;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;
import java.util.WeakHashMap;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.properties.Property;

/**
 * Action that toggles displaying a CAL (Consistency, Availability, Latency) feedback comment box
 * when the deadline clock is double-clicked. The comment box shows analysis results from the CAL
 * Theorem and is dynamically created/removed to avoid layout space reservation.
 *
 * @author Shaokai Lin
 * @ingroup Diagram
 */
public class ShowCaLFeedbackAction extends AbstractAction {

  public static final String ID = "org.lflang.diagram.synthesis.action.ShowCaLFeedbackAction";

  /** Property to store the CAL feedback message on a reaction node. */
  public static final Property<String> CAL_FEEDBACK_MESSAGE =
      new Property<>("org.lflang.diagram.synthesis.action.calfeedback.message", null);

  /** Property to mark a comment node as a CAL feedback comment. */
  public static final Property<Boolean> CAL_FEEDBACK_COMMENT =
      new Property<>("org.lflang.diagram.synthesis.action.calfeedback.comment", false);

  /** Property on reaction node pointing to its CAL feedback comment node. */
  public static final Property<KNode> CAL_FEEDBACK_COMMENT_NODE =
      new Property<>("org.lflang.diagram.synthesis.action.calfeedback.commentnode", null);

  /** Cache of created comment nodes */
  private static final WeakHashMap<KNode, KNode> COMMENT_NODES = new WeakHashMap<>();

  @Override
  public IAction.ActionResult execute(final IAction.ActionContext context) {
    KGraphElement element = context.getKGraphElement();

    // Find the reaction node that was clicked
    KNode reactionNode = findParentNode(element);
    if (reactionNode == null) {
      return IAction.ActionResult.createResult(false);
    }

    // Get the CAL feedback message from the reaction node
    String message = reactionNode.getProperty(CAL_FEEDBACK_MESSAGE);
    if (message == null) {
      return IAction.ActionResult.createResult(false);
    }

    // Check if comment already exists
    KNode existingComment = COMMENT_NODES.get(reactionNode);

    if (existingComment != null) {
      // Remove the existing comment
      removeComment(existingComment, reactionNode);
      COMMENT_NODES.remove(reactionNode);
    } else {
      // Create and add new CAL feedback comment
      KNode comment = createCaLFeedbackComment(reactionNode, message);
      COMMENT_NODES.put(reactionNode, comment);
    }

    return IAction.ActionResult.createResult(true);
  }

  /** Creates a CAL feedback comment box and connects it to the reaction node. */
  private KNode createCaLFeedbackComment(KNode reactionNode, String message) {
    KGraphFactory kGraphFactory = KGraphFactory.eINSTANCE;
    KRenderingFactory kRenderingFactory = KRenderingFactory.eINSTANCE;

    // Create comment node
    KNode comment = kGraphFactory.createKNode();
    DiagramSyntheses.setLayoutOption(comment, CoreOptions.COMMENT_BOX, true);

    // Create comment figure (rounded rectangle with text)
    KRoundedRectangle commentFigure = kRenderingFactory.createKRoundedRectangle();
    commentFigure.setCornerWidth(1);
    commentFigure.setCornerHeight(1);
    comment.getData().add(commentFigure);

    // Set line width
    var lineWidth = kRenderingFactory.createKLineWidth();
    lineWidth.setLineWidth(1);
    commentFigure.getStyles().add(lineWidth);

    // Set foreground color (blue)
    var foreground = kRenderingFactory.createKForeground();
    foreground.setColor(kRenderingFactory.createKColor());
    foreground.getColor().setRed(255);
    foreground.getColor().setGreen(255);
    foreground.getColor().setBlue(255);
    commentFigure.getStyles().add(foreground);

    // Set background color (light blue)
    var background = kRenderingFactory.createKBackground();
    background.setColor(kRenderingFactory.createKColor());
    background.getColor().setRed(173);
    background.getColor().setGreen(216);
    background.getColor().setBlue(230);
    commentFigure.getStyles().add(background);

    // Set up grid placement for padding
    KGridPlacement gridPlacement = kRenderingFactory.createKGridPlacement();
    gridPlacement.setNumColumns(1);
    commentFigure.setChildPlacement(gridPlacement);

    // Add text with padding
    KText text = kRenderingFactory.createKText();
    text.setText(message);
    var fontSize = kRenderingFactory.createKFontSize();
    fontSize.setSize(6);
    text.getStyles().add(fontSize);

    // Add padding around text (5 pixels on all sides)
    KAreaPlacementData placementData = kRenderingFactory.createKGridPlacementData();
    KPosition topLeft = kRenderingFactory.createKPosition();
    topLeft.setX(kRenderingFactory.createKLeftPosition());
    topLeft.getX().setAbsolute(5); // left padding
    topLeft.setY(kRenderingFactory.createKTopPosition());
    topLeft.getY().setAbsolute(5); // top padding
    placementData.setTopLeft(topLeft);

    KPosition bottomRight = kRenderingFactory.createKPosition();
    bottomRight.setX(kRenderingFactory.createKRightPosition());
    bottomRight.getX().setAbsolute(5); // right padding
    bottomRight.setY(kRenderingFactory.createKBottomPosition());
    bottomRight.getY().setAbsolute(5); // bottom padding
    placementData.setBottomRight(bottomRight);

    text.setPlacementData(placementData);
    commentFigure.getChildren().add(text);

    // Mark as CAL feedback comment
    comment.setProperty(CAL_FEEDBACK_COMMENT, true);

    // Add to parent (same parent as reaction node)
    KNode parent = reactionNode.getParent();
    if (parent != null) {
      parent.getChildren().add(comment);
    }

    // Create edge connecting comment to reaction
    KEdge edge = kGraphFactory.createKEdge();
    edge.setSource(comment);
    edge.setTarget(reactionNode);

    // Create polyline rendering for edge
    KPolyline polyline = kRenderingFactory.createKPolyline();

    // Set line width for edge
    var edgeLineWidth = kRenderingFactory.createKLineWidth();
    edgeLineWidth.setLineWidth(1);
    polyline.getStyles().add(edgeLineWidth);

    // Set edge foreground color (blue)
    var edgeForeground = kRenderingFactory.createKForeground();
    edgeForeground.setColor(kRenderingFactory.createKColor());
    edgeForeground.getColor().setRed(0);
    edgeForeground.getColor().setGreen(0);
    edgeForeground.getColor().setBlue(255);
    polyline.getStyles().add(edgeForeground);

    edge.getData().add(polyline);

    return comment;
  }

  /** Removes a comment node and its connecting edge. */
  private void removeComment(KNode comment, KNode reactionNode) {
    // Remove all edges from this comment
    comment.getOutgoingEdges().clear();
    comment.getIncomingEdges().clear();

    // Remove from parent
    KNode parent = comment.getParent();
    if (parent != null) {
      parent.getChildren().remove(comment);
    }
  }

  /** Find the parent KNode by traversing up the hierarchy. */
  private KNode findParentNode(KGraphElement element) {
    if (element instanceof KNode) {
      return (KNode) element;
    }

    // Try to find parent node through the container hierarchy
    if (element != null && element.eContainer() != null) {
      Object container = element.eContainer();
      while (container != null) {
        if (container instanceof KNode) {
          return (KNode) container;
        }
        if (container instanceof org.eclipse.emf.ecore.EObject) {
          container = ((org.eclipse.emf.ecore.EObject) container).eContainer();
        } else {
          break;
        }
      }
    }

    return null;
  }
}

